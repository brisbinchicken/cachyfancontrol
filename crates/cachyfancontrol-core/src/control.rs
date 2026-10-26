//! Fan control channel loop, calibration mapping, avoid-region jumping, zero-RPM kickstart,
//! and asymmetric slew-rate limiting reverse-engineered from `FanControl.Domain.Control` (`0x0200005e`).

use crate::calibration::{Calibration, CalibrationCheck, CommandCheck};
use crate::curves::CommandMode;
use serde::{Deserialize, Serialize};

const SELECTED_START_TIMEOUT_TICKS: i32 = 6;
const KICKSTART_BOOST_STEP_PCT: f32 = 3.0;

/// Serializable configuration for a controllable PWM/fan channel (`IControlConfig`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlConfig {
    pub id: String,
    pub name: String,
    pub hardware_id: String,
    pub enabled: bool,
    pub manual_control: bool,
    pub manual_control_value: f32,
    pub selected_fan_curve_id: Option<String>,
    pub paired_fan_sensor_id: Option<String>,
    pub selected_start: f32,
    pub selected_stop: f32,
    pub selected_command_step_up: f32,
    pub selected_command_step_down: f32,
    pub selected_offset: f32,
    pub minimum_percent: f32,
    #[serde(default)]
    pub force_apply: bool,
    #[serde(default)]
    pub calibration: Option<Calibration>,
}

impl ControlConfig {
    pub fn default_for_hardware(id: String, name: String, hardware_id: String) -> Self {
        Self {
            id,
            name,
            hardware_id,
            enabled: false,
            manual_control: false,
            manual_control_value: 50.0,
            selected_fan_curve_id: None,
            paired_fan_sensor_id: None,
            selected_start: 20.0,
            selected_stop: 15.0,
            selected_command_step_up: 8.0,
            selected_command_step_down: 5.0,
            selected_offset: 0.0,
            minimum_percent: 0.0,
            force_apply: false,
            calibration: None,
        }
    }
}

/// Runtime state of a fan `Control` (`FanControl.Domain.Control`, `0x0200005e`).
#[derive(Debug, Clone)]
pub struct ControlRuntime {
    pub config: ControlConfig,
    pub current_command: Option<f32>,
    pub target_command: Option<f32>,
    pub hardware_readback_pct: Option<f32>,
    pub paired_rpm: Option<f32>,
    pub in_avoid_mode: bool,
    pub has_rpm_discrepancy: bool,
    pub is_command_stuck: bool,
    start_timeout_counter: i32,
    calibration_check: CalibrationCheck,
    command_check: CommandCheck,
}

impl ControlRuntime {
    pub fn new(mut config: ControlConfig) -> Self {
        if let Some(cal) = &mut config.calibration {
            cal.rebuild_tables();
        }
        Self {
            config,
            current_command: None,
            target_command: None,
            hardware_readback_pct: None,
            paired_rpm: None,
            in_avoid_mode: false,
            has_rpm_discrepancy: false,
            is_command_stuck: false,
            start_timeout_counter: 0,
            calibration_check: CalibrationCheck::new(8),
            command_check: CommandCheck::new(5),
        }
    }

    pub fn update_config(&mut self, mut config: ControlConfig) {
        if let Some(cal) = &mut config.calibration {
            cal.rebuild_tables();
        }
        self.config = config;
    }

    /// Computes effective stop threshold (`0x060002e7` IL_0000..IL_0037):
    /// `(SelectedStart > 0) ? ((SelectedStop != 0 && SelectedStop != SelectedStart) ? SelectedStop : SelectedStart - 1) : 0`
    pub fn stop_threshold(&self) -> f32 {
        let start = self.config.selected_start;
        let stop = self.config.selected_stop;
        if start > 0.0 {
            if stop != 0.0 && (stop - start).abs() > f32::EPSILON {
                stop
            } else {
                (start - 1.0).max(0.0)
            }
        } else {
            0.0
        }
    }

    /// Evaluates one 1 Hz tick of `Control.Update()` (`0x060002e5`).
    /// Returns `Some(pwm_percent)` if the control should write a command to hardware, or `None` if disabled/unassigned.
    pub fn update(
        &mut self,
        curve_output: Option<(f32, CommandMode)>,
        paired_rpm: Option<f32>,
        hw_readback_pct: Option<f32>,
    ) -> Option<f32> {
        self.paired_rpm = paired_rpm;
        self.hardware_readback_pct = hw_readback_pct;

        if !self.config.enabled {
            self.current_command = None;
            self.target_command = None;
            self.in_avoid_mode = false;
            self.has_rpm_discrepancy = false;
            self.is_command_stuck = false;
            self.start_timeout_counter = 0;
            self.calibration_check.reset();
            self.command_check.reset();
            return None;
        }

        // 1. Determine base target command %
        let raw_target_pct = if self.config.manual_control {
            Some(self.config.manual_control_value)
        } else if let Some((val, mode)) = curve_output {
            match mode {
                CommandMode::Percent => Some(val),
                CommandMode::Rpm => {
                    if let Some(cal) = &self.config.calibration {
                        Some(cal.get_command(val))
                    } else {
                        Some(val.clamp(0.0, 100.0))
                    }
                }
            }
        } else {
            None
        };

        let Some(mut target) = raw_target_pct else {
            self.target_command = None;
            return None;
        };

        // 2. Apply SelectedOffset and clamp to [MinimumPercent, 100.0]
        target = (target + self.config.selected_offset).clamp(self.config.minimum_percent, 100.0);

        // 3. Apply Calibration Avoid Regions (`GetAvoidedCommand`, `0x060002a1`)
        self.in_avoid_mode = false;
        if let Some(cal) = &self.config.calibration {
            let prev = self.current_command.unwrap_or(f32::NAN);
            let avoided = cal.get_avoided_command(target, prev);
            if (avoided - target).abs() > 0.01 {
                self.in_avoid_mode = true;
                target = avoided;
            }
        }
        self.target_command = Some(target);

        // 4. Slew-rate limiting + Start/Stop% kickstart (`GetUpdatedCommand`, `0x060002e7`)
        let current = self.current_command.or(hw_readback_pct).unwrap_or(0.0);
        let next_cmd = self.get_updated_command(target, current, paired_rpm);
        self.current_command = Some(next_cmd);

        // 5. Health & Discrepancy checks
        self.has_rpm_discrepancy = self.calibration_check.rpm_sensor_has_discrepancy(
            self.config.calibration.as_ref(),
            paired_rpm,
            self.current_command,
        );
        self.is_command_stuck = self.command_check.command_stuck(
            hw_readback_pct,
            self.current_command,
            self.config.force_apply,
        );

        Some(next_cmd)
    }

    /// Exact port of `Control.GetUpdatedCommand(float target, float current)` (`0x060002e7`).
    fn get_updated_command(&mut self, target: f32, current: f32, paired_rpm: Option<f32>) -> f32 {
        let stop_thresh = self.stop_threshold();
        let start_pct = self.config.selected_start;

        // If target is at or below stop threshold, ramp down toward 0 and snap to 0 once <= stop_threshold
        if target <= stop_thresh {
            self.start_timeout_counter = 0;
            let step_down = self.config.selected_command_step_down.max(0.1);
            let next = (current - step_down).max(0.0);
            return if next <= stop_thresh { 0.0 } else { next };
        }

        // Target is above stop_threshold: check if fan is currently stopped or stalled at 0 RPM
        let fan_stopped = current <= stop_thresh
            || (start_pct > 0.0 && paired_rpm.map(|r| r <= 1.0).unwrap_or(false));

        if fan_stopped && start_pct > 0.0 {
            self.start_timeout_counter += 1;
            if self.start_timeout_counter <= SELECTED_START_TIMEOUT_TICKS {
                return target.max(start_pct).clamp(0.0, 100.0);
            } else if paired_rpm.map(|r| r <= 1.0).unwrap_or(false) {
                // Paired RPM sensor still reads 0 after 6 ticks: boost by +3%/tick (`IL_00ce: ldc.r4 3; add`)
                let base = current.max(start_pct);
                return (base + KICKSTART_BOOST_STEP_PCT).clamp(0.0, 100.0);
            }
        } else {
            self.start_timeout_counter = 0;
        }

        // Asymmetric StepUp / StepDown slew-rate limiter
        let step_up = self.config.selected_command_step_up.max(0.1);
        let step_down = self.config.selected_command_step_down.max(0.1);
        let delta = (target - current).clamp(-step_down, step_up);
        let next = (current + delta).clamp(0.0, 100.0);

        if next <= stop_thresh {
            0.0
        } else {
            next
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_control_kickstart_and_step_rate() {
        let mut cfg = ControlConfig::default_for_hardware("c1".into(), "CPU Fan".into(), "hw1".into());
        cfg.enabled = true;
        cfg.manual_control = true;
        cfg.manual_control_value = 40.0;
        cfg.selected_start = 25.0;
        cfg.selected_stop = 15.0;
        cfg.selected_command_step_up = 5.0;
        cfg.selected_command_step_down = 2.0;

        let mut rt = ControlRuntime::new(cfg);

        // Tick 1 from stopped (0%) with RPM=0 -> kicks immediately to max(40, 25) = 40%
        let cmd1 = rt.update(None, Some(0.0), Some(0.0)).unwrap();
        assert_eq!(cmd1, 40.0);

        // Once fan spins (RPM = 900) and target increases to 60%, steps up by +5%/tick
        rt.config.manual_control_value = 60.0;
        let cmd2 = rt.update(None, Some(900.0), Some(40.0)).unwrap();
        assert_eq!(cmd2, 45.0);
    }
}
