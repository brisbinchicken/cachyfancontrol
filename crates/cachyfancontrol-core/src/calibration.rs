//! Fan calibration, RPM-to-PWM lookup tables, resonant avoid regions, and discrepancy checks
//! reverse-engineered from `FanControl.Domain.Calibration` (`0x0200005a`),
//! `CalibrationCheck` (`0x0200005b`), and `CommandCheck` (`0x0200005c`).

use crate::utilities::ResponseTime;
use serde::{Deserialize, Serialize};

/// Single calibration sample mapping PWM command `%` (`0..=100`) to measured RPM (`0x02000058`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct CalibrationPoint {
    pub command: i32,
    pub rpm: i32,
    #[serde(default)]
    pub avoid: bool,
}

/// Contiguous command `%` region marked `avoid = true` (`0x020000e5`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AvoidRegion {
    pub lower: f32,
    pub upper: f32,
}

const BITSHIFT_RESOLUTION: i32 = 5;
const RESOLUTION_RPM: i32 = 32;
const HALF_RESOLUTION_RPM: i32 = 16;

/// Calibration map and resonant avoid-region solver (`FanControl.Domain.Calibration`, `0x0200005a`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Calibration {
    pub points: Vec<CalibrationPoint>,
    pub stop_command: f32,
    pub start_command: f32,
    #[serde(skip)]
    min_rpm: i32,
    #[serde(skip)]
    max_rpm: i32,
    #[serde(skip)]
    commands_in_multiples_of_resolution_rpm: Vec<f32>,
    #[serde(skip)]
    command_to_rpm_reverse_map: Vec<f32>,
    #[serde(skip)]
    avoided_command_map: Vec<f32>,
    #[serde(skip)]
    avoided_command_map_coming_up: Vec<f32>,
    #[serde(skip)]
    avoided_command_map_coming_down: Vec<f32>,
    #[serde(skip)]
    avoid_regions: Vec<AvoidRegion>,
}

impl Calibration {
    /// Constructs and precomputes all lookup tables (`0x06000299`).
    pub fn new(mut points: Vec<CalibrationPoint>, stop_command: f32, start_command: f32) -> Self {
        points.sort_by_key(|p| p.command);
        let mut cal = Self {
            points,
            stop_command,
            start_command,
            min_rpm: 0,
            max_rpm: 0,
            commands_in_multiples_of_resolution_rpm: Vec::new(),
            command_to_rpm_reverse_map: Vec::new(),
            avoided_command_map: Vec::new(),
            avoided_command_map_coming_up: Vec::new(),
            avoided_command_map_coming_down: Vec::new(),
            avoid_regions: Vec::new(),
        };
        cal.rebuild_tables();
        cal
    }

    /// Rebuilds the 32-RPM resolution forward map, 0..=100 reverse map, and directional avoid maps.
    pub fn rebuild_tables(&mut self) {
        self.points.sort_by_key(|p| p.command);
        self.avoid_regions = Self::compute_avoid_regions(&self.points);

        if self.points.len() <= 1 {
            return;
        }

        self.min_rpm = self.points.first().map(|p| p.rpm).unwrap_or(0);
        self.max_rpm = self.points.last().map(|p| p.rpm).unwrap_or(0);

        let bucket_count = (2 + self.max_rpm / RESOLUTION_RPM).max(0) as usize;
        let mut forward = Vec::with_capacity(bucket_count);
        for i in 0..bucket_count {
            let rpm = (i as i32 * RESOLUTION_RPM) as f32;
            forward.push(self.interpolate_command(rpm));
        }
        self.commands_in_multiples_of_resolution_rpm = forward;

        let mut reverse = Vec::with_capacity(101);
        for cmd in 0..=100 {
            reverse.push(self.interpolate_rpm(cmd as f32));
        }
        self.command_to_rpm_reverse_map = reverse;

        if !self.avoid_regions.is_empty() {
            self.avoided_command_map = (0..=100)
                .map(|c| self.compute_avoided_command(c as f32, f32::NAN))
                .collect();
            self.avoided_command_map_coming_up = (0..=100)
                .map(|c| self.compute_avoided_command(c as f32, (c - 1) as f32))
                .collect();
            self.avoided_command_map_coming_down = (0..=100)
                .map(|c| self.compute_avoided_command(c as f32, (c + 1) as f32))
                .collect();
        } else {
            self.avoided_command_map.clear();
            self.avoided_command_map_coming_up.clear();
            self.avoided_command_map_coming_down.clear();
        }
    }

    pub fn avoid_regions(&self) -> &[AvoidRegion] {
        &self.avoid_regions
    }

    /// Converts target RPM into PWM command `%` (`0..=100`) (`0x0600029e`).
    pub fn get_command(&self, rpm: f32) -> f32 {
        if rpm < self.min_rpm as f32 {
            return 0.0;
        }
        if rpm >= self.max_rpm as f32 {
            return 100.0;
        }
        let rpm_i = rpm as i32;
        if !self.commands_in_multiples_of_resolution_rpm.is_empty() {
            let idx = (rpm_i >> BITSHIFT_RESOLUTION) as usize;
            if idx >= self.commands_in_multiples_of_resolution_rpm.len() - 1 {
                return 100.0;
            }
            if (rpm_i % RESOLUTION_RPM) >= HALF_RESOLUTION_RPM {
                self.commands_in_multiples_of_resolution_rpm[idx + 1]
            } else {
                self.commands_in_multiples_of_resolution_rpm[idx]
            }
        } else {
            self.interpolate_command(rpm)
        }
    }

    /// Converts PWM command `%` (`0..=100`) into expected calibrated RPM (`0x0600029f`).
    pub fn get_rpm(&self, command: f32) -> f32 {
        let cmd = self.get_avoided_command(command, f32::NAN);
        if cmd < 0.0 {
            return 0.0;
        }
        if cmd >= 100.0 {
            return self.max_rpm as f32;
        }
        if !self.command_to_rpm_reverse_map.is_empty() {
            let idx = (cmd.round() as isize).clamp(0, 100) as usize;
            self.command_to_rpm_reverse_map[idx]
        } else {
            self.interpolate_rpm(cmd)
        }
    }

    /// Jumps over resonant avoid regions based on approach direction (`0x060002a1`).
    pub fn get_avoided_command(&self, command: f32, prev_command: f32) -> f32 {
        if self.points.len() < 2 {
            return command;
        }
        let clamped = command.clamp(0.0, 100.0);
        if self.avoid_regions.is_empty() {
            return clamped;
        }
        if !self.avoided_command_map.is_empty() {
            let idx = (clamped.round() as isize).clamp(0, 100) as usize;
            if prev_command.is_nan() {
                return self.avoided_command_map[idx];
            }
            let prev_clamped = prev_command.clamp(0.0, 100.0);
            if clamped > prev_clamped {
                return self.avoided_command_map_coming_up[idx];
            } else if clamped < prev_clamped {
                return self.avoided_command_map_coming_down[idx];
            } else {
                return self.avoided_command_map[idx];
            }
        }
        self.compute_avoided_command(clamped, prev_command)
    }

    /// Computes avoided command against all `AvoidRegion`s (`0x060002a2`).
    pub fn compute_avoided_command(&self, mut cmd: f32, prev_cmd: f32) -> f32 {
        cmd = cmd.clamp(0.0, 100.0);
        for region in &self.avoid_regions {
            if cmd > region.lower && cmd < region.upper {
                if !prev_cmd.is_nan() {
                    if prev_cmd >= region.upper {
                        cmd = region.upper;
                    } else if prev_cmd <= region.lower {
                        cmd = region.lower;
                    } else if cmd >= prev_cmd {
                        cmd = region.lower;
                    } else {
                        cmd = region.upper;
                    }
                } else if (cmd - region.lower) > (region.upper - cmd) {
                    cmd = region.upper;
                } else {
                    cmd = region.lower;
                }
            }
        }
        cmd
    }

    fn compute_avoid_regions(points: &[CalibrationPoint]) -> Vec<AvoidRegion> {
        let mut regions = Vec::new();
        let mut i = 0;
        while i < points.len() {
            if !points[i].avoid {
                i += 1;
                continue;
            }
            let start = i;
            while i + 1 < points.len() && points[i + 1].avoid {
                i += 1;
            }
            let end = i;
            if end > start {
                regions.push(AvoidRegion {
                    lower: points[start].command as f32,
                    upper: points[end].command as f32,
                });
            }
            i += 1;
        }
        regions
    }

    fn interpolate_command(&self, rpm: f32) -> f32 {
        let mut prev = CalibrationPoint {
            command: 0,
            rpm: 0,
            avoid: false,
        };
        for pt in &self.points {
            if (pt.rpm as f32) > rpm {
                return Self::lerp_command_from_rpm(
                    (prev.command as f32, prev.rpm),
                    (pt.command as f32, pt.rpm),
                    rpm,
                );
            }
            prev = *pt;
        }
        100.0
    }

    fn interpolate_rpm(&self, command: f32) -> f32 {
        let mut prev = CalibrationPoint {
            command: 0,
            rpm: 0,
            avoid: false,
        };
        for pt in &self.points {
            if (pt.command as f32) > command {
                return Self::lerp_rpm_from_command(
                    (prev.command, prev.rpm as f32),
                    (pt.command, pt.rpm as f32),
                    command,
                );
            }
            prev = *pt;
        }
        self.points.last().map(|p| p.rpm as f32).unwrap_or(0.0)
    }

    fn lerp_command_from_rpm(p0: (f32, i32), p1: (f32, i32), rpm: f32) -> f32 {
        let d_cmd = p1.0 - p0.0;
        let d_rpm = (p1.1 - p0.1) as f64;
        if d_rpm.abs() < f64::EPSILON {
            return p0.0;
        }
        let t = (rpm - p0.1 as f32) as f64 / d_rpm;
        (p0.0 as f64 + (d_cmd as f64) * t) as f32
    }

    fn lerp_rpm_from_command(p0: (i32, f32), p1: (i32, f32), cmd: f32) -> f32 {
        let d_rpm = p1.1 - p0.1;
        let d_cmd = (p1.0 - p0.0) as f64;
        if d_cmd.abs() < f64::EPSILON {
            return p0.1;
        }
        let t = (cmd - p0.0 as f32) as f64 / d_cmd;
        (p0.1 as f64 + (d_rpm as f64) * t) as f32
    }
}

/// Checks if actual fan RPM deviates from calibrated RPM by `> 25%` of max RPM (`0x0200005b`).
#[derive(Debug, Clone)]
pub struct CalibrationCheck {
    response_time: ResponseTime,
}

impl CalibrationCheck {
    pub fn new(response_ticks: i32) -> Self {
        Self {
            response_time: ResponseTime::with_value(false, response_ticks),
        }
    }

    pub fn reset(&mut self) {
        self.response_time.reset();
    }

    pub fn rpm_sensor_has_discrepancy(
        &mut self,
        calibration: Option<&Calibration>,
        actual_rpm: Option<f32>,
        command_pct: Option<f32>,
    ) -> bool {
        let (Some(cal), Some(rpm), Some(cmd)) = (calibration, actual_rpm, command_pct) else {
            self.response_time.reset();
            return false;
        };
        let max_rpm = cal.get_rpm(100.0);
        if max_rpm <= 0.0 {
            self.response_time.reset();
            return false;
        }
        let expected_rpm = cal.get_rpm(cmd);
        if ((rpm - expected_rpm).abs() / max_rpm) > 0.25 {
            self.response_time.trigger()
        } else {
            self.response_time.reset();
            false
        }
    }
}

/// Checks if hardware readback command `%` differs from target command by `> 1.0%` (`0x0200005c`).
#[derive(Debug, Clone)]
pub struct CommandCheck {
    response_time: ResponseTime,
}

impl CommandCheck {
    pub fn new(response_ticks: i32) -> Self {
        Self {
            response_time: ResponseTime::with_value(false, response_ticks),
        }
    }

    pub fn reset(&mut self) {
        self.response_time.reset();
    }

    pub fn command_stuck(
        &mut self,
        sensor_value: Option<f32>,
        target_command: Option<f32>,
        force_apply: bool,
    ) -> bool {
        let (Some(actual), Some(target)) = (sensor_value, target_command) else {
            self.response_time.reset();
            return false;
        };
        if (actual - target).abs() > 1.0 {
            if force_apply {
                true
            } else {
                self.response_time.trigger()
            }
        } else {
            self.response_time.reset();
            false
        }
    }
}
