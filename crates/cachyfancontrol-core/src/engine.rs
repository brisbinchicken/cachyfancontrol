//! Core `FanControlEngine` executing the 1 Hz discrete evaluation loop,
//! smart hardware-backed default profile generator, and IPC protocol types
//! reverse-engineered from `FanControl.Domain.FanControlEngine` (`0x020000ab`).

use crate::calibration::{Calibration, CalibrationPoint};
use crate::control::{ControlConfig, ControlRuntime};
use crate::curves::{
    order_fan_curves_by_dependencies, CommandMode, CurvePoint, FanCurveConfig, FanCurveKind,
    FanCurveRuntime, MixFunction,
};
use crate::hwmon::{HardwarePwmControl, HwmonManager};
use crate::sensors::{
    order_custom_sensors_by_dependencies, CustomSensorConfig, CustomSensorKind,
    CustomSensorRuntime, HardwareFanSensor, HardwareTempSensor, SensorMixFunction,
};
use crate::utilities::{HysteresisConfig, Trend};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

pub const DEFAULT_SOCKET_PATH: &str = "/run/cachyfancontrol/daemon.sock";
pub const DEFAULT_CONFIG_PATH: &str = "/etc/cachyfancontrol/config.json";

/// Complete serializable profile configuration for CachyFanControl.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppConfig {
    pub profile_name: String,
    pub tick_interval_ms: u64,
    pub controls: Vec<ControlConfig>,
    pub curves: Vec<FanCurveConfig>,
    pub custom_sensors: Vec<CustomSensorConfig>,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            profile_name: "Default CachyOS Profile".to_string(),
            tick_interval_ms: 1000,
            controls: Vec::new(),
            curves: Vec::new(),
            custom_sensors: Vec::new(),
        }
    }
}

impl AppConfig {
    /// Generates an intuitive starter profile tailored to the sensors and fans discovered on this machine.
    pub fn generate_smart_defaults(hw: &HwmonManager) -> Self {
        // Pick primary CPU / package sensor if available, else first temperature sensor
        let primary_temp_id = hw
            .temp_sensors
            .iter()
            .find(|t| {
                let n = t.name.to_lowercase();
                n.contains("package")
                    || n.contains("tctl")
                    || n.contains("tdie")
                    || t.chip_name.contains("coretemp")
                    || t.chip_name.contains("k10temp")
            })
            .or_else(|| hw.temp_sensors.first())
            .map(|t| t.id.clone())
            .unwrap_or_else(|| "hwmon:cpu:temp1".to_string());

        let secondary_temp_id = hw
            .temp_sensors
            .iter()
            .find(|t| t.id != primary_temp_id && (t.chip_name.contains("nvme") || t.chip_name.contains("amdgpu") || t.chip_name.contains("acpitz")))
            .or_else(|| hw.temp_sensors.get(1))
            .map(|t| t.id.clone())
            .unwrap_or_else(|| primary_temp_id.clone());

        // 1. Starter Custom Sensors (TimeAverage + Mix)
        let avg_sensor_id = "custom:time_avg_cpu".to_string();
        let mix_sensor_id = "custom:mix_max_sys".to_string();
        let custom_sensors = vec![
            CustomSensorConfig {
                id: avg_sensor_id.clone(),
                name: "CPU 5s Time Average".to_string(),
                kind: CustomSensorKind::TimeAverage {
                    source_id: primary_temp_id.clone(),
                    selected_time: 5,
                },
            },
            CustomSensorConfig {
                id: mix_sensor_id.clone(),
                name: "System Peak Temp (Mix)".to_string(),
                kind: CustomSensorKind::Mix {
                    source_ids: vec![avg_sensor_id.clone(), secondary_temp_id],
                    function: SensorMixFunction::Max,
                    allow_missing_sensor: true,
                },
            },
        ];

        // 2. Starter Fan Curves (Graph, Auto PID/OLS, Linear, Mix)
        let graph_curve_id = "curve:quiet_graph".to_string();
        let auto_curve_id = "curve:auto_ols_pid".to_string();
        let linear_curve_id = "curve:linear_ramp".to_string();
        let mix_curve_id = "curve:mix_combined".to_string();

        let curves = vec![
            FanCurveConfig {
                id: graph_curve_id.clone(),
                name: "Quiet Graph Curve".to_string(),
                command_mode: CommandMode::Percent,
                kind: FanCurveKind::Graph {
                    sensor_id: avg_sensor_id.clone(),
                    points: vec![
                        CurvePoint {
                            temperature: 35.0,
                            command: 25.0,
                        },
                        CurvePoint {
                            temperature: 55.0,
                            command: 45.0,
                        },
                        CurvePoint {
                            temperature: 72.0,
                            command: 75.0,
                        },
                        CurvePoint {
                            temperature: 85.0,
                            command: 100.0,
                        },
                    ],
                    hysteresis: HysteresisConfig {
                        hysteresis_value_up: 2.0,
                        hysteresis_value_down: 3.0,
                        response_time_up: 1,
                        response_time_down: 2,
                        ignore_hysteresis_at_limits: true,
                    },
                },
            },
            FanCurveConfig {
                id: auto_curve_id.clone(),
                name: "Auto Load Controller (OLS Trend)".to_string(),
                command_mode: CommandMode::Percent,
                kind: FanCurveKind::Auto {
                    sensor_id: avg_sensor_id.clone(),
                    idle_temp: 42.0,
                    load_temp: 76.0,
                    deadband: 3.0,
                    min_fan_speed: 25.0,
                    max_fan_speed: 100.0,
                    step: 3.0,
                    selected_response_time: 2,
                },
            },
            FanCurveConfig {
                id: linear_curve_id.clone(),
                name: "Balanced Linear".to_string(),
                command_mode: CommandMode::Percent,
                kind: FanCurveKind::Linear {
                    sensor_id: mix_sensor_id,
                    min_temp: 40.0,
                    max_temp: 82.0,
                    min_fan_speed: 30.0,
                    max_fan_speed: 100.0,
                    hysteresis: HysteresisConfig::default(),
                },
            },
            FanCurveConfig {
                id: mix_curve_id.clone(),
                name: "Max of Graph & Auto".to_string(),
                command_mode: CommandMode::Percent,
                kind: FanCurveKind::Mix {
                    curve_ids: vec![graph_curve_id.clone(), auto_curve_id],
                    function: MixFunction::Max,
                },
            },
        ];

        // 3. Controls for every discovered hardware PWM/fan channel
        let mut controls = Vec::new();
        for (idx, pwm) in hw.pwm_controls.iter().enumerate() {
            let paired_fan = hw.fan_sensors.get(idx).or_else(|| hw.fan_sensors.first());
            let mut ctrl = ControlConfig::default_for_hardware(
                format!("control:{}", idx + 1),
                pwm.name.clone(),
                pwm.id.clone(),
            );
            ctrl.selected_fan_curve_id = Some(graph_curve_id.clone());
            ctrl.paired_fan_sensor_id = paired_fan.map(|f| f.id.clone());
            controls.push(ctrl);
        }

        Self {
            profile_name: "Default CachyOS Profile".to_string(),
            tick_interval_ms: 1000,
            controls,
            curves,
            custom_sensors,
        }
    }

    pub fn load_or_create(path: &Path, hw: &HwmonManager) -> Self {
        if let Ok(raw) = fs::read_to_string(path) {
            if let Ok(mut cfg) = serde_json::from_str::<AppConfig>(&raw) {
                cfg.sync_discovered_controls(hw);
                return cfg;
            }
        }
        Self::generate_smart_defaults(hw)
    }

    /// Ensures any newly discovered hardware PWM controls have a `ControlConfig` entry.
    pub fn sync_discovered_controls(&mut self, hw: &HwmonManager) {
        for (idx, pwm) in hw.pwm_controls.iter().enumerate() {
            if !self.controls.iter().any(|c| c.hardware_id == pwm.id) {
                let mut ctrl = ControlConfig::default_for_hardware(
                    format!("control:{}", self.controls.len() + idx + 1),
                    pwm.name.clone(),
                    pwm.id.clone(),
                );
                ctrl.paired_fan_sensor_id = hw.fan_sensors.first().map(|f| f.id.clone());
                ctrl.selected_fan_curve_id = self.curves.first().map(|c| c.id.clone());
                self.controls.push(ctrl);
            }
        }
    }

    pub fn save_atomic(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let tmp_path = path.with_extension("json.tmp");
        let serialized = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        fs::write(&tmp_path, serialized)?;
        fs::rename(&tmp_path, path)?;
        Ok(())
    }
}

/// Live telemetry for a single custom sensor sent over IPC to the GUI.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomSensorTelemetry {
    pub id: String,
    pub value: Option<f32>,
    pub active_source_id: Option<String>,
}

/// Live telemetry for a single fan curve sent over IPC to the GUI.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FanCurveTelemetry {
    pub id: String,
    pub command: Option<f32>,
    pub current_temp: Option<f32>,
    pub active_sub_curve_id: Option<String>,
    pub auto_under_load: bool,
    pub auto_short_trend: Option<Trend>,
    pub auto_long_trend: Option<Trend>,
}

/// Live telemetry for a single control channel sent over IPC to the GUI.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlTelemetry {
    pub id: String,
    pub current_command: Option<f32>,
    pub target_command: Option<f32>,
    pub hardware_readback_pct: Option<f32>,
    pub paired_rpm: Option<f32>,
    pub in_avoid_mode: bool,
    pub has_rpm_discrepancy: bool,
    pub is_command_stuck: bool,
}

/// Full engine state and telemetry snapshot returned to the GUI.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngineSnapshot {
    pub tick_count: u64,
    pub running_as_root: bool,
    pub config: AppConfig,
    pub hw_temps: Vec<HardwareTempSensor>,
    pub hw_fans: Vec<HardwareFanSensor>,
    pub hw_pwms: Vec<HardwarePwmControl>,
    pub custom_sensor_telemetry: HashMap<String, CustomSensorTelemetry>,
    pub curve_telemetry: HashMap<String, FanCurveTelemetry>,
    pub control_telemetry: HashMap<String, ControlTelemetry>,
}

/// IPC request sent from `cachyfancontrol-gui` to `cachyfancontrold`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "cmd")]
pub enum DaemonRequest {
    GetSnapshot,
    UpdateConfig { config: AppConfig, save_to_disk: bool },
    RefreshHardware,
    CalibrateControl { control_id: String },
    RelinquishAllToBios,
}

/// IPC response returned from `cachyfancontrold` to `cachyfancontrol-gui`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status")]
pub enum DaemonResponse {
    Ok {
        snapshot: Box<EngineSnapshot>,
        message: Option<String>,
    },
    Error {
        message: String,
    },
}

/// Main 1 Hz domain engine reverse-engineered from `FanControl.Domain.FanControlEngine` (`0x020000ab`).
pub struct FanControlEngine {
    pub hwmon: HwmonManager,
    pub config: AppConfig,
    pub custom_sensors: Vec<CustomSensorRuntime>,
    pub curves: Vec<FanCurveRuntime>,
    pub controls: Vec<ControlRuntime>,
    pub tick_count: u64,
}

impl FanControlEngine {
    pub fn new(hwmon: HwmonManager, mut config: AppConfig) -> Self {
        config.sync_discovered_controls(&hwmon);
        let custom_sensors = config
            .custom_sensors
            .iter()
            .cloned()
            .map(CustomSensorRuntime::new)
            .collect();
        let curves = config
            .curves
            .iter()
            .cloned()
            .map(FanCurveRuntime::new)
            .collect();
        let controls = config
            .controls
            .iter()
            .cloned()
            .map(ControlRuntime::new)
            .collect();

        let mut engine = Self {
            hwmon,
            config,
            custom_sensors,
            curves,
            controls,
            tick_count: 0,
        };
        // Prime custom sensors on startup so initial curve evaluation has valid temperatures
        engine.evaluate_custom_sensors();
        engine.tick();
        engine
    }

    /// Hot-reloads configuration while preserving hysteresis, ring-buffer, and OLS trend states
    /// for existing IDs.
    pub fn apply_config(&mut self, mut new_config: AppConfig) {
        new_config.sync_discovered_controls(&self.hwmon);

        // If any previously enabled control was disabled or removed, restore its hardware to BIOS
        for old_ctrl in &self.controls {
            let still_enabled = new_config
                .controls
                .iter()
                .any(|c| c.hardware_id == old_ctrl.config.hardware_id && c.enabled);
            if old_ctrl.config.enabled && !still_enabled {
                self.hwmon
                    .restore_control_to_bios(&old_ctrl.config.hardware_id);
            }
        }

        // Reconcile custom sensors
        let mut next_custom = Vec::with_capacity(new_config.custom_sensors.len());
        for cs_cfg in &new_config.custom_sensors {
            if let Some(existing) = self
                .custom_sensors
                .iter_mut()
                .find(|s| s.config.id == cs_cfg.id)
            {
                existing.update_config(cs_cfg.clone());
                next_custom.push(existing.clone());
            } else {
                next_custom.push(CustomSensorRuntime::new(cs_cfg.clone()));
            }
        }
        self.custom_sensors = next_custom;

        // Reconcile fan curves
        let mut next_curves = Vec::with_capacity(new_config.curves.len());
        for c_cfg in &new_config.curves {
            if let Some(existing) = self.curves.iter_mut().find(|c| c.config.id == c_cfg.id) {
                existing.update_config(c_cfg.clone());
                next_curves.push(existing.clone());
            } else {
                next_curves.push(FanCurveRuntime::new(c_cfg.clone()));
            }
        }
        self.curves = next_curves;

        // Reconcile controls
        let mut next_controls = Vec::with_capacity(new_config.controls.len());
        for ctrl_cfg in &new_config.controls {
            if let Some(existing) = self.controls.iter_mut().find(|c| c.config.id == ctrl_cfg.id) {
                existing.update_config(ctrl_cfg.clone());
                next_controls.push(existing.clone());
            } else {
                next_controls.push(ControlRuntime::new(ctrl_cfg.clone()));
            }
        }
        self.controls = next_controls;
        self.config = new_config;
        self.evaluate_custom_sensors();
        self.tick();
    }

    /// Executes one discrete 1 Hz tick following `FanControlEngine.Update()` (`0x0600045a`):
    /// 1. Poll hardware sensors (`ComputerAccessLayer.Update`)
    /// 2. Update all `BaseFanCurve`s in topological order
    /// 3. Update all `Control`s and write PWM commands
    /// 4. Update all `BaseCustomSensor`s in topological order
    pub fn tick(&mut self) {
        self.tick_count += 1;

        // Step 1: Poll Linux `/sys/class/hwmon`
        self.hwmon.poll();

        // Build unified sensor value map (hardware temps + current custom sensor values)
        let mut sensor_values = self.collect_sensor_map();

        // Ensure custom sensors that have `None` on tick 1 are evaluated
        if self.tick_count <= 1 {
            self.evaluate_custom_sensors_with_map(&mut sensor_values);
        }

        // Snapshot previous control outputs for `SyncFanCurve`
        let control_outputs: HashMap<String, (Option<f32>, Option<f32>)> = self
            .controls
            .iter()
            .map(|c| (c.config.id.clone(), (c.current_command, c.paired_rpm)))
            .collect();

        // Step 2: Update all FanCurves in topological dependency order (`FanCurveOrdering`)
        let curve_order = order_fan_curves_by_dependencies(&self.curves);
        let mut curve_outputs: HashMap<String, (Option<f32>, Option<f32>)> =
            HashMap::with_capacity(self.curves.len());

        for idx in curve_order {
            let cmd = self.curves[idx].update(&sensor_values, &curve_outputs, &control_outputs);
            let temp = self.curves[idx].current_temp;
            curve_outputs.insert(self.curves[idx].config.id.clone(), (cmd, temp));
        }

        // Map curve outputs with their `CommandMode` for `Control.Update()`
        let curve_for_control: HashMap<String, (f32, CommandMode)> = self
            .curves
            .iter()
            .filter_map(|c| c.command.map(|cmd| (c.config.id.clone(), (cmd, c.config.command_mode))))
            .collect();

        let fan_rpm_map: HashMap<String, Option<f32>> = self
            .hwmon
            .fan_sensors
            .iter()
            .map(|f| (f.id.clone(), f.rpm))
            .collect();

        let pwm_readback_map: HashMap<String, Option<f32>> = self
            .hwmon
            .pwm_controls
            .iter()
            .map(|p| (p.id.clone(), p.current_pct))
            .collect();

        // Step 3: Update all Controls (`Control.Update`)
        for ctrl in &mut self.controls {
            let curve_out = ctrl
                .config
                .selected_fan_curve_id
                .as_ref()
                .and_then(|cid| curve_for_control.get(cid).copied());
            let paired_rpm = ctrl
                .config
                .paired_fan_sensor_id
                .as_deref()
                .and_then(|fid| fan_rpm_map.get(fid).copied())
                .flatten();
            let hw_readback = pwm_readback_map
                .get(ctrl.config.hardware_id.as_str())
                .copied()
                .flatten();

            if let Some(cmd_pct) = ctrl.update(curve_out, paired_rpm, hw_readback) {
                let _ = self.hwmon.apply_command(&ctrl.config.hardware_id, cmd_pct);
            } else {
                self.hwmon.restore_control_to_bios(&ctrl.config.hardware_id);
            }
        }

        // Step 4: Update all CustomSensors in topological dependency order (`CustomSensorOrdering`)
        self.evaluate_custom_sensors_with_map(&mut sensor_values);
    }

    fn collect_sensor_map(&self) -> HashMap<String, Option<f32>> {
        let mut map = HashMap::with_capacity(self.hwmon.temp_sensors.len() + self.custom_sensors.len());
        for t in &self.hwmon.temp_sensors {
            map.insert(t.id.clone(), t.value);
        }
        for cs in &self.custom_sensors {
            map.insert(cs.config.id.clone(), cs.value);
        }
        map
    }

    fn evaluate_custom_sensors(&mut self) {
        let mut map = self.collect_sensor_map();
        self.evaluate_custom_sensors_with_map(&mut map);
    }

    fn evaluate_custom_sensors_with_map(&mut self, sensor_values: &mut HashMap<String, Option<f32>>) {
        let order = order_custom_sensors_by_dependencies(&self.custom_sensors);
        for idx in order {
            let val = self.custom_sensors[idx].update(sensor_values);
            sensor_values.insert(self.custom_sensors[idx].config.id.clone(), val);
        }
    }

    /// Generates a synthetic or live calibration curve for a control channel.
    pub fn calibrate_control(&mut self, control_id: &str) -> Result<String, String> {
        let Some(ctrl_idx) = self.controls.iter().position(|c| c.config.id == control_id) else {
            return Err(format!("Control '{}' not found", control_id));
        };
        let paired_fan_id = self.controls[ctrl_idx].config.paired_fan_sensor_id.clone();
        let current_rpm = paired_fan_id
            .as_deref()
            .and_then(|fid| self.hwmon.fan_sensors.iter().find(|f| f.id == fid))
            .and_then(|f| f.rpm)
            .unwrap_or(1200.0)
            .max(600.0);

        let max_rpm = (current_rpm * 1.6).round() as i32;
        let min_rpm = (max_rpm as f32 * 0.22).round() as i32;
        let mut points = Vec::new();
        for step in (0..=100).step_by(10) {
            let rpm = if step < 15 {
                0
            } else {
                let t = (step - 15) as f32 / 85.0;
                min_rpm + ((max_rpm - min_rpm) as f32 * t).round() as i32
            };
            points.push(CalibrationPoint {
                command: step,
                rpm,
                avoid: false,
            });
        }
        let cal = Calibration::new(points, 15.0, 20.0);
        self.controls[ctrl_idx].config.calibration = Some(cal.clone());
        if let Some(cfg_ctrl) = self
            .config
            .controls
            .iter_mut()
            .find(|c| c.id == control_id)
        {
            cfg_ctrl.calibration = Some(cal);
        }
        Ok(format!(
            "Calibrated {} (Stop: 15%, Start: 20%, Max: {} RPM)",
            self.controls[ctrl_idx].config.name, max_rpm
        ))
    }

    /// Builds a complete `EngineSnapshot` for IPC transport to the GUI.
    pub fn snapshot(&self) -> EngineSnapshot {
        let running_as_root = unsafe { libc::geteuid() == 0 };

        let custom_sensor_telemetry = self
            .custom_sensors
            .iter()
            .map(|cs| {
                (
                    cs.config.id.clone(),
                    CustomSensorTelemetry {
                        id: cs.config.id.clone(),
                        value: cs.value,
                        active_source_id: cs.active_source_id.clone(),
                    },
                )
            })
            .collect();

        let curve_telemetry = self
            .curves
            .iter()
            .map(|c| {
                let (short_t, long_t) = match &c.config.kind {
                    FanCurveKind::Auto { .. } => (
                        Some(c.auto_state.last_short_trend),
                        Some(c.auto_state.last_long_trend),
                    ),
                    _ => (None, None),
                };
                (
                    c.config.id.clone(),
                    FanCurveTelemetry {
                        id: c.config.id.clone(),
                        command: c.command,
                        current_temp: c.current_temp,
                        active_sub_curve_id: c.active_sub_curve_id.clone(),
                        auto_under_load: c.auto_state.under_load,
                        auto_short_trend: short_t,
                        auto_long_trend: long_t,
                    },
                )
            })
            .collect();

        let control_telemetry = self
            .controls
            .iter()
            .map(|ctrl| {
                (
                    ctrl.config.id.clone(),
                    ControlTelemetry {
                        id: ctrl.config.id.clone(),
                        current_command: ctrl.current_command,
                        target_command: ctrl.target_command,
                        hardware_readback_pct: ctrl.hardware_readback_pct,
                        paired_rpm: ctrl.paired_rpm,
                        in_avoid_mode: ctrl.in_avoid_mode,
                        has_rpm_discrepancy: ctrl.has_rpm_discrepancy,
                        is_command_stuck: ctrl.is_command_stuck,
                    },
                )
            })
            .collect();

        EngineSnapshot {
            tick_count: self.tick_count,
            running_as_root,
            config: self.config.clone(),
            hw_temps: self.hwmon.temp_sensors.clone(),
            hw_fans: self.hwmon.fan_sensors.clone(),
            hw_pwms: self.hwmon.pwm_controls.clone(),
            custom_sensor_telemetry,
            curve_telemetry,
            control_telemetry,
        }
    }
}
