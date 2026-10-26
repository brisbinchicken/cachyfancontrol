//! Fan curve models and mathematical evaluation engines reverse-engineered from
//! `FanControl.Domain.FanCurves` in `FanControl.Library.dll` (V284).
//!
//! Includes:
//! - [`FanCurveKind::Linear`] (`LinearFanCurve`, `0x0200009b`)
//! - [`FanCurveKind::Graph`] (`GraphFanCurve`, `0x0200009a`)
//! - [`FanCurveKind::Auto`] (`AutoFanCurve` OLS Trend + Creep PID, `0x02000095`)
//! - [`FanCurveKind::Mix`] (`MixFanCurve`, `0x0200009d`)
//! - [`FanCurveKind::Trigger`] (`TriggerFanCurve`, `0x0200009f`)
//! - [`FanCurveKind::Flat`] (`FlatFanCurve`, `0x02000098`)
//! - [`FanCurveKind::Sync`] (`SyncFanCurve`, `0x0200009e`)
//! - [`order_fan_curves_by_dependencies`] (`FanCurveOrdering`, `0x020000a0`)

use crate::utilities::{Hysteresis, HysteresisConfig, LinearRegressionTrend, ResponseTime, Trend};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// Output mode of a fan curve (`FanControl.Serializable.Enums.CommandMode`, `0x02000041`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum CommandMode {
    #[default]
    Percent = 0,
    Rpm = 1,
}

impl CommandMode {
    pub fn max_bound(&self) -> f32 {
        match self {
            Self::Percent => 100.0,
            Self::Rpm => 10000.0,
        }
    }

    pub fn unit(&self) -> &'static str {
        match self {
            Self::Percent => "%",
            Self::Rpm => "RPM",
        }
    }
}

/// Mix function for [`FanCurveKind::Mix`] (`FanControl.Serializable.Enums.MixFunction`, `0x02000042`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MixFunction {
    Max = 0,
    Sum = 1,
    Average = 2,
    Min = 3,
    Subtract = 4,
}

impl MixFunction {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Max => "Max",
            Self::Sum => "Sum",
            Self::Average => "Average",
            Self::Min => "Min",
            Self::Subtract => "Subtract",
        }
    }

    pub const ALL: [MixFunction; 5] = [
        Self::Max,
        Self::Min,
        Self::Average,
        Self::Sum,
        Self::Subtract,
    ];
}

/// Control point `(temperature, command)` for [`FanCurveKind::Graph`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CurvePoint {
    pub temperature: f32,
    pub command: f32,
}

/// Serializable configuration for any FanControl fan curve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FanCurveConfig {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub command_mode: CommandMode,
    pub kind: FanCurveKind,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum FanCurveKind {
    /// Linear interpolation between `(min_temp, min_fan_speed)` and `(max_temp, max_fan_speed)` (`0x0200009b`).
    Linear {
        sensor_id: String,
        min_temp: f32,
        max_temp: f32,
        min_fan_speed: f32,
        max_fan_speed: f32,
        #[serde(default)]
        hysteresis: HysteresisConfig,
    },
    /// Piecewise linear curve over sorted `(temperature, command)` control points (`0x0200009a`).
    Graph {
        sensor_id: String,
        points: Vec<CurvePoint>,
        #[serde(default)]
        hysteresis: HysteresisConfig,
    },
    /// Target/Load temperature controller using dual-horizon OLS linear regression trends + creep (`0x02000095`).
    Auto {
        sensor_id: String,
        idle_temp: f32,
        load_temp: f32,
        deadband: f32,
        min_fan_speed: f32,
        max_fan_speed: f32,
        step: f32,
        selected_response_time: i32,
    },
    /// Combines multiple fan curves (`0x0200009d`).
    Mix {
        curve_ids: Vec<String>,
        function: MixFunction,
    },
    /// Two-point Schmitt-trigger curve (`0x0200009f`).
    Trigger {
        sensor_id: String,
        idle_temp: f32,
        load_temp: f32,
        idle_fan_speed: f32,
        load_fan_speed: f32,
        #[serde(default = "default_one")]
        response_time_up: i32,
        #[serde(default = "default_one")]
        response_time_down: i32,
    },
    /// Constant percentage or RPM (`0x02000098`).
    Flat {
        command: f32,
    },
    /// Synchronizes to another `Control` with an additive or proportional offset (`0x0200009e`).
    Sync {
        control_id: String,
        offset: f32,
        #[serde(default)]
        proportional: bool,
    },
}

fn default_one() -> i32 {
    1
}

impl FanCurveConfig {
    /// Evaluates the static curve function at temperature `temp` (without hysteresis/state) for UI plotting.
    pub fn evaluate_preview_at(&self, temp: f32) -> Option<f32> {
        match &self.kind {
            FanCurveKind::Linear {
                min_temp,
                max_temp,
                min_fan_speed,
                max_fan_speed,
                ..
            } => {
                if temp <= *min_temp {
                    Some(*min_fan_speed)
                } else if temp >= *max_temp {
                    Some(*max_fan_speed)
                } else {
                    let denom = (*max_temp - *min_temp).max(0.001);
                    let slope = (*max_fan_speed - *min_fan_speed) / denom;
                    Some(*min_fan_speed + (temp - *min_temp) * slope)
                }
            }
            FanCurveKind::Graph { points, .. } => Some(interpolate_graph_points(points, temp).0),
            FanCurveKind::Auto {
                idle_temp,
                load_temp,
                min_fan_speed,
                max_fan_speed,
                ..
            } => {
                if temp <= *idle_temp {
                    Some(*min_fan_speed)
                } else if temp >= *load_temp {
                    Some(*max_fan_speed)
                } else {
                    let denom = (*load_temp - *idle_temp).max(0.001);
                    Some(*min_fan_speed + (*max_fan_speed - *min_fan_speed) * (temp - *idle_temp) / denom)
                }
            }
            FanCurveKind::Trigger {
                idle_temp,
                load_temp,
                idle_fan_speed,
                load_fan_speed,
                ..
            } => {
                let mid = 0.5 * (*idle_temp + *load_temp);
                if temp >= mid {
                    Some(*load_fan_speed)
                } else {
                    Some(*idle_fan_speed)
                }
            }
            FanCurveKind::Flat { command } => Some(*command),
            FanCurveKind::Mix { .. } | FanCurveKind::Sync { .. } => None,
        }
    }
}

/// Piecewise linear interpolation over sorted `points` (`GraphFanCurve.UpdateCommand`, `0x06000405`).
/// Returns `(command, is_at_limit)`.
pub fn interpolate_graph_points(points: &[CurvePoint], temp: f32) -> (f32, bool) {
    if points.is_empty() {
        return (0.0, true);
    }
    let first = points[0];
    if temp <= first.temperature {
        return (first.command, true);
    }
    let last = points[points.len() - 1];
    if temp >= last.temperature {
        return (last.command, true);
    }
    for i in 1..points.len() {
        let p0 = points[i - 1];
        let p1 = points[i];
        if temp >= p0.temperature && temp <= p1.temperature {
            if (p0.temperature - p1.temperature).abs() < f32::EPSILON {
                return (p0.command, false);
            }
            let slope = (p0.command - p1.command) / (p0.temperature - p1.temperature);
            let intercept = p0.command - slope * p0.temperature;
            return (slope * temp + intercept, false);
        }
    }
    (last.command, true)
}

/// Internal runtime state for `AutoFanCurve` (`0x02000095`).
#[derive(Debug, Clone)]
pub struct AutoCurveState {
    pub hysteresis: Hysteresis,
    pub short_trend: LinearRegressionTrend,
    pub long_trend: LinearRegressionTrend,
    pub response_up_time: ResponseTime,
    pub response_down_time: ResponseTime,
    pub under_load: bool,
    pub last_load_command_target: Option<f32>,
    pub last_short_trend: Trend,
    pub last_long_trend: Trend,
}

impl AutoCurveState {
    pub fn new(selected_response_time: i32) -> Self {
        let r = selected_response_time.max(1);
        let hyst_cfg = HysteresisConfig {
            hysteresis_value_up: 0.0,
            hysteresis_value_down: 0.0,
            response_time_up: r,
            response_time_down: r,
            ignore_hysteresis_at_limits: false,
        };
        Self {
            hysteresis: Hysteresis::new(hyst_cfg),
            short_trend: LinearRegressionTrend::new((3 * r) as usize, 1.0),
            long_trend: LinearRegressionTrend::new((5 * r) as usize, 1.0),
            response_up_time: ResponseTime::with_value(false, r),
            response_down_time: ResponseTime::with_value(false, 2 * r),
            under_load: false,
            last_load_command_target: None,
            last_short_trend: Trend::Unknown,
            last_long_trend: Trend::Unknown,
        }
    }
}

/// Runtime instance of a fan curve in the 1 Hz engine.
#[derive(Debug, Clone)]
pub struct FanCurveRuntime {
    pub config: FanCurveConfig,
    pub command: Option<f32>,
    pub current_temp: Option<f32>,
    pub active_sub_curve_id: Option<String>,
    // State for Linear & Graph
    hysteresis: Hysteresis,
    // State for Auto
    pub auto_state: AutoCurveState,
    // State for Trigger
    trigger_up: ResponseTime,
    trigger_down: ResponseTime,
    trigger_prev_temp: Option<f32>,
}

impl FanCurveRuntime {
    pub fn new(config: FanCurveConfig) -> Self {
        let hyst_cfg = match &config.kind {
            FanCurveKind::Linear { hysteresis, .. } | FanCurveKind::Graph { hysteresis, .. } => {
                *hysteresis
            }
            _ => HysteresisConfig::default(),
        };
        let auto_r = match &config.kind {
            FanCurveKind::Auto {
                selected_response_time,
                ..
            } => *selected_response_time,
            _ => 2,
        };
        let (trig_up, trig_down) = match &config.kind {
            FanCurveKind::Trigger {
                response_time_up,
                response_time_down,
                ..
            } => (*response_time_up, *response_time_down),
            _ => (1, 1),
        };
        let mut rt = Self {
            config,
            command: None,
            current_temp: None,
            active_sub_curve_id: None,
            hysteresis: Hysteresis::new(hyst_cfg),
            auto_state: AutoCurveState::new(auto_r),
            trigger_up: ResponseTime::with_value(true, trig_up),
            trigger_down: ResponseTime::with_value(true, trig_down),
            trigger_prev_temp: None,
        };
        rt.normalize_points();
        rt
    }

    pub fn update_config(&mut self, config: FanCurveConfig) {
        match &config.kind {
            FanCurveKind::Linear { hysteresis, .. } | FanCurveKind::Graph { hysteresis, .. } => {
                self.hysteresis.on_update_config(*hysteresis);
            }
            FanCurveKind::Auto {
                selected_response_time,
                ..
            } => {
                self.auto_state = AutoCurveState::new(*selected_response_time);
            }
            FanCurveKind::Trigger {
                response_time_up,
                response_time_down,
                ..
            } => {
                self.trigger_up = ResponseTime::with_value(true, *response_time_up);
                self.trigger_down = ResponseTime::with_value(true, *response_time_down);
                self.trigger_prev_temp = None;
            }
            _ => {}
        }
        self.config = config;
        self.normalize_points();
    }

    fn normalize_points(&mut self) {
        if let FanCurveKind::Graph { points, .. } = &mut self.config.kind {
            points.sort_by(|a, b| {
                a.temperature
                    .partial_cmp(&b.temperature)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }
    }

    /// Evaluates the curve for the current 1 Hz tick.
    pub fn update(
        &mut self,
        sensor_values: &HashMap<String, Option<f32>>,
        curve_outputs: &HashMap<String, (Option<f32>, Option<f32>)>, // id -> (command, temp)
        control_outputs: &HashMap<String, (Option<f32>, Option<f32>)>, // id -> (cmd_pct, paired_rpm)
    ) -> Option<f32> {
        let prev_cmd = self.command;
        match &self.config.kind {
            FanCurveKind::Linear {
                sensor_id,
                min_temp,
                max_temp,
                min_fan_speed,
                max_fan_speed,
                ..
            } => {
                let Some(temp) = sensor_values.get(sensor_id).copied().flatten() else {
                    self.current_temp = None;
                    self.command = None;
                    return None;
                };
                self.current_temp = Some(temp);

                let (raw_cmd, is_at_limit) = if temp <= *min_temp {
                    (*min_fan_speed, true)
                } else if temp >= *max_temp {
                    (*max_fan_speed, true)
                } else {
                    let denom = (*max_temp - *min_temp) as f64;
                    let slope = if denom.abs() > f64::EPSILON {
                        ((*max_fan_speed - *min_fan_speed) as f64) / denom
                    } else {
                        0.0
                    };
                    (
                        *min_fan_speed + (((temp - *min_temp) as f64) * slope) as f32,
                        false,
                    )
                };

                if self.hysteresis.trigger(temp, is_at_limit) || prev_cmd.is_none() {
                    self.command = Some(raw_cmd);
                }
                self.command
            }

            FanCurveKind::Graph {
                sensor_id, points, ..
            } => {
                let Some(temp) = sensor_values.get(sensor_id).copied().flatten() else {
                    self.current_temp = None;
                    self.command = None;
                    return None;
                };
                self.current_temp = Some(temp);

                let (raw_cmd, is_at_limit) = interpolate_graph_points(points, temp);
                if self.hysteresis.trigger(temp, is_at_limit) || prev_cmd.is_none() {
                    self.command = Some(raw_cmd);
                }
                self.command
            }

            FanCurveKind::Auto {
                sensor_id,
                idle_temp,
                load_temp,
                deadband,
                min_fan_speed,
                max_fan_speed,
                step,
                ..
            } => {
                let Some(temp) = sensor_values.get(sensor_id).copied().flatten() else {
                    self.current_temp = None;
                    self.command = None;
                    return None;
                };
                self.current_temp = Some(temp);

                let min_spd = *min_fan_speed;
                let max_spd = *max_fan_speed;
                let idle_t = *idle_temp;
                let load_t = *load_temp;
                let db = *deadband;
                let step_val = *step;

                // Initial state when `currentCommand` is null or `< 0` (`0x060003e5` IL_0000..IL_0055)
                let Some(cur_cmd) = prev_cmd.filter(|&c| c >= 0.0) else {
                    if self.auto_state.last_load_command_target.is_none() {
                        self.auto_state.last_load_command_target =
                            Some(0.75 * (max_spd - min_spd) + min_spd);
                    }
                    let init_cmd = if temp <= load_t { min_spd } else { max_spd };
                    self.command = Some(init_cmd);
                    return self.command;
                };

                let long_trend = self.auto_state.long_trend.update(temp);
                let short_trend = self.auto_state.short_trend.update(temp);
                self.auto_state.last_long_trend = long_trend;
                self.auto_state.last_short_trend = short_trend;

                // IsUnderLoad (`0x060003e8`): temp >= 0.75 * (LoadTemp - Deadband - IdleTemp) + IdleTemp
                let is_under_load =
                    (temp as f64) >= 0.75 * ((load_t - db - idle_t) as f64) + (idle_t as f64);

                if is_under_load {
                    self.auto_state.under_load = true;
                    let mut target = self
                        .auto_state
                        .last_load_command_target
                        .unwrap_or(cur_cmd);

                    // ShouldStepUp (`0x060003e7`): (shortTrend == Up && temp >= LoadTemp - Deadband) || temp > LoadTemp
                    let should_step_up =
                        (short_trend == Trend::Up && temp >= load_t - db) || (temp > load_t);
                    // ShouldStepDown (`0x060003e6`): (longTrend == Down) || (temp <= LoadTemp - Deadband && longTrend == Neutral)
                    let should_step_down = (long_trend == Trend::Down)
                        || (temp <= load_t - db && long_trend == Trend::Neutral);

                    if should_step_up {
                        self.auto_state.response_down_time.reset();
                        if self.auto_state.response_up_time.trigger() {
                            target = (cur_cmd + step_val).clamp(min_spd, max_spd);
                        }
                    } else if should_step_down {
                        self.auto_state.response_up_time.reset();
                        if self.auto_state.response_down_time.trigger() {
                            // Steps down at half of `Step` (`IL_0151: ldc.r8 2; div`)
                            let half_step = (step_val as f64 / 2.0) as f32;
                            target = (cur_cmd - half_step).clamp(min_spd, max_spd);
                        }
                    } else {
                        self.auto_state.response_up_time.reset();
                        self.auto_state.response_down_time.reset();
                    }

                    self.auto_state.last_load_command_target = Some(target);
                    self.command = Some(target);
                } else {
                    if self.auto_state.under_load {
                        if (long_trend == Trend::Neutral || long_trend == Trend::Down)
                            && self.auto_state.response_down_time.trigger()
                        {
                            self.auto_state.under_load = false;
                            self.auto_state.response_down_time.reset();
                            self.auto_state.response_up_time.reset();
                            self.auto_state.hysteresis.reset();
                        }
                    }
                    if !self.auto_state.under_load {
                        if temp <= idle_t {
                            self.auto_state.hysteresis.reset();
                            self.command = Some(min_spd);
                        } else if temp > idle_t && self.auto_state.hysteresis.trigger(temp, false) {
                            let denom = (load_t - idle_t).max(0.001);
                            let interp =
                                min_spd + ((max_spd - min_spd) / denom) * (temp - idle_t);
                            self.command = Some(interp.clamp(min_spd, max_spd));
                        }
                    }
                }
                self.command
            }

            FanCurveKind::Mix {
                curve_ids,
                function,
            } => {
                if curve_ids.is_empty() {
                    self.command = None;
                    self.current_temp = None;
                    self.active_sub_curve_id = None;
                    return None;
                }

                let mut valid: Vec<(&String, f32, Option<f32>)> =
                    Vec::with_capacity(curve_ids.len());
                for cid in curve_ids {
                    if let Some(&(Some(cmd), temp_opt)) = curve_outputs.get(cid) {
                        if !cmd.is_nan() && cmd >= 0.0 {
                            valid.push((cid, cmd, temp_opt));
                        }
                    }
                }

                if valid.is_empty() {
                    self.command = None;
                    self.current_temp = None;
                    self.active_sub_curve_id = None;
                    return None;
                }

                let max_bound = self.config.command_mode.max_bound();
                match function {
                    MixFunction::Max => {
                        let mut best = valid[0];
                        for item in &valid[1..] {
                            if item.1 > best.1 {
                                best = *item;
                            }
                        }
                        self.active_sub_curve_id = Some(best.0.clone());
                        self.current_temp = best.2;
                        self.command = Some(best.1);
                    }
                    MixFunction::Min => {
                        let mut best = valid[0];
                        for item in &valid[1..] {
                            if item.1 < best.1 {
                                best = *item;
                            }
                        }
                        self.active_sub_curve_id = Some(best.0.clone());
                        self.current_temp = best.2;
                        self.command = Some(best.1.min(max_bound));
                    }
                    MixFunction::Sum => {
                        let sum: f32 = valid.iter().map(|(_, c, _)| *c).sum();
                        self.active_sub_curve_id = None;
                        self.current_temp = None;
                        self.command = Some(sum.clamp(0.0, max_bound));
                    }
                    MixFunction::Average => {
                        let sum: f32 = valid.iter().map(|(_, c, _)| *c).sum();
                        let avg = sum / (valid.len() as f32);
                        self.active_sub_curve_id = None;
                        self.current_temp = None;
                        self.command = Some(avg.clamp(0.0, max_bound));
                    }
                    MixFunction::Subtract => {
                        let first = valid[0].1;
                        let rest_sum: f32 = valid[1..].iter().map(|(_, c, _)| *c).sum();
                        self.active_sub_curve_id = None;
                        self.current_temp = None;
                        self.command = Some((first - rest_sum).clamp(0.0, max_bound));
                    }
                }
                self.command
            }

            FanCurveKind::Trigger {
                sensor_id,
                idle_temp,
                load_temp,
                idle_fan_speed,
                load_fan_speed,
                ..
            } => {
                let Some(temp) = sensor_values.get(sensor_id).copied().flatten() else {
                    self.current_temp = None;
                    self.command = None;
                    return None;
                };
                self.current_temp = Some(temp);

                if prev_cmd.is_none() {
                    self.command = Some(if temp > *load_temp {
                        *load_fan_speed
                    } else {
                        *idle_fan_speed
                    });
                    return self.command;
                }

                let prev_t_below_load = self
                    .trigger_prev_temp
                    .map(|pt| pt < *load_temp)
                    .unwrap_or(true);
                let prev_t_above_idle = self
                    .trigger_prev_temp
                    .map(|pt| pt > *idle_temp)
                    .unwrap_or(true);

                if prev_t_below_load && temp >= *load_temp {
                    if self.trigger_up.trigger() {
                        self.trigger_prev_temp = Some(temp);
                        self.command = Some(*load_fan_speed);
                    }
                } else if prev_t_above_idle && temp <= *idle_temp {
                    if self.trigger_down.trigger() {
                        self.trigger_prev_temp = Some(temp);
                        self.command = Some(*idle_fan_speed);
                    }
                } else {
                    self.trigger_up.reset();
                    self.trigger_down.reset();
                }
                self.command
            }

            FanCurveKind::Flat { command } => {
                self.current_temp = None;
                self.command = Some(*command);
                self.command
            }

            FanCurveKind::Sync {
                control_id,
                offset,
                proportional,
            } => {
                let Some(&(cmd_pct, paired_rpm)) = control_outputs.get(control_id) else {
                    self.command = None;
                    return None;
                };
                let base_val = match self.config.command_mode {
                    CommandMode::Percent => cmd_pct,
                    CommandMode::Rpm => paired_rpm.or(cmd_pct),
                };
                self.command = base_val.map(|v| {
                    let out = if *proportional {
                        v * (1.0 + (*offset / 100.0))
                    } else {
                        v + *offset
                    };
                    out.clamp(0.0, self.config.command_mode.max_bound())
                });
                self.command
            }
        }
    }

    /// Returns direct fan curve IDs this curve depends on (`MixFanCurve`).
    pub fn dependency_curve_ids(&self) -> Vec<String> {
        match &self.config.kind {
            FanCurveKind::Mix { curve_ids, .. } => curve_ids.clone(),
            _ => Vec::new(),
        }
    }
}

/// Topological ordering of fan curves reverse-engineered from
/// `FanControl.Domain.FanCurves.FanCurveOrdering.OrderByFanCurveDependencies` (`0x0600044b`).
pub fn order_fan_curves_by_dependencies(curves: &[FanCurveRuntime]) -> Vec<usize> {
    let id_to_idx: HashMap<&str, usize> = curves
        .iter()
        .enumerate()
        .map(|(i, c)| (c.config.id.as_str(), i))
        .collect();

    fn get_depth(
        idx: usize,
        curves: &[FanCurveRuntime],
        id_to_idx: &HashMap<&str, usize>,
        visited: &mut HashSet<usize>,
    ) -> usize {
        if !visited.insert(idx) {
            return 0;
        }
        let mut depth = 1;
        for dep_id in curves[idx].dependency_curve_ids() {
            if let Some(&dep_idx) = id_to_idx.get(dep_id.as_str()) {
                depth += get_depth(dep_idx, curves, id_to_idx, visited);
            }
        }
        visited.remove(&idx);
        depth
    }

    let mut weighted: Vec<(usize, usize)> = (0..curves.len())
        .map(|i| {
            let mut visited = HashSet::new();
            (i, get_depth(i, curves, &id_to_idx, &mut visited))
        })
        .collect();

    weighted.sort_by_key(|&(_, d)| d);
    weighted.into_iter().map(|(i, _)| i).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_graph_fan_curve_interpolation() {
        let pts = vec![
            CurvePoint {
                temperature: 30.0,
                command: 20.0,
            },
            CurvePoint {
                temperature: 50.0,
                command: 50.0,
            },
            CurvePoint {
                temperature: 80.0,
                command: 100.0,
            },
        ];
        assert_eq!(interpolate_graph_points(&pts, 25.0), (20.0, true));
        assert_eq!(interpolate_graph_points(&pts, 40.0), (35.0, false));
        assert_eq!(interpolate_graph_points(&pts, 85.0), (100.0, true));
    }

    #[test]
    fn test_auto_fan_curve_load_step_up_and_half_step_down() {
        let cfg = FanCurveConfig {
            id: "auto_1".into(),
            name: "Auto CPU".into(),
            command_mode: CommandMode::Percent,
            kind: FanCurveKind::Auto {
                sensor_id: "cpu".into(),
                idle_temp: 40.0,
                load_temp: 75.0,
                deadband: 3.0,
                min_fan_speed: 25.0,
                max_fan_speed: 100.0,
                step: 4.0,
                selected_response_time: 1,
            },
        };
        let mut rt = FanCurveRuntime::new(cfg);
        let mut sensors = HashMap::new();
        let curves = HashMap::new();
        let controls = HashMap::new();

        // Initial tick at 50C (<= 75C) -> initializes to min_fan_speed (25%), _lastLoadCommandTarget = 81.25%
        sensors.insert("cpu".into(), Some(50.0));
        assert_eq!(rt.update(&sensors, &curves, &controls), Some(25.0));

        // Jump to 78C (> LoadTemp 75C, under load) -> steps up from 25% by +4% = 29%
        sensors.insert("cpu".into(), Some(78.0));
        assert_eq!(rt.update(&sensors, &curves, &controls), Some(29.0));
        assert!(rt.auto_state.under_load);
    }
}
