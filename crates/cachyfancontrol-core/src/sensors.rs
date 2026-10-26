//! Hardware sensor models and Custom Sensors reverse-engineered from
//! `FanControl.Domain.CustomSensors` in `FanControl.Library.dll` (V284).
//!
//! Includes:
//! - [`TimeAverageSensor`] (`0x020000b3`)
//! - [`MixSensor`] & [`SensorMixFunction`] (`0x020000b1`)
//! - [`OffsetSensor`] (`0x020000b2`)
//! - [`FileSensor`] (`0x020000af`)
//! - [`order_custom_sensors_by_dependencies`] (`0x020000ae`)

use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::fs;
use std::time::SystemTime;

/// Mix function for [`CustomSensorKind::Mix`] (`FanControl.Serializable.Enums.SensorMixFunction`, `0x02000043`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SensorMixFunction {
    Average = 0,
    Max = 1,
    Min = 2,
    Sum = 3,
    Subtract = 4,
}

impl SensorMixFunction {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Average => "Average",
            Self::Max => "Max",
            Self::Min => "Min",
            Self::Sum => "Sum",
            Self::Subtract => "Subtract",
        }
    }

    pub const ALL: [SensorMixFunction; 5] = [
        Self::Average,
        Self::Max,
        Self::Min,
        Self::Sum,
        Self::Subtract,
    ];
}

/// Hardware temperature sensor discovered from `/sys/class/hwmon/`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HardwareTempSensor {
    pub id: String,
    pub name: String,
    pub chip_name: String,
    pub sysfs_path: String,
    pub value: Option<f32>,
    pub min: Option<f32>,
    pub max: Option<f32>,
    pub crit: Option<f32>,
}

/// Hardware fan tachometer (RPM) sensor discovered from `/sys/class/hwmon/`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HardwareFanSensor {
    pub id: String,
    pub name: String,
    pub chip_name: String,
    pub sysfs_path: String,
    pub rpm: Option<f32>,
}

/// Serializable configuration for a custom virtual temperature sensor.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomSensorConfig {
    pub id: String,
    pub name: String,
    pub kind: CustomSensorKind,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CustomSensorKind {
    /// Moving-average sensor (`TimeAverageSensor`, `0x020000b3`).
    TimeAverage {
        source_id: String,
        selected_time: usize,
    },
    /// Multi-sensor combination (`MixSensor`, `0x020000b1`).
    Mix {
        source_ids: Vec<String>,
        function: SensorMixFunction,
        #[serde(default = "default_true")]
        allow_missing_sensor: bool,
    },
    /// Additive or proportional offset (`OffsetSensor`, `0x020000b2`).
    Offset {
        source_id: String,
        offset: f32,
        #[serde(default)]
        proportional: bool,
    },
    /// External file sensor (`FileSensor`, `0x020000af`).
    File {
        path: String,
    },
}

fn default_true() -> bool {
    true
}

/// Runtime state of a custom sensor in the 1 Hz engine.
#[derive(Debug, Clone)]
pub struct CustomSensorRuntime {
    pub config: CustomSensorConfig,
    pub value: Option<f32>,
    pub active_source_id: Option<String>,
    // TimeAverage internal state
    average_buffer: Vec<Option<f32>>,
    current_index: usize,
    // FileSensor internal state
    last_write_time: Option<SystemTime>,
    skip_count: i32,
}

const FILE_ERROR_SKIP_TICKS: i32 = 30;

impl CustomSensorRuntime {
    pub fn new(config: CustomSensorConfig) -> Self {
        let buf_len = match &config.kind {
            CustomSensorKind::TimeAverage { selected_time, .. } => (*selected_time).max(1),
            _ => 0,
        };
        Self {
            config,
            value: None,
            active_source_id: None,
            average_buffer: vec![None; buf_len],
            current_index: 0,
            last_write_time: None,
            skip_count: 0,
        }
    }

    pub fn update_config(&mut self, config: CustomSensorConfig) {
        if let CustomSensorKind::TimeAverage { selected_time, .. } = &config.kind {
            let len = (*selected_time).max(1);
            if self.average_buffer.len() != len {
                self.average_buffer = vec![None; len];
                self.current_index = 0;
            }
        }
        self.config = config;
    }

    /// Evaluates the custom sensor for the current 1 Hz tick using `sensor_values`.
    pub fn update(&mut self, sensor_values: &HashMap<String, Option<f32>>) -> Option<f32> {
        match &self.config.kind {
            CustomSensorKind::TimeAverage {
                source_id,
                selected_time,
            } => {
                let len = (*selected_time).max(1);
                if self.average_buffer.len() != len {
                    self.average_buffer = vec![None; len];
                    self.current_index = 0;
                }
                let src_val = sensor_values.get(source_id).copied().flatten();
                if let Some(v) = src_val {
                    self.average_buffer[self.current_index] = Some(v);
                    self.current_index = (self.current_index + 1) % len;
                    // Exact match to .NET `Enumerable.Average(float?[])`: averages all non-null slots!
                    let mut sum = 0.0_f32;
                    let mut count = 0_usize;
                    for item in self.average_buffer.iter().flatten() {
                        sum += *item;
                        count += 1;
                    }
                    self.value = if count > 0 {
                        Some(sum / (count as f32))
                    } else {
                        None
                    };
                } else {
                    self.value = None;
                }
                self.active_source_id = Some(source_id.clone());
                self.value
            }

            CustomSensorKind::Mix {
                source_ids,
                function,
                allow_missing_sensor,
            } => {
                if source_ids.is_empty() {
                    self.value = None;
                    self.active_source_id = None;
                    return None;
                }

                let mut valid_pairs: Vec<(&String, f32)> = Vec::with_capacity(source_ids.len());
                for id in source_ids {
                    match sensor_values.get(id).copied().flatten() {
                        Some(v) if !v.is_nan() => valid_pairs.push((id, v)),
                        _ => {
                            if !*allow_missing_sensor {
                                self.value = None;
                                self.active_source_id = None;
                                return None;
                            }
                        }
                    }
                }

                if valid_pairs.is_empty() {
                    self.value = None;
                    self.active_source_id = None;
                    return None;
                }

                match function {
                    SensorMixFunction::Average => {
                        let sum: f32 = valid_pairs.iter().map(|(_, v)| *v).sum();
                        self.active_source_id = None;
                        self.value = Some(sum / (valid_pairs.len() as f32));
                    }
                    SensorMixFunction::Max => {
                        let mut best = valid_pairs[0];
                        for pair in &valid_pairs[1..] {
                            if pair.1 > best.1 {
                                best = *pair;
                            }
                        }
                        self.active_source_id = Some(best.0.clone());
                        self.value = Some(best.1);
                    }
                    SensorMixFunction::Min => {
                        let mut best = valid_pairs[0];
                        for pair in &valid_pairs[1..] {
                            if pair.1 < best.1 {
                                best = *pair;
                            }
                        }
                        self.active_source_id = Some(best.0.clone());
                        self.value = Some(best.1);
                    }
                    SensorMixFunction::Sum => {
                        let sum: f32 = valid_pairs.iter().map(|(_, v)| *v).sum();
                        self.active_source_id = None;
                        self.value = Some(sum);
                    }
                    SensorMixFunction::Subtract => {
                        // Requires >= 2 sources (`0x06000419` IL_0166: `ldlen < 2 -> null`)
                        if source_ids.len() < 2 {
                            self.value = None;
                            self.active_source_id = None;
                            return None;
                        }
                        let Some(first) = sensor_values.get(&source_ids[0]).copied().flatten() else {
                            self.value = None;
                            self.active_source_id = None;
                            return None;
                        };
                        let mut acc = first;
                        for id in &source_ids[1..] {
                            if let Some(v) = sensor_values.get(id).copied().flatten() {
                                acc -= v;
                            } else {
                                self.value = None;
                                self.active_source_id = None;
                                return None;
                            }
                        }
                        self.active_source_id = None;
                        self.value = Some(acc);
                    }
                }
                self.value
            }

            CustomSensorKind::Offset {
                source_id,
                offset,
                proportional,
            } => {
                let src = sensor_values.get(source_id).copied().flatten();
                self.active_source_id = Some(source_id.clone());
                self.value = src.map(|v| {
                    if *proportional {
                        v * (1.0 + (*offset / 100.0))
                    } else {
                        v + *offset
                    }
                });
                self.value
            }

            CustomSensorKind::File { path } => {
                if self.skip_count > 0 {
                    self.skip_count -= 1;
                    return self.value;
                }
                match fs::metadata(path).and_then(|m| m.modified()) {
                    Ok(mtime) => {
                        if self.last_write_time == Some(mtime) && self.value.is_some() {
                            return self.value;
                        }
                        match fs::read_to_string(path) {
                            Ok(text) => {
                                let trimmed = text.trim();
                                if let Ok(parsed) = trimmed.parse::<f32>() {
                                    // Support both millidegrees (e.g. 45000) and degrees (45.0)
                                    let val = if parsed > 1000.0 {
                                        parsed / 1000.0
                                    } else {
                                        parsed
                                    };
                                    self.last_write_time = Some(mtime);
                                    self.value = Some(val);
                                } else {
                                    self.value = None;
                                }
                            }
                            Err(_) => {
                                self.value = None;
                                self.skip_count = FILE_ERROR_SKIP_TICKS;
                            }
                        }
                    }
                    Err(_) => {
                        self.value = None;
                        self.skip_count = FILE_ERROR_SKIP_TICKS;
                    }
                }
                self.value
            }
        }
    }

    /// Returns direct sensor IDs this custom sensor depends on.
    pub fn dependency_ids(&self) -> Vec<String> {
        match &self.config.kind {
            CustomSensorKind::TimeAverage { source_id, .. } => vec![source_id.clone()],
            CustomSensorKind::Mix { source_ids, .. } => source_ids.clone(),
            CustomSensorKind::Offset { source_id, .. } => vec![source_id.clone()],
            CustomSensorKind::File { .. } => Vec::new(),
        }
    }
}

/// Topological ordering of custom sensors reverse-engineered from
/// `FanControl.Domain.CustomSensors.CustomSensorOrdering.OrderBySensorDependencies` (`0x060003fb`).
pub fn order_custom_sensors_by_dependencies(sensors: &[CustomSensorRuntime]) -> Vec<usize> {
    let id_to_idx: HashMap<&str, usize> = sensors
        .iter()
        .enumerate()
        .map(|(i, s)| (s.config.id.as_str(), i))
        .collect();

    fn get_weight(
        idx: usize,
        sensors: &[CustomSensorRuntime],
        id_to_idx: &HashMap<&str, usize>,
        visited: &mut HashSet<usize>,
    ) -> usize {
        if !visited.insert(idx) {
            return 0; // Guard against circular dependency
        }
        let mut weight = 1;
        for dep_id in sensors[idx].dependency_ids() {
            if let Some(&dep_idx) = id_to_idx.get(dep_id.as_str()) {
                weight += get_weight(dep_idx, sensors, id_to_idx, visited);
            }
        }
        visited.remove(&idx);
        weight
    }

    let mut indexed_weights: Vec<(usize, usize)> = (0..sensors.len())
        .map(|i| {
            let mut visited = HashSet::new();
            (i, get_weight(i, sensors, &id_to_idx, &mut visited))
        })
        .collect();

    // Leaf custom sensors (weight 1) evaluate first so composite custom sensors see updated values
    indexed_weights.sort_by_key(|&(_, w)| w);
    indexed_weights.into_iter().map(|(i, _)| i).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_average_sensor_immediate_startup() {
        let cfg = CustomSensorConfig {
            id: "avg_cpu".into(),
            name: "CPU 5s Avg".into(),
            kind: CustomSensorKind::TimeAverage {
                source_id: "hw_cpu".into(),
                selected_time: 5,
            },
        };
        let mut runtime = CustomSensorRuntime::new(cfg);
        let mut map = HashMap::new();
        map.insert("hw_cpu".into(), Some(50.0));
        // Tick 1: only 1 of 5 slots filled, immediately averages to 50.0
        assert_eq!(runtime.update(&map), Some(50.0));
        map.insert("hw_cpu".into(), Some(60.0));
        // Tick 2: 2 of 5 slots filled -> (50 + 60) / 2 = 55.0
        assert_eq!(runtime.update(&map), Some(55.0));
    }

    #[test]
    fn test_mix_sensor_functions() {
        let mut map = HashMap::new();
        map.insert("s1".into(), Some(40.0));
        map.insert("s2".into(), Some(65.0));

        let mut max_sensor = CustomSensorRuntime::new(CustomSensorConfig {
            id: "mix_max".into(),
            name: "Max".into(),
            kind: CustomSensorKind::Mix {
                source_ids: vec!["s1".into(), "s2".into()],
                function: SensorMixFunction::Max,
                allow_missing_sensor: true,
            },
        });
        assert_eq!(max_sensor.update(&map), Some(65.0));
        assert_eq!(max_sensor.active_source_id.as_deref(), Some("s2"));

        let mut sub_sensor = CustomSensorRuntime::new(CustomSensorConfig {
            id: "mix_sub".into(),
            name: "Delta".into(),
            kind: CustomSensorKind::Mix {
                source_ids: vec!["s2".into(), "s1".into()],
                function: SensorMixFunction::Subtract,
                allow_missing_sensor: true,
            },
        });
        assert_eq!(sub_sensor.update(&map), Some(25.0));
    }
}
