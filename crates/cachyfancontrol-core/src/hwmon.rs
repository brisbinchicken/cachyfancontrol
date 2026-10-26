//! Native Linux `/sys/class/hwmon` and `/sys/class/thermal/cooling_device*`
//! Hardware Abstraction Layer (HAL) with non-blocking `O_NONBLOCK` sysfs I/O
//! and automatic BIOS/firmware fallback (`pwmX_enable = 2`).

use crate::sensors::{HardwareFanSensor, HardwareTempSensor};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::{self, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};

/// Mechanism used by the Linux kernel driver to control a fan channel.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinuxPwmMethod {
    /// Standard `/sys/class/hwmon/hwmonX/pwmY` (`0..=255`) + optional `pwmY_enable` (`1` = manual, `2` = BIOS auto).
    StandardPwm {
        pwm_path: String,
        enable_path: Option<String>,
    },
    /// WMI/EC `pwmY_enable`-only controller (e.g. `hp-wmi` where `0` = max fan speed, `2` = BIOS auto).
    EnableOnlyToggle {
        enable_path: String,
    },
    /// ACPI Fan cooling device (`/sys/class/thermal/cooling_deviceX/cur_state` in `0..=max_state`).
    ThermalCoolingDevice {
        cur_state_path: String,
        max_state: u32,
    },
}

/// Controllable hardware fan channel discovered on the system.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HardwarePwmControl {
    pub id: String,
    pub name: String,
    pub chip_name: String,
    pub method: LinuxPwmMethod,
    pub current_pct: Option<f32>,
    pub pwm_enable_mode: Option<i32>,
    pub writable: bool,
}

#[derive(Debug, Clone)]
struct SavedHardwareState {
    method: LinuxPwmMethod,
    initial_enable: Option<i32>,
    initial_cur_state: Option<u32>,
}

/// Non-blocking sysfs reader using `O_NONBLOCK`.
fn read_sysfs_trimmed(path: &Path) -> Option<String> {
    let mut file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(path)
        .ok()?;
    let mut buf = String::with_capacity(64);
    file.read_to_string(&mut buf).ok()?;
    Some(buf.trim().to_string())
}

/// Non-blocking sysfs writer using `O_NONBLOCK`.
fn write_sysfs_str(path: &Path, val: &str) -> std::io::Result<()> {
    let mut file = OpenOptions::new()
        .write(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(path)?;
    file.write_all(val.as_bytes())?;
    Ok(())
}

fn check_writable_or_root(path: &Path) -> bool {
    if unsafe { libc::geteuid() } == 0 {
        if let Ok(meta) = fs::metadata(path) {
            return !meta.permissions().readonly();
        }
    }
    OpenOptions::new()
        .write(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(path)
        .is_ok()
}

/// Stable device identifier derived from the `device` symlink target in `/sys/class/hwmon/hwmonX`.
fn resolve_stable_device_tag(hwmon_dir: &Path, chip_name: &str) -> String {
    let dev_link = hwmon_dir.join("device");
    if let Ok(target) = fs::read_link(&dev_link) {
        if let Some(name) = target.file_name().and_then(|s| s.to_str()) {
            return format!("{}@{}", chip_name, name);
        }
    }
    if let Ok(canon) = fs::canonicalize(hwmon_dir) {
        if let Some(parent) = canon.parent().and_then(|p| p.parent()) {
            if let Some(name) = parent.file_name().and_then(|s| s.to_str()) {
                return format!("{}@{}", chip_name, name);
            }
        }
    }
    chip_name.to_string()
}

/// Manages Linux `/sys/class/hwmon` and `/sys/class/thermal` discovery, polling, PWM writes,
/// and deterministic hardware restoration on exit.
#[derive(Debug, Default)]
pub struct HwmonManager {
    pub temp_sensors: Vec<HardwareTempSensor>,
    pub fan_sensors: Vec<HardwareFanSensor>,
    pub pwm_controls: Vec<HardwarePwmControl>,
    /// Tracks every hardware control channel touched by CachyFanControl so we can restore BIOS control.
    touched_controls: HashMap<String, SavedHardwareState>,
}

impl HwmonManager {
    pub fn new() -> Self {
        let mut mgr = Self::default();
        mgr.discover();
        mgr
    }

    /// Scans `/sys/class/hwmon/` and `/sys/class/thermal/` for all temperature sensors,
    /// fan tachometers, and PWM/cooling fan controls.
    pub fn discover(&mut self) {
        let mut temps = Vec::new();
        let mut fans = Vec::new();
        let mut pwms = Vec::new();

        let hwmon_root = Path::new("/sys/class/hwmon");
        if let Ok(entries) = fs::read_dir(hwmon_root) {
            let mut dirs: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
            dirs.sort();

            for dir in dirs {
                let chip_name = read_sysfs_trimmed(&dir.join("name"))
                    .unwrap_or_else(|| "unknown".to_string());
                let stable_tag = resolve_stable_device_tag(&dir, &chip_name);

                let Ok(files) = fs::read_dir(&dir) else {
                    continue;
                };
                let mut file_names: Vec<String> = files
                    .flatten()
                    .filter_map(|e| e.file_name().into_string().ok())
                    .collect();
                file_names.sort();

                for fname in &file_names {
                    // 1. Temperature inputs (`tempX_input`)
                    if let Some(prefix) = fname.strip_suffix("_input") {
                        if prefix.starts_with("temp") {
                            let input_path = dir.join(fname);
                            let label = read_sysfs_trimmed(&dir.join(format!("{}_label", prefix)))
                                .unwrap_or_else(|| prefix.to_uppercase());
                            let id = format!("hwmon:{}:{}", stable_tag, prefix);
                            let friendly_name = format!("{} - {}", chip_name, label);

                            let val = read_sysfs_trimmed(&input_path)
                                .and_then(|s| s.parse::<f32>().ok())
                                .map(|v| v / 1000.0);
                            let min = read_sysfs_trimmed(&dir.join(format!("{}_min", prefix)))
                                .and_then(|s| s.parse::<f32>().ok())
                                .map(|v| v / 1000.0);
                            let max = read_sysfs_trimmed(&dir.join(format!("{}_max", prefix)))
                                .and_then(|s| s.parse::<f32>().ok())
                                .map(|v| v / 1000.0);
                            let crit = read_sysfs_trimmed(&dir.join(format!("{}_crit", prefix)))
                                .and_then(|s| s.parse::<f32>().ok())
                                .map(|v| v / 1000.0);

                            temps.push(HardwareTempSensor {
                                id,
                                name: friendly_name,
                                chip_name: chip_name.clone(),
                                sysfs_path: input_path.to_string_lossy().into_owned(),
                                value: val,
                                min,
                                max,
                                crit,
                            });
                        } else if prefix.starts_with("fan") {
                            // 2. Fan RPM tachometers (`fanX_input`)
                            let input_path = dir.join(fname);
                            let label = read_sysfs_trimmed(&dir.join(format!("{}_label", prefix)))
                                .unwrap_or_else(|| prefix.to_uppercase());
                            let id = format!("hwmon:{}:{}", stable_tag, prefix);
                            let friendly_name = format!("{} - {}", chip_name, label);
                            let rpm = read_sysfs_trimmed(&input_path)
                                .and_then(|s| s.parse::<f32>().ok());

                            fans.push(HardwareFanSensor {
                                id,
                                name: friendly_name,
                                chip_name: chip_name.clone(),
                                sysfs_path: input_path.to_string_lossy().into_owned(),
                                rpm,
                            });
                        }
                    }

                    // 3a. Standard PWM duty cycle file (`pwm1`, `pwm2`, ...)
                    if fname.starts_with("pwm") && !fname.contains('_') {
                        let pwm_path = dir.join(fname);
                        let enable_file = format!("{}_enable", fname);
                        let enable_path = if file_names.contains(&enable_file) {
                            Some(dir.join(&enable_file).to_string_lossy().into_owned())
                        } else {
                            None
                        };
                        let id = format!("hwmon:{}:{}", stable_tag, fname);
                        let label = read_sysfs_trimmed(&dir.join(format!("{}_label", fname)))
                            .unwrap_or_else(|| fname.to_uppercase());
                        let friendly_name = format!("{} - {}", chip_name, label);
                        let cur_raw = read_sysfs_trimmed(&pwm_path)
                            .and_then(|s| s.parse::<f32>().ok());
                        let cur_pct = cur_raw.map(|v| (v / 255.0 * 100.0).clamp(0.0, 100.0));
                        let mode = enable_path
                            .as_ref()
                            .and_then(|p| read_sysfs_trimmed(Path::new(p)))
                            .and_then(|s| s.parse::<i32>().ok());
                        let writable = check_writable_or_root(&pwm_path);

                        pwms.push(HardwarePwmControl {
                            id,
                            name: friendly_name,
                            chip_name: chip_name.clone(),
                            method: LinuxPwmMethod::StandardPwm {
                                pwm_path: pwm_path.to_string_lossy().into_owned(),
                                enable_path,
                            },
                            current_pct: cur_pct,
                            pwm_enable_mode: mode,
                            writable,
                        });
                    }

                    // 3b. Enable-only WMI fan controller (`pwmX_enable` exists without `pwmX`, e.g. `hp-wmi`)
                    if let Some(pwm_prefix) = fname.strip_suffix("_enable") {
                        if pwm_prefix.starts_with("pwm") && !file_names.contains(&pwm_prefix.to_string()) {
                            let enable_path = dir.join(fname);
                            let id = format!("hwmon:{}:{}", stable_tag, pwm_prefix);
                            let friendly_name = format!("{} - {} (Auto/Full-Blast WMI)", chip_name, pwm_prefix.to_uppercase());
                            let mode = read_sysfs_trimmed(&enable_path)
                                .and_then(|s| s.parse::<i32>().ok());
                            // In kernel hwmon / hp-wmi, pwm1_enable=0 is Full Speed (100%), >=2 is Auto
                            let cur_pct = match mode {
                                Some(0) => Some(100.0),
                                Some(_) => Some(50.0),
                                None => None,
                            };
                            let writable = check_writable_or_root(&enable_path);

                            pwms.push(HardwarePwmControl {
                                id,
                                name: friendly_name,
                                chip_name: chip_name.clone(),
                                method: LinuxPwmMethod::EnableOnlyToggle {
                                    enable_path: enable_path.to_string_lossy().into_owned(),
                                },
                                current_pct: cur_pct,
                                pwm_enable_mode: mode,
                                writable,
                            });
                        }
                    }
                }
            }
        }

        // 4. Discover ACPI Thermal `Fan` Cooling Devices (`/sys/class/thermal/cooling_device*`)
        let thermal_root = Path::new("/sys/class/thermal");
        if let Ok(entries) = fs::read_dir(thermal_root) {
            let mut dirs: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
            dirs.sort();

            for dir in dirs {
                let Some(dir_name) = dir.file_name().and_then(|s| s.to_str()) else {
                    continue;
                };
                if !dir_name.starts_with("cooling_device") {
                    continue;
                }
                let dev_type = read_sysfs_trimmed(&dir.join("type")).unwrap_or_default();
                if !dev_type.eq_ignore_ascii_case("Fan") {
                    continue;
                }
                let max_state = read_sysfs_trimmed(&dir.join("max_state"))
                    .and_then(|s| s.parse::<u32>().ok())
                    .unwrap_or(0);
                if max_state == 0 {
                    continue;
                }
                let cur_state_path = dir.join("cur_state");
                let cur_state = read_sysfs_trimmed(&cur_state_path)
                    .and_then(|s| s.parse::<u32>().ok());
                let cur_pct = cur_state.map(|s| (s as f32 / max_state as f32 * 100.0).clamp(0.0, 100.0));
                let writable = check_writable_or_root(&cur_state_path);
                let id = format!("thermal:{}:{}", dev_type.to_lowercase(), dir_name);
                let friendly_name = format!("ACPI Fan ({})", dir_name);

                pwms.push(HardwarePwmControl {
                    id,
                    name: friendly_name,
                    chip_name: "acpi_thermal".to_string(),
                    method: LinuxPwmMethod::ThermalCoolingDevice {
                        cur_state_path: cur_state_path.to_string_lossy().into_owned(),
                        max_state,
                    },
                    current_pct: cur_pct,
                    pwm_enable_mode: Some(1),
                    writable,
                });
            }
        }

        self.temp_sensors = temps;
        self.fan_sensors = fans;
        self.pwm_controls = pwms;
    }

    /// Non-blocking refresh of all hardware sensor values and PWM readbacks.
    pub fn poll(&mut self) {
        for t in &mut self.temp_sensors {
            t.value = read_sysfs_trimmed(Path::new(&t.sysfs_path))
                .and_then(|s| s.parse::<f32>().ok())
                .map(|v| v / 1000.0);
        }
        for f in &mut self.fan_sensors {
            f.rpm = read_sysfs_trimmed(Path::new(&f.sysfs_path))
                .and_then(|s| s.parse::<f32>().ok());
        }
        for p in &mut self.pwm_controls {
            match &p.method {
                LinuxPwmMethod::StandardPwm {
                    pwm_path,
                    enable_path,
                } => {
                    p.current_pct = read_sysfs_trimmed(Path::new(pwm_path))
                        .and_then(|s| s.parse::<f32>().ok())
                        .map(|v| (v / 255.0 * 100.0).clamp(0.0, 100.0));
                    p.pwm_enable_mode = enable_path
                        .as_ref()
                        .and_then(|ep| read_sysfs_trimmed(Path::new(ep)))
                        .and_then(|s| s.parse::<i32>().ok());
                }
                LinuxPwmMethod::EnableOnlyToggle { enable_path } => {
                    let mode = read_sysfs_trimmed(Path::new(enable_path))
                        .and_then(|s| s.parse::<i32>().ok());
                    p.pwm_enable_mode = mode;
                    p.current_pct = match mode {
                        Some(0) => Some(100.0),
                        Some(_) => Some(50.0),
                        None => None,
                    };
                }
                LinuxPwmMethod::ThermalCoolingDevice {
                    cur_state_path,
                    max_state,
                } => {
                    let cur = read_sysfs_trimmed(Path::new(cur_state_path))
                        .and_then(|s| s.parse::<u32>().ok());
                    p.current_pct =
                        cur.map(|s| (s as f32 / (*max_state).max(1) as f32 * 100.0).clamp(0.0, 100.0));
                }
            }
        }
    }

    /// Applies a `0.0..=100.0%` PWM command to a hardware control channel, saving the original
    /// BIOS `pwmX_enable` state on first touch.
    pub fn apply_command(&mut self, hardware_id: &str, command_pct: f32) -> std::io::Result<()> {
        let Some(ctrl) = self.pwm_controls.iter_mut().find(|c| c.id == hardware_id) else {
            return Ok(());
        };
        let clamped_pct = command_pct.clamp(0.0, 100.0);

        // Snapshot initial BIOS state before modifying for the first time
        if !self.touched_controls.contains_key(hardware_id) {
            let saved = match &ctrl.method {
                LinuxPwmMethod::StandardPwm { enable_path, .. } => {
                    let init_en = enable_path
                        .as_ref()
                        .and_then(|ep| read_sysfs_trimmed(Path::new(ep)))
                        .and_then(|s| s.parse::<i32>().ok());
                    SavedHardwareState {
                        method: ctrl.method.clone(),
                        initial_enable: init_en,
                        initial_cur_state: None,
                    }
                }
                LinuxPwmMethod::EnableOnlyToggle { enable_path } => {
                    let init_en = read_sysfs_trimmed(Path::new(enable_path))
                        .and_then(|s| s.parse::<i32>().ok());
                    SavedHardwareState {
                        method: ctrl.method.clone(),
                        initial_enable: init_en,
                        initial_cur_state: None,
                    }
                }
                LinuxPwmMethod::ThermalCoolingDevice { cur_state_path, .. } => {
                    let init_state = read_sysfs_trimmed(Path::new(cur_state_path))
                        .and_then(|s| s.parse::<u32>().ok());
                    SavedHardwareState {
                        method: ctrl.method.clone(),
                        initial_enable: None,
                        initial_cur_state: init_state,
                    }
                }
            };
            self.touched_controls.insert(hardware_id.to_string(), saved);
        }

        match &ctrl.method {
            LinuxPwmMethod::StandardPwm {
                pwm_path,
                enable_path,
            } => {
                // Ensure manual control mode (`pwmX_enable = 1`)
                if let Some(ep) = enable_path {
                    if ctrl.pwm_enable_mode != Some(1) {
                        let _ = write_sysfs_str(Path::new(ep), "1\n");
                        ctrl.pwm_enable_mode = Some(1);
                    }
                }
                let pwm_byte = ((clamped_pct / 100.0) * 255.0).round().clamp(0.0, 255.0) as u8;
                write_sysfs_str(Path::new(pwm_path), &format!("{}\n", pwm_byte))?;
                ctrl.current_pct = Some(clamped_pct);
            }
            LinuxPwmMethod::EnableOnlyToggle { enable_path } => {
                // For `pwmX_enable`-only WMI chips (like hp-wmi): >= 85% triggers full speed (`0`), else auto (`2`)
                let target_mode = if clamped_pct >= 85.0 { 0 } else { 2 };
                if ctrl.pwm_enable_mode != Some(target_mode) {
                    write_sysfs_str(Path::new(enable_path), &format!("{}\n", target_mode))?;
                    ctrl.pwm_enable_mode = Some(target_mode);
                }
                ctrl.current_pct = Some(if target_mode == 0 { 100.0 } else { clamped_pct });
            }
            LinuxPwmMethod::ThermalCoolingDevice {
                cur_state_path,
                max_state,
            } => {
                let state = ((clamped_pct / 100.0) * (*max_state as f32))
                    .round()
                    .clamp(0.0, *max_state as f32) as u32;
                write_sysfs_str(Path::new(cur_state_path), &format!("{}\n", state))?;
                ctrl.current_pct = Some(clamped_pct);
            }
        }
        Ok(())
    }

    /// Relinquishes control of a single hardware channel back to BIOS/firmware.
    pub fn restore_control_to_bios(&mut self, hardware_id: &str) {
        if let Some(saved) = self.touched_controls.remove(hardware_id) {
            Self::restore_saved_state(hardware_id, &saved);
        }
    }

    /// Relinquishes ALL touched hardware fan channels back to automatic BIOS/firmware control (`pwmX_enable = 2`).
    pub fn restore_all_to_bios(&mut self) {
        for (hw_id, saved) in self.touched_controls.drain() {
            Self::restore_saved_state(&hw_id, &saved);
        }
    }

    fn restore_saved_state(hw_id: &str, saved: &SavedHardwareState) {
        match &saved.method {
            LinuxPwmMethod::StandardPwm { enable_path, .. } => {
                if let Some(ep) = enable_path {
                    // Restore to initial mode if >= 2, otherwise `2` (BIOS auto)
                    let restore_mode = saved.initial_enable.filter(|&m| m >= 2).unwrap_or(2);
                    log::info!(
                        "Restoring {} ({}) to BIOS auto mode (pwm_enable={})",
                        hw_id,
                        ep,
                        restore_mode
                    );
                    let _ = write_sysfs_str(Path::new(ep), &format!("{}\n", restore_mode));
                }
            }
            LinuxPwmMethod::EnableOnlyToggle { enable_path } => {
                let restore_mode = saved.initial_enable.filter(|&m| m >= 2).unwrap_or(2);
                log::info!(
                    "Restoring WMI fan control {} ({}) to BIOS auto mode (pwm_enable={})",
                    hw_id,
                    enable_path,
                    restore_mode
                );
                let _ = write_sysfs_str(Path::new(enable_path), &format!("{}\n", restore_mode));
            }
            LinuxPwmMethod::ThermalCoolingDevice { cur_state_path, .. } => {
                if let Some(init_state) = saved.initial_cur_state {
                    log::info!(
                        "Restoring ACPI cooling device {} ({}) to state {}",
                        hw_id,
                        cur_state_path,
                        init_state
                    );
                    let _ = write_sysfs_str(Path::new(cur_state_path), &format!("{}\n", init_state));
                }
            }
        }
    }
}

impl Drop for HwmonManager {
    fn drop(&mut self) {
        self.restore_all_to_bios();
    }
}
