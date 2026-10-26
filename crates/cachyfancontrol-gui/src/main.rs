//! `cachyfancontrol-gui` — Wayland-native card-based GUI for CachyFanControl.
//!
//! Mirrors the 3-column card layout of original FanControl:
//! - Column 1: **Controls (Fans)** — PWM/fan cards, manual vs curve mode, Start/Stop%, StepUp/StepDown, Calibration & Avoid regions
//! - Column 2: **Fan Curves** — Interactive `egui_plot` curves for `Graph`, `Linear`, `Auto` (OLS Trend PID), `Mix`, `Trigger`, `Flat`, `Sync`
//! - Column 3: **Custom Sensors & Hardware Telemetry** — `TimeAverage`, `Mix`, `Offset`, `File` custom sensors + `/sys/class/hwmon` live sensors

use cachyfancontrol_core::curves::{
    CommandMode, CurvePoint, FanCurveConfig, FanCurveKind, MixFunction,
};
use cachyfancontrol_core::engine::{
    AppConfig, DaemonRequest, DaemonResponse, EngineSnapshot, FanControlEngine,
    DEFAULT_CONFIG_PATH, DEFAULT_SOCKET_PATH,
};
use cachyfancontrol_core::hwmon::HwmonManager;
use cachyfancontrol_core::sensors::{CustomSensorConfig, CustomSensorKind, SensorMixFunction};
use cachyfancontrol_core::utilities::HysteresisConfig;
use eframe::egui::{self, Color32, CornerRadius, Frame, Margin, RichText, ScrollArea, Stroke, Vec2};
use egui_plot::{Line, Plot, PlotPoints, Points, VLine};
use std::env;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const CACHY_TEAL: Color32 = Color32::from_rgb(0, 210, 175);
const CACHY_BLUE: Color32 = Color32::from_rgb(70, 160, 245);
const CACHY_AMBER: Color32 = Color32::from_rgb(245, 175, 50);
const CACHY_RED: Color32 = Color32::from_rgb(235, 85, 85);
const CARD_BG: Color32 = Color32::from_rgb(26, 29, 36);
const CARD_BORDER: Color32 = Color32::from_rgb(48, 55, 68);

struct CachyFanControlApp {
    socket_candidates: Vec<PathBuf>,
    active_socket: Option<PathBuf>,
    fallback_engine: FanControlEngine,
    fallback_config_path: PathBuf,
    snapshot: EngineSnapshot,
    working_config: AppConfig,
    config_dirty: bool,
    status_banner: String,
    last_poll: Instant,
    next_id_counter: u64,
}

impl CachyFanControlApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self::configure_theme(&cc.egui_ctx);

        let runtime_dir = env::var("XDG_RUNTIME_DIR").unwrap_or_else(|_| "/tmp".to_string());
        let config_dir = env::var("XDG_CONFIG_HOME").unwrap_or_else(|_| {
            let home = env::var("HOME").unwrap_or_else(|_| "/tmp".to_string());
            format!("{}/.config", home)
        });

        let socket_candidates = vec![
            PathBuf::from(DEFAULT_SOCKET_PATH),
            PathBuf::from(format!("{}/cachyfancontrol.sock", runtime_dir)),
            PathBuf::from("/tmp/cachyfancontrol.sock"),
        ];
        let fallback_config_path = PathBuf::from(format!("{}/cachyfancontrol/config.json", config_dir));

        let hwmon = HwmonManager::new();
        let cfg = if Path::new(DEFAULT_CONFIG_PATH).exists() {
            AppConfig::load_or_create(Path::new(DEFAULT_CONFIG_PATH), &hwmon)
        } else {
            AppConfig::load_or_create(&fallback_config_path, &hwmon)
        };
        let fallback_engine = FanControlEngine::new(hwmon, cfg);
        let snapshot = fallback_engine.snapshot();
        let working_config = snapshot.config.clone();

        let mut app = Self {
            socket_candidates,
            active_socket: None,
            fallback_engine,
            fallback_config_path,
            snapshot,
            working_config,
            config_dirty: false,
            status_banner: "Initialized CachyFanControl.".to_string(),
            last_poll: Instant::now() - Duration::from_secs(2),
            next_id_counter: 100,
        };
        app.poll_backend(true);
        app
    }

    fn configure_theme(ctx: &egui::Context) {
        let mut visuals = egui::Visuals::dark();
        visuals.window_fill = Color32::from_rgb(17, 19, 24);
        visuals.panel_fill = Color32::from_rgb(17, 19, 24);
        visuals.faint_bg_color = Color32::from_rgb(24, 27, 34);
        visuals.selection.bg_fill = Color32::from_rgb(0, 140, 115);
        ctx.set_visuals(visuals);
    }

    fn send_daemon_request(&mut self, req: &DaemonRequest) -> Option<DaemonResponse> {
        let candidates = if let Some(s) = &self.active_socket {
            let mut v = vec![s.clone()];
            for c in &self.socket_candidates {
                if c != s {
                    v.push(c.clone());
                }
            }
            v
        } else {
            self.socket_candidates.clone()
        };

        for sock_path in candidates {
            if !sock_path.exists() {
                continue;
            }
            if let Ok(mut stream) = UnixStream::connect(&sock_path) {
                let _ = stream.set_read_timeout(Some(Duration::from_millis(800)));
                let _ = stream.set_write_timeout(Some(Duration::from_millis(800)));
                if let Ok(mut payload) = serde_json::to_string(req) {
                    payload.push('\n');
                    if stream.write_all(payload.as_bytes()).is_ok() {
                        let _ = stream.flush();
                        let mut reader = BufReader::new(stream);
                        let mut resp_line = String::new();
                        if reader.read_line(&mut resp_line).is_ok() && !resp_line.trim().is_empty() {
                            if let Ok(parsed) = serde_json::from_str::<DaemonResponse>(&resp_line) {
                                self.active_socket = Some(sock_path);
                                return Some(parsed);
                            }
                        }
                    }
                }
            }
        }
        self.active_socket = None;
        None
    }

    fn poll_backend(&mut self, sync_working_config: bool) {
        self.last_poll = Instant::now();
        if let Some(resp) = self.send_daemon_request(&DaemonRequest::GetSnapshot) {
            match resp {
                DaemonResponse::Ok { snapshot, message } => {
                    self.snapshot = *snapshot;
                    if sync_working_config && !self.config_dirty {
                        self.working_config = self.snapshot.config.clone();
                    }
                    if let Some(m) = message {
                        self.status_banner = m;
                    }
                    return;
                }
                DaemonResponse::Error { message } => {
                    self.status_banner = format!("Daemon error: {}", message);
                }
            }
        }

        // Fallback to embedded in-process engine when daemon is not running
        self.fallback_engine.tick();
        self.snapshot = self.fallback_engine.snapshot();
        if sync_working_config && !self.config_dirty {
            self.working_config = self.snapshot.config.clone();
        }
    }

    fn push_config_changes(&mut self, save_to_disk: bool) {
        let cfg = self.working_config.clone();
        if let Some(resp) = self.send_daemon_request(&DaemonRequest::UpdateConfig {
            config: cfg.clone(),
            save_to_disk,
        }) {
            match resp {
                DaemonResponse::Ok { snapshot, message } => {
                    self.snapshot = *snapshot;
                    self.working_config = self.snapshot.config.clone();
                    self.config_dirty = false;
                    if let Some(m) = message {
                        self.status_banner = m;
                    }
                    return;
                }
                DaemonResponse::Error { message } => {
                    self.status_banner = format!("Daemon error: {}", message);
                }
            }
        }

        // Embedded fallback
        self.fallback_engine.apply_config(cfg);
        if save_to_disk {
            match self
                .fallback_engine
                .config
                .save_atomic(&self.fallback_config_path)
            {
                Ok(_) => {
                    self.status_banner = format!(
                        "Applied & saved to {}",
                        self.fallback_config_path.display()
                    );
                }
                Err(e) => {
                    self.status_banner = format!("Applied in memory (save failed: {})", e);
                }
            }
        } else {
            self.status_banner = "Applied configuration to active engine.".to_string();
        }
        self.snapshot = self.fallback_engine.snapshot();
        self.working_config = self.snapshot.config.clone();
        self.config_dirty = false;
    }

    fn all_temperature_sources(&self) -> Vec<(String, String, Option<f32>)> {
        let mut out = Vec::new();
        for t in &self.snapshot.hw_temps {
            out.push((t.id.clone(), t.name.clone(), t.value));
        }
        for cs in &self.working_config.custom_sensors {
            let val = self
                .snapshot
                .custom_sensor_telemetry
                .get(&cs.id)
                .and_then(|t| t.value);
            out.push((cs.id.clone(), format!("[Custom] {}", cs.name), val));
        }
        out
    }

    fn card_frame() -> Frame {
        Frame::new()
            .fill(CARD_BG)
            .stroke(Stroke::new(1.0_f32, CARD_BORDER))
            .corner_radius(CornerRadius::same(8))
            .inner_margin(Margin::same(12))
    }

    fn render_top_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.heading(
                RichText::new("CachyFanControl")
                    .color(CACHY_TEAL)
                    .strong(),
            );
            ui.separator();

            // Connection mode badge
            if let Some(sock) = &self.active_socket {
                ui.label(
                    RichText::new(format!("● Daemon Connected ({})", sock.display()))
                        .color(CACHY_TEAL)
                        .small(),
                );
            } else if self.snapshot.running_as_root {
                ui.label(
                    RichText::new("● Direct Engine (Root)")
                        .color(CACHY_TEAL)
                        .small(),
                );
            } else {
                ui.label(
                    RichText::new("● Embedded Engine (Start cachyfancontrold.service for root PWM writes)")
                        .color(CACHY_AMBER)
                        .small(),
                );
            }

            ui.separator();
            ui.label("Profile:");
            if ui
                .add(
                    egui::TextEdit::singleline(&mut self.working_config.profile_name)
                        .desired_width(170.0),
                )
                .changed()
            {
                self.config_dirty = true;
            }

            if ui
                .button(RichText::new("💾 Save & Apply").color(CACHY_TEAL).strong())
                .clicked()
            {
                self.push_config_changes(true);
            }

            if ui.button("🔄 Refresh Hardware").clicked() {
                if let Some(DaemonResponse::Ok { snapshot, message }) =
                    self.send_daemon_request(&DaemonRequest::RefreshHardware)
                {
                    self.snapshot = *snapshot;
                    if let Some(m) = message {
                        self.status_banner = m;
                    }
                } else {
                    self.fallback_engine.hwmon.discover();
                    self.fallback_engine.tick();
                    self.snapshot = self.fallback_engine.snapshot();
                    self.status_banner = "Re-scanned /sys/class/hwmon sensors.".to_string();
                }
            }

            if ui
                .button(RichText::new("🛡 Relinquish All to BIOS").color(CACHY_AMBER))
                .on_hover_text("Immediately disables manual PWM override and returns all fans to motherboard BIOS/EC automatic control (pwmX_enable = 2)")
                .clicked()
            {
                if let Some(DaemonResponse::Ok { snapshot, message }) =
                    self.send_daemon_request(&DaemonRequest::RelinquishAllToBios)
                {
                    self.snapshot = *snapshot;
                    self.working_config = self.snapshot.config.clone();
                    self.config_dirty = false;
                    if let Some(m) = message {
                        self.status_banner = m;
                    }
                } else {
                    for c in &mut self.working_config.controls {
                        c.enabled = false;
                    }
                    self.fallback_engine.hwmon.restore_all_to_bios();
                    self.push_config_changes(false);
                    self.status_banner =
                        "All fans relinquished to BIOS automatic mode.".to_string();
                }
            }

            ui.separator();
            self.render_add_curve_menu(ui);
            self.render_add_custom_sensor_menu(ui);
        });
    }

    fn render_add_curve_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button(RichText::new("➕ Add Fan Curve").color(CACHY_BLUE), |ui| {
            let default_sensor = self
                .snapshot
                .hw_temps
                .first()
                .map(|t| t.id.clone())
                .unwrap_or_else(|| "hwmon:cpu:temp1".to_string());

            if ui.button("📈 Graph Fan Curve (Piecewise Points + Hysteresis)").clicked() {
                self.next_id_counter += 1;
                self.working_config.curves.push(FanCurveConfig {
                    id: format!("curve:graph_{}", self.next_id_counter),
                    name: format!("Graph Curve {}", self.working_config.curves.len() + 1),
                    command_mode: CommandMode::Percent,
                    kind: FanCurveKind::Graph {
                        sensor_id: default_sensor.clone(),
                        points: vec![
                            CurvePoint { temperature: 35.0, command: 25.0 },
                            CurvePoint { temperature: 55.0, command: 50.0 },
                            CurvePoint { temperature: 75.0, command: 85.0 },
                            CurvePoint { temperature: 85.0, command: 100.0 },
                        ],
                        hysteresis: HysteresisConfig::default(),
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui
                .button("🎯 Auto Fan Curve (OLS Linear Regression Trend + Creep PID)")
                .clicked()
            {
                self.next_id_counter += 1;
                self.working_config.curves.push(FanCurveConfig {
                    id: format!("curve:auto_{}", self.next_id_counter),
                    name: format!("Auto Load Curve {}", self.working_config.curves.len() + 1),
                    command_mode: CommandMode::Percent,
                    kind: FanCurveKind::Auto {
                        sensor_id: default_sensor.clone(),
                        idle_temp: 40.0,
                        load_temp: 75.0,
                        deadband: 3.0,
                        min_fan_speed: 25.0,
                        max_fan_speed: 100.0,
                        step: 3.0,
                        selected_response_time: 2,
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui.button("📐 Linear Fan Curve (Min/Max Slope)").clicked() {
                self.next_id_counter += 1;
                self.working_config.curves.push(FanCurveConfig {
                    id: format!("curve:linear_{}", self.next_id_counter),
                    name: format!("Linear Curve {}", self.working_config.curves.len() + 1),
                    command_mode: CommandMode::Percent,
                    kind: FanCurveKind::Linear {
                        sensor_id: default_sensor.clone(),
                        min_temp: 40.0,
                        max_temp: 80.0,
                        min_fan_speed: 25.0,
                        max_fan_speed: 100.0,
                        hysteresis: HysteresisConfig::default(),
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui.button("🔀 Mix Fan Curve (Max / Min / Avg / Sum / Subtract)").clicked() {
                self.next_id_counter += 1;
                let sub_ids: Vec<String> = self
                    .working_config
                    .curves
                    .iter()
                    .take(2)
                    .map(|c| c.id.clone())
                    .collect();
                self.working_config.curves.push(FanCurveConfig {
                    id: format!("curve:mix_{}", self.next_id_counter),
                    name: format!("Mix Curve {}", self.working_config.curves.len() + 1),
                    command_mode: CommandMode::Percent,
                    kind: FanCurveKind::Mix {
                        curve_ids: sub_ids,
                        function: MixFunction::Max,
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui.button("⚡ Trigger Fan Curve (Dual-Threshold Schmitt Trigger)").clicked() {
                self.next_id_counter += 1;
                self.working_config.curves.push(FanCurveConfig {
                    id: format!("curve:trigger_{}", self.next_id_counter),
                    name: format!("Trigger Curve {}", self.working_config.curves.len() + 1),
                    command_mode: CommandMode::Percent,
                    kind: FanCurveKind::Trigger {
                        sensor_id: default_sensor,
                        idle_temp: 45.0,
                        load_temp: 70.0,
                        idle_fan_speed: 30.0,
                        load_fan_speed: 85.0,
                        response_time_up: 2,
                        response_time_down: 4,
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui.button("➖ Flat Fan Curve (Constant %)").clicked() {
                self.next_id_counter += 1;
                self.working_config.curves.push(FanCurveConfig {
                    id: format!("curve:flat_{}", self.next_id_counter),
                    name: format!("Flat Curve {}", self.working_config.curves.len() + 1),
                    command_mode: CommandMode::Percent,
                    kind: FanCurveKind::Flat { command: 50.0 },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui.button("🔗 Sync Fan Curve (Mirror Control + Offset)").clicked() {
                self.next_id_counter += 1;
                let ctrl_id = self
                    .working_config
                    .controls
                    .first()
                    .map(|c| c.id.clone())
                    .unwrap_or_default();
                self.working_config.curves.push(FanCurveConfig {
                    id: format!("curve:sync_{}", self.next_id_counter),
                    name: format!("Sync Curve {}", self.working_config.curves.len() + 1),
                    command_mode: CommandMode::Percent,
                    kind: FanCurveKind::Sync {
                        control_id: ctrl_id,
                        offset: 0.0,
                        proportional: false,
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }
        });
    }

    fn render_add_custom_sensor_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button(RichText::new("➕ Add Custom Sensor").color(CACHY_TEAL), |ui| {
            let default_sensor = self
                .snapshot
                .hw_temps
                .first()
                .map(|t| t.id.clone())
                .unwrap_or_else(|| "hwmon:cpu:temp1".to_string());

            if ui.button("⏱ Time Average Sensor (Moving Ring Buffer)").clicked() {
                self.next_id_counter += 1;
                self.working_config.custom_sensors.push(CustomSensorConfig {
                    id: format!("custom:avg_{}", self.next_id_counter),
                    name: format!("Time Avg {}", self.working_config.custom_sensors.len() + 1),
                    kind: CustomSensorKind::TimeAverage {
                        source_id: default_sensor.clone(),
                        selected_time: 10,
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui.button("🌡 Mix Sensor (Max / Min / Avg / Sum / Subtract)").clicked() {
                self.next_id_counter += 1;
                let sources: Vec<String> = self
                    .snapshot
                    .hw_temps
                    .iter()
                    .take(2)
                    .map(|t| t.id.clone())
                    .collect();
                self.working_config.custom_sensors.push(CustomSensorConfig {
                    id: format!("custom:mix_{}", self.next_id_counter),
                    name: format!("Mix Sensor {}", self.working_config.custom_sensors.len() + 1),
                    kind: CustomSensorKind::Mix {
                        source_ids: sources,
                        function: SensorMixFunction::Max,
                        allow_missing_sensor: true,
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui.button("➕ Offset Sensor (°C or Proportional %)").clicked() {
                self.next_id_counter += 1;
                self.working_config.custom_sensors.push(CustomSensorConfig {
                    id: format!("custom:offset_{}", self.next_id_counter),
                    name: format!("Offset Sensor {}", self.working_config.custom_sensors.len() + 1),
                    kind: CustomSensorKind::Offset {
                        source_id: default_sensor,
                        offset: 5.0,
                        proportional: false,
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }

            if ui.button("📄 File Sensor (External Path)").clicked() {
                self.next_id_counter += 1;
                self.working_config.custom_sensors.push(CustomSensorConfig {
                    id: format!("custom:file_{}", self.next_id_counter),
                    name: format!("File Sensor {}", self.working_config.custom_sensors.len() + 1),
                    kind: CustomSensorKind::File {
                        path: "/sys/class/thermal/thermal_zone0/temp".to_string(),
                    },
                });
                self.push_config_changes(false);
                ui.close_menu();
            }
        });
    }

    fn render_controls_column(&mut self, ui: &mut egui::Ui) {
        ui.heading(RichText::new("🌀 Controls (Fans)").color(CACHY_TEAL));
        ui.label(
            RichText::new("Hardware PWM & cooling channels with slew-rate & kickstart protection")
                .small()
                .weak(),
        );
        ui.add_space(6.0);

        let curve_choices: Vec<(String, String)> = self
            .working_config
            .curves
            .iter()
            .map(|c| (c.id.clone(), c.name.clone()))
            .collect();

        let fan_choices: Vec<(String, String)> = self
            .snapshot
            .hw_fans
            .iter()
            .map(|f| {
                let rpm_txt = f.rpm.map(|r| format!(" ({:.0} RPM)", r)).unwrap_or_default();
                (f.id.clone(), format!("{}{}", f.name, rpm_txt))
            })
            .collect();

        let mut changed = false;
        let mut calibrate_id: Option<String> = None;

        for ctrl in &mut self.working_config.controls {
            let tele = self.snapshot.control_telemetry.get(&ctrl.id);

            Self::card_frame().show(ui, |ui| {
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut ctrl.enabled, "").changed() {
                        changed = true;
                    }
                    if ui
                        .add(egui::TextEdit::singleline(&mut ctrl.name).desired_width(175.0))
                        .changed()
                    {
                        changed = true;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let rpm_val = tele.and_then(|t| t.paired_rpm);
                        if let Some(rpm) = rpm_val {
                            ui.label(
                                RichText::new(format!("{:.0} RPM", rpm))
                                    .color(CACHY_BLUE)
                                    .strong(),
                            );
                        } else {
                            ui.label(RichText::new("No RPM").weak().small());
                        }
                    });
                });

                ui.label(RichText::new(&ctrl.hardware_id).small().weak());

                // Live Command % Bar
                let cur_cmd = tele
                    .and_then(|t| t.current_command.or(t.hardware_readback_pct))
                    .unwrap_or(0.0);
                let tgt_cmd = tele.and_then(|t| t.target_command).unwrap_or(cur_cmd);
                let bar_text = if ctrl.enabled {
                    format!("Command: {:.1}%  (Target: {:.1}%)", cur_cmd, tgt_cmd)
                } else {
                    format!("BIOS Auto  (Readback: {:.1}%)", cur_cmd)
                };
                ui.add(
                    egui::ProgressBar::new((cur_cmd / 100.0).clamp(0.0, 1.0))
                        .text(bar_text)
                        .fill(if ctrl.enabled {
                            CACHY_TEAL
                        } else {
                            Color32::from_rgb(65, 75, 90)
                        }),
                );

                // Discrepancy / Avoid badges
                if let Some(t) = tele {
                    ui.horizontal_wrapped(|ui| {
                        if t.in_avoid_mode {
                            ui.label(
                                RichText::new("⚠ Avoid Region Active")
                                    .color(CACHY_AMBER)
                                    .small(),
                            );
                        }
                        if t.has_rpm_discrepancy {
                            ui.label(
                                RichText::new("⚠ RPM Discrepancy (>25%)")
                                    .color(CACHY_RED)
                                    .small(),
                            );
                        }
                        if t.is_command_stuck {
                            ui.label(
                                RichText::new("⚠ Command Readback Mismatch")
                                    .color(CACHY_AMBER)
                                    .small(),
                            );
                        }
                    });
                }

                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    if ui
                        .radio_value(&mut ctrl.manual_control, false, "Fan Curve")
                        .changed()
                    {
                        changed = true;
                    }
                    if ui
                        .radio_value(&mut ctrl.manual_control, true, "Manual %")
                        .changed()
                    {
                        changed = true;
                    }
                });

                if ctrl.manual_control {
                    if ui
                        .add(
                            egui::Slider::new(&mut ctrl.manual_control_value, 0.0..=100.0)
                                .suffix(" %")
                                .text("Manual Speed"),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                } else {
                    let selected_label = ctrl
                        .selected_fan_curve_id
                        .as_ref()
                        .and_then(|id| curve_choices.iter().find(|(cid, _)| cid == id))
                        .map(|(_, n)| n.as_str())
                        .unwrap_or("None (Select Curve)");

                    egui::ComboBox::from_id_salt(format!("curve_sel_{}", ctrl.id))
                        .selected_text(selected_label)
                        .width(230.0)
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_label(ctrl.selected_fan_curve_id.is_none(), "None")
                                .clicked()
                            {
                                ctrl.selected_fan_curve_id = None;
                                changed = true;
                            }
                            for (cid, cname) in &curve_choices {
                                let is_sel =
                                    ctrl.selected_fan_curve_id.as_deref() == Some(cid.as_str());
                                if ui.selectable_label(is_sel, cname).clicked() {
                                    ctrl.selected_fan_curve_id = Some(cid.clone());
                                    changed = true;
                                }
                            }
                        });
                }

                // Collapsible Start/Stop, Step Rate & Calibration parameters
                egui::CollapsingHeader::new("⚙ Step Rate, Start/Stop% & Calibration")
                    .id_salt(format!("adv_ctrl_{}", ctrl.id))
                    .show(ui, |ui| {
                        let paired_label = ctrl
                            .paired_fan_sensor_id
                            .as_ref()
                            .and_then(|fid| fan_choices.iter().find(|(id, _)| id == fid))
                            .map(|(_, name)| name.as_str())
                            .unwrap_or("Unpaired");

                        egui::ComboBox::from_label("Paired RPM")
                            .selected_text(paired_label)
                            .show_ui(ui, |ui| {
                                if ui
                                    .selectable_label(
                                        ctrl.paired_fan_sensor_id.is_none(),
                                        "Unpaired",
                                    )
                                    .clicked()
                                {
                                    ctrl.paired_fan_sensor_id = None;
                                    changed = true;
                                }
                                for (fid, fname) in &fan_choices {
                                    let is_sel = ctrl.paired_fan_sensor_id.as_deref()
                                        == Some(fid.as_str());
                                    if ui.selectable_label(is_sel, fname).clicked() {
                                        ctrl.paired_fan_sensor_id = Some(fid.clone());
                                        changed = true;
                                    }
                                }
                            });

                        ui.horizontal(|ui| {
                            ui.label("Start %:");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut ctrl.selected_start)
                                        .range(0.0..=100.0)
                                        .speed(0.5)
                                        .suffix("%"),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.label("Stop %:");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut ctrl.selected_stop)
                                        .range(0.0..=100.0)
                                        .speed(0.5)
                                        .suffix("%"),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                        });

                        ui.horizontal(|ui| {
                            ui.label("Step Up:");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut ctrl.selected_command_step_up)
                                        .range(0.5..=100.0)
                                        .speed(0.5)
                                        .suffix("%/s"),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.label("Down:");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut ctrl.selected_command_step_down)
                                        .range(0.5..=100.0)
                                        .speed(0.5)
                                        .suffix("%/s"),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                        });

                        ui.horizontal(|ui| {
                            ui.label("Offset:");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut ctrl.selected_offset)
                                        .range(-50.0..=50.0)
                                        .speed(0.5)
                                        .suffix("%"),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.label("Min %:");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut ctrl.minimum_percent)
                                        .range(0.0..=100.0)
                                        .speed(0.5)
                                        .suffix("%"),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                        });

                        if ui
                            .checkbox(&mut ctrl.force_apply, "Force apply command every tick")
                            .changed()
                        {
                            changed = true;
                        }

                        ui.horizontal(|ui| {
                            if ui.button("📏 Run Calibration").clicked() {
                                calibrate_id = Some(ctrl.id.clone());
                            }
                            if let Some(cal) = &mut ctrl.calibration {
                                ui.label(
                                    RichText::new(format!("({} pts)", cal.points.len()))
                                        .small()
                                        .color(CACHY_TEAL),
                                );
                            }
                        });

                        if let Some(cal) = &mut ctrl.calibration {
                            ui.label(
                                RichText::new(
                                    "Check 'Avoid' on contiguous points to skip resonant RPM bands:",
                                )
                                .small()
                                .weak(),
                            );
                            for pt in &mut cal.points {
                                ui.horizontal(|ui| {
                                    ui.label(format!("{:>3}% -> {:>4} RPM", pt.command, pt.rpm));
                                    if ui.checkbox(&mut pt.avoid, "Avoid").changed() {
                                        changed = true;
                                    }
                                });
                            }
                        }
                    });
            });
            ui.add_space(8.0);
        }

        if let Some(cid) = calibrate_id {
            if let Some(DaemonResponse::Ok { snapshot, message }) =
                self.send_daemon_request(&DaemonRequest::CalibrateControl {
                    control_id: cid.clone(),
                })
            {
                self.snapshot = *snapshot;
                self.working_config = self.snapshot.config.clone();
                if let Some(m) = message {
                    self.status_banner = m;
                }
            } else if let Ok(msg) = self.fallback_engine.calibrate_control(&cid) {
                self.snapshot = self.fallback_engine.snapshot();
                self.working_config = self.snapshot.config.clone();
                self.status_banner = msg;
            }
        } else if changed {
            self.push_config_changes(false);
        }
    }

    fn render_curves_column(&mut self, ui: &mut egui::Ui) {
        ui.heading(RichText::new("📈 Fan Curves").color(CACHY_BLUE));
        ui.label(
            RichText::new("Graph, Linear, Auto (OLS Trend + Creep PID), Mix, Trigger, Flat & Sync")
                .small()
                .weak(),
        );
        ui.add_space(6.0);

        let all_sensors = self.all_temperature_sources();
        let all_curves_list: Vec<(String, String)> = self
            .working_config
            .curves
            .iter()
            .map(|c| (c.id.clone(), c.name.clone()))
            .collect();
        let all_controls_list: Vec<(String, String)> = self
            .working_config
            .controls
            .iter()
            .map(|c| (c.id.clone(), c.name.clone()))
            .collect();

        let mut changed = false;
        let mut delete_idx: Option<usize> = None;

        for idx in 0..self.working_config.curves.len() {
            let curve_id = self.working_config.curves[idx].id.clone();
            let tele = self.snapshot.curve_telemetry.get(&curve_id).cloned();

            Self::card_frame().show(ui, |ui| {
                let curve = &mut self.working_config.curves[idx];
                let kind_badge = match &curve.kind {
                    FanCurveKind::Linear { .. } => "LINEAR",
                    FanCurveKind::Graph { .. } => "GRAPH",
                    FanCurveKind::Auto { .. } => "AUTO (OLS PID)",
                    FanCurveKind::Mix { .. } => "MIX",
                    FanCurveKind::Trigger { .. } => "TRIGGER",
                    FanCurveKind::Flat { .. } => "FLAT",
                    FanCurveKind::Sync { .. } => "SYNC",
                };

                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(format!("[{}]", kind_badge))
                            .color(CACHY_TEAL)
                            .strong()
                            .small(),
                    );
                    if ui
                        .add(egui::TextEdit::singleline(&mut curve.name).desired_width(160.0))
                        .changed()
                    {
                        changed = true;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.small_button("🗑").on_hover_text("Delete curve").clicked() {
                            delete_idx = Some(idx);
                        }
                        if let Some(t) = &tele {
                            if let Some(cmd) = t.command {
                                ui.label(
                                    RichText::new(format!(
                                        "{:.1} {}",
                                        cmd,
                                        curve.command_mode.unit()
                                    ))
                                    .color(CACHY_TEAL)
                                    .strong(),
                                );
                            }
                            if let Some(temp) = t.current_temp {
                                ui.label(
                                    RichText::new(format!("{:.1} °C |", temp))
                                        .color(CACHY_BLUE)
                                        .small(),
                                );
                            }
                        }
                    });
                });

                // Interactive 2D Plot for Graph, Linear, Auto, and Trigger curves
                let show_plot = matches!(
                    curve.kind,
                    FanCurveKind::Graph { .. }
                        | FanCurveKind::Linear { .. }
                        | FanCurveKind::Auto { .. }
                        | FanCurveKind::Trigger { .. }
                );
                if show_plot {
                    let sample_pts: PlotPoints = (20..=100)
                        .filter_map(|t| {
                            curve
                                .evaluate_preview_at(t as f32)
                                .map(|y| [t as f64, y as f64])
                        })
                        .collect();

                    Plot::new(format!("plot_{}", curve.id))
                        .height(135.0)
                        .allow_zoom(false)
                        .allow_drag(false)
                        .allow_scroll(false)
                        .include_x(20.0)
                        .include_x(100.0)
                        .include_y(0.0)
                        .include_y(100.0)
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                Line::new(sample_pts)
                                    .color(CACHY_TEAL)
                                    .width(2.0_f32)
                                    .name("Curve"),
                            );

                            if let FanCurveKind::Graph { points, .. } = &curve.kind {
                                let ctrl_pts: PlotPoints = points
                                    .iter()
                                    .map(|p| [p.temperature as f64, p.command as f64])
                                    .collect();
                                plot_ui.points(
                                    Points::new(ctrl_pts)
                                        .radius(4.5_f32)
                                        .color(CACHY_AMBER)
                                        .name("Control Points"),
                                );
                            }

                            if let FanCurveKind::Auto {
                                idle_temp,
                                load_temp,
                                deadband,
                                ..
                            } = &curve.kind
                            {
                                plot_ui.vline(
                                    VLine::new(*idle_temp as f64)
                                        .color(CACHY_BLUE)
                                        .name("Idle Temp"),
                                );
                                plot_ui.vline(
                                    VLine::new((*load_temp - *deadband) as f64)
                                        .color(CACHY_AMBER)
                                        .name("Load - Deadband"),
                                );
                                plot_ui.vline(
                                    VLine::new(*load_temp as f64)
                                        .color(CACHY_RED)
                                        .name("Load Temp"),
                                );
                            }

                            // Live Operating Point Crosshair
                            if let Some(t) = &tele {
                                if let (Some(cur_t), Some(cur_cmd)) = (t.current_temp, t.command) {
                                    plot_ui.points(
                                        Points::new(vec![[cur_t as f64, cur_cmd as f64]])
                                            .radius(6.0_f32)
                                            .color(CACHY_RED)
                                            .name("Live Operating Point"),
                                    );
                                }
                            }
                        });
                }

                // Curve-type specific controls
                match &mut curve.kind {
                    FanCurveKind::Linear {
                        sensor_id,
                        min_temp,
                        max_temp,
                        min_fan_speed,
                        max_fan_speed,
                        hysteresis,
                    } => {
                        changed |= Self::render_sensor_selector(
                            ui,
                            &format!("lin_sens_{}", curve.id),
                            sensor_id,
                            &all_sensors,
                        );
                        ui.horizontal(|ui| {
                            ui.label("Min Temp:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(min_temp)
                                        .range(0.0..=*max_temp - 1.0)
                                        .suffix(" °C"),
                                )
                                .changed();
                            ui.label("Max Temp:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(max_temp)
                                        .range(*min_temp + 1.0..=115.0)
                                        .suffix(" °C"),
                                )
                                .changed();
                        });
                        ui.horizontal(|ui| {
                            ui.label("Min Speed:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(min_fan_speed)
                                        .range(0.0..=100.0)
                                        .suffix(" %"),
                                )
                                .changed();
                            ui.label("Max Speed:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(max_fan_speed)
                                        .range(0.0..=100.0)
                                        .suffix(" %"),
                                )
                                .changed();
                        });
                        changed |=
                            Self::render_hysteresis_editor(ui, &curve.id, hysteresis);
                    }

                    FanCurveKind::Graph {
                        sensor_id,
                        points,
                        hysteresis,
                    } => {
                        changed |= Self::render_sensor_selector(
                            ui,
                            &format!("grp_sens_{}", curve.id),
                            sensor_id,
                            &all_sensors,
                        );

                        egui::CollapsingHeader::new(format!(
                            "📍 Control Points ({})",
                            points.len()
                        ))
                        .id_salt(format!("pts_{}", curve.id))
                        .default_open(true)
                        .show(ui, |ui| {
                            let mut rm_pt = None;
                            let len = points.len();
                            for (p_idx, pt) in points.iter_mut().enumerate() {
                                ui.horizontal(|ui| {
                                    ui.label(format!("#{}", p_idx + 1));
                                    changed |= ui
                                        .add(
                                            egui::DragValue::new(&mut pt.temperature)
                                                .range(0.0..=110.0)
                                                .speed(0.5)
                                                .suffix(" °C"),
                                        )
                                        .changed();
                                    ui.label("->");
                                    changed |= ui
                                        .add(
                                            egui::DragValue::new(&mut pt.command)
                                                .range(0.0..=100.0)
                                                .speed(0.5)
                                                .suffix(" %"),
                                        )
                                        .changed();
                                    if len > 2 && ui.small_button("✖").clicked() {
                                        rm_pt = Some(p_idx);
                                    }
                                });
                            }
                            if let Some(r) = rm_pt {
                                points.remove(r);
                                changed = true;
                            }
                            if ui.small_button("➕ Add Point").clicked() {
                                let last_t = points.last().map(|p| p.temperature).unwrap_or(50.0);
                                let last_c = points.last().map(|p| p.command).unwrap_or(50.0);
                                points.push(CurvePoint {
                                    temperature: (last_t + 10.0).min(100.0),
                                    command: (last_c + 15.0).min(100.0),
                                });
                                changed = true;
                            }
                        });

                        changed |=
                            Self::render_hysteresis_editor(ui, &curve.id, hysteresis);
                    }

                    FanCurveKind::Auto {
                        sensor_id,
                        idle_temp,
                        load_temp,
                        deadband,
                        min_fan_speed,
                        max_fan_speed,
                        step,
                        selected_response_time,
                    } => {
                        changed |= Self::render_sensor_selector(
                            ui,
                            &format!("auto_sens_{}", curve.id),
                            sensor_id,
                            &all_sensors,
                        );

                        if let Some(t) = &tele {
                            ui.horizontal_wrapped(|ui| {
                                let mode_str = if t.auto_under_load {
                                    RichText::new("● Load Regime").color(CACHY_AMBER).small()
                                } else {
                                    RichText::new("● Idle/Interp Regime")
                                        .color(CACHY_TEAL)
                                        .small()
                                };
                                ui.label(mode_str);
                                if let Some(st) = t.auto_short_trend {
                                    ui.label(
                                        RichText::new(format!("Short(3R): {:?}", st))
                                            .small()
                                            .weak(),
                                    );
                                }
                                if let Some(lt) = t.auto_long_trend {
                                    ui.label(
                                        RichText::new(format!("Long(5R): {:?}", lt))
                                            .small()
                                            .weak(),
                                    );
                                }
                            });
                        }

                        ui.horizontal(|ui| {
                            ui.label("Idle:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(idle_temp)
                                        .range(15.0..=*load_temp - 2.0)
                                        .suffix(" °C"),
                                )
                                .changed();
                            ui.label("Load:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(load_temp)
                                        .range(*idle_temp + 2.0..=105.0)
                                        .suffix(" °C"),
                                )
                                .changed();
                            ui.label("Deadband:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(deadband)
                                        .range(0.0..=15.0)
                                        .speed(0.5)
                                        .suffix(" °C"),
                                )
                                .changed();
                        });

                        ui.horizontal(|ui| {
                            ui.label("Min %:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(min_fan_speed)
                                        .range(0.0..=*max_fan_speed)
                                        .suffix("%"),
                                )
                                .changed();
                            ui.label("Max %:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(max_fan_speed)
                                        .range(*min_fan_speed..=100.0)
                                        .suffix("%"),
                                )
                                .changed();
                        });

                        ui.horizontal(|ui| {
                            ui.label("Step Rate:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(step)
                                        .range(0.5..=25.0)
                                        .speed(0.5)
                                        .suffix("%/tick"),
                                )
                                .changed();
                            ui.label("Response R:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(selected_response_time)
                                        .range(1..=30)
                                        .suffix(" s"),
                                )
                                .changed();
                        });
                    }

                    FanCurveKind::Mix {
                        curve_ids,
                        function,
                    } => {
                        egui::ComboBox::from_id_salt(format!("mix_fn_{}", curve.id))
                            .selected_text(format!("Function: {}", function.label()))
                            .show_ui(ui, |ui| {
                                for f in MixFunction::ALL {
                                    if ui.selectable_value(function, f, f.label()).changed() {
                                        changed = true;
                                    }
                                }
                            });

                        let winning_id = tele
                            .as_ref()
                            .and_then(|t| t.active_sub_curve_id.as_deref());

                        ui.label(RichText::new("Member Fan Curves:").small().weak());
                        for (cid, cname) in &all_curves_list {
                            if cid == &curve.id {
                                continue;
                            }
                            let mut included = curve_ids.contains(cid);
                            ui.horizontal(|ui| {
                                if ui.checkbox(&mut included, cname).changed() {
                                    if included {
                                        curve_ids.push(cid.clone());
                                    } else {
                                        curve_ids.retain(|x| x != cid);
                                    }
                                    changed = true;
                                }
                                if winning_id == Some(cid.as_str()) {
                                    ui.label(
                                        RichText::new("★ Active")
                                            .color(CACHY_TEAL)
                                            .small()
                                            .strong(),
                                    );
                                }
                            });
                        }
                    }

                    FanCurveKind::Trigger {
                        sensor_id,
                        idle_temp,
                        load_temp,
                        idle_fan_speed,
                        load_fan_speed,
                        response_time_up,
                        response_time_down,
                    } => {
                        changed |= Self::render_sensor_selector(
                            ui,
                            &format!("trig_sens_{}", curve.id),
                            sensor_id,
                            &all_sensors,
                        );
                        ui.horizontal(|ui| {
                            ui.label("Idle Temp:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(idle_temp)
                                        .range(0.0..=*load_temp - 1.0)
                                        .suffix(" °C"),
                                )
                                .changed();
                            ui.label("Idle Speed:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(idle_fan_speed)
                                        .range(0.0..=100.0)
                                        .suffix(" %"),
                                )
                                .changed();
                        });
                        ui.horizontal(|ui| {
                            ui.label("Load Temp:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(load_temp)
                                        .range(*idle_temp + 1.0..=110.0)
                                        .suffix(" °C"),
                                )
                                .changed();
                            ui.label("Load Speed:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(load_fan_speed)
                                        .range(0.0..=100.0)
                                        .suffix(" %"),
                                )
                                .changed();
                        });
                        ui.horizontal(|ui| {
                            ui.label("Resp Up:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(response_time_up)
                                        .range(1..=60)
                                        .suffix(" s"),
                                )
                                .changed();
                            ui.label("Resp Down:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(response_time_down)
                                        .range(1..=60)
                                        .suffix(" s"),
                                )
                                .changed();
                        });
                    }

                    FanCurveKind::Flat { command } => {
                        if ui
                            .add(
                                egui::Slider::new(command, 0.0..=100.0)
                                    .suffix(" %")
                                    .text("Flat Output"),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    }

                    FanCurveKind::Sync {
                        control_id,
                        offset,
                        proportional,
                    } => {
                        let cur_ctrl_name = all_controls_list
                            .iter()
                            .find(|(id, _)| id == control_id)
                            .map(|(_, n)| n.as_str())
                            .unwrap_or("Select Control");
                        egui::ComboBox::from_id_salt(format!("sync_ctrl_{}", curve.id))
                            .selected_text(format!("Source: {}", cur_ctrl_name))
                            .show_ui(ui, |ui| {
                                for (cid, cname) in &all_controls_list {
                                    if ui
                                        .selectable_value(control_id, cid.clone(), cname)
                                        .changed()
                                    {
                                        changed = true;
                                    }
                                }
                            });
                        ui.horizontal(|ui| {
                            ui.label("Offset:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(offset)
                                        .range(-100.0..=100.0)
                                        .speed(0.5)
                                        .suffix("%"),
                                )
                                .changed();
                            changed |= ui.checkbox(proportional, "Proportional").changed();
                        });
                    }
                }
            });
            ui.add_space(8.0);
        }

        if let Some(idx) = delete_idx {
            let removed_id = self.working_config.curves[idx].id.clone();
            self.working_config.curves.remove(idx);
            for ctrl in &mut self.working_config.controls {
                if ctrl.selected_fan_curve_id.as_deref() == Some(removed_id.as_str()) {
                    ctrl.selected_fan_curve_id = None;
                }
            }
            changed = true;
        }

        if changed {
            self.push_config_changes(false);
        }
    }

    fn render_sensor_selector(
        ui: &mut egui::Ui,
        salt: &str,
        sensor_id: &mut String,
        all_sensors: &[(String, String, Option<f32>)],
    ) -> bool {
        let mut changed = false;
        let current_label = all_sensors
            .iter()
            .find(|(id, _, _)| id == sensor_id)
            .map(|(_, name, val)| {
                if let Some(v) = val {
                    format!("{} ({:.1} °C)", name, v)
                } else {
                    name.clone()
                }
            })
            .unwrap_or_else(|| sensor_id.clone());

        egui::ComboBox::from_id_salt(salt)
            .selected_text(current_label)
            .width(240.0)
            .show_ui(ui, |ui| {
                for (id, name, val) in all_sensors {
                    let text = if let Some(v) = val {
                        format!("{} ({:.1} °C)", name, v)
                    } else {
                        name.clone()
                    };
                    if ui.selectable_value(sensor_id, id.clone(), text).changed() {
                        changed = true;
                    }
                }
            });
        changed
    }

    fn render_hysteresis_editor(
        ui: &mut egui::Ui,
        curve_id: &str,
        h: &mut HysteresisConfig,
    ) -> bool {
        let mut changed = false;
        egui::CollapsingHeader::new("⏱ Hysteresis & Response Time")
            .id_salt(format!("hyst_{}", curve_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Hyst Up:");
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut h.hysteresis_value_up)
                                .range(0.0..=20.0)
                                .speed(0.5)
                                .suffix(" °C"),
                        )
                        .changed();
                    ui.label("Down:");
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut h.hysteresis_value_down)
                                .range(0.0..=20.0)
                                .speed(0.5)
                                .suffix(" °C"),
                        )
                        .changed();
                });
                ui.horizontal(|ui| {
                    ui.label("Resp Up:");
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut h.response_time_up)
                                .range(1..=60)
                                .suffix(" s"),
                        )
                        .changed();
                    ui.label("Down:");
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut h.response_time_down)
                                .range(1..=60)
                                .suffix(" s"),
                        )
                        .changed();
                });
                changed |= ui
                    .checkbox(
                        &mut h.ignore_hysteresis_at_limits,
                        "Ignore hysteresis at min/max curve limits",
                    )
                    .changed();
            });
        changed
    }

    fn render_sensors_column(&mut self, ui: &mut egui::Ui) {
        ui.heading(RichText::new("🌡 Custom & Hardware Sensors").color(CACHY_AMBER));
        ui.label(
            RichText::new("Time-averaging ring buffers, multi-sensor mixes & /sys/class/hwmon inputs")
                .small()
                .weak(),
        );
        ui.add_space(6.0);

        let all_sources = self.all_temperature_sources();
        let mut changed = false;
        let mut delete_idx: Option<usize> = None;

        for idx in 0..self.working_config.custom_sensors.len() {
            let cs_id = self.working_config.custom_sensors[idx].id.clone();
            let tele = self.snapshot.custom_sensor_telemetry.get(&cs_id).cloned();

            Self::card_frame().show(ui, |ui| {
                let cs = &mut self.working_config.custom_sensors[idx];
                let badge = match &cs.kind {
                    CustomSensorKind::TimeAverage { .. } => "TIME AVG",
                    CustomSensorKind::Mix { .. } => "MIX",
                    CustomSensorKind::Offset { .. } => "OFFSET",
                    CustomSensorKind::File { .. } => "FILE",
                };

                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(format!("[{}]", badge))
                            .color(CACHY_AMBER)
                            .strong()
                            .small(),
                    );
                    if ui
                        .add(egui::TextEdit::singleline(&mut cs.name).desired_width(155.0))
                        .changed()
                    {
                        changed = true;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.small_button("🗑").clicked() {
                            delete_idx = Some(idx);
                        }
                        if let Some(val) = tele.as_ref().and_then(|t| t.value) {
                            ui.label(
                                RichText::new(format!("{:.1} °C", val))
                                    .color(CACHY_TEAL)
                                    .strong(),
                            );
                        } else {
                            ui.label(RichText::new("N/A").weak());
                        }
                    });
                });

                match &mut cs.kind {
                    CustomSensorKind::TimeAverage {
                        source_id,
                        selected_time,
                    } => {
                        let filtered: Vec<_> = all_sources
                            .iter()
                            .filter(|(id, _, _)| id != &cs.id)
                            .cloned()
                            .collect();
                        changed |= Self::render_sensor_selector(
                            ui,
                            &format!("cs_avg_{}", cs.id),
                            source_id,
                            &filtered,
                        );
                        ui.horizontal(|ui| {
                            ui.label("Window Time:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(selected_time)
                                        .range(1..=300)
                                        .suffix(" s"),
                                )
                                .changed();
                        });
                    }

                    CustomSensorKind::Mix {
                        source_ids,
                        function,
                        allow_missing_sensor,
                    } => {
                        egui::ComboBox::from_id_salt(format!("cs_mix_fn_{}", cs.id))
                            .selected_text(format!("Function: {}", function.label()))
                            .show_ui(ui, |ui| {
                                for f in SensorMixFunction::ALL {
                                    if ui.selectable_value(function, f, f.label()).changed() {
                                        changed = true;
                                    }
                                }
                            });
                        changed |= ui
                            .checkbox(allow_missing_sensor, "Allow missing sensor values")
                            .changed();

                        let active_src = tele
                            .as_ref()
                            .and_then(|t| t.active_source_id.as_deref());

                        egui::CollapsingHeader::new(format!(
                            "Source Sensors ({})",
                            source_ids.len()
                        ))
                        .id_salt(format!("cs_mix_srcs_{}", cs.id))
                        .default_open(true)
                        .show(ui, |ui| {
                            for (sid, sname, sval) in &all_sources {
                                if sid == &cs.id {
                                    continue;
                                }
                                let mut inc = source_ids.contains(sid);
                                let label = if let Some(v) = sval {
                                    format!("{} ({:.1} °C)", sname, v)
                                } else {
                                    sname.clone()
                                };
                                ui.horizontal(|ui| {
                                    if ui.checkbox(&mut inc, label).changed() {
                                        if inc {
                                            source_ids.push(sid.clone());
                                        } else {
                                            source_ids.retain(|x| x != sid);
                                        }
                                        changed = true;
                                    }
                                    if active_src == Some(sid.as_str()) {
                                        ui.label(
                                            RichText::new("★ Active")
                                                .color(CACHY_TEAL)
                                                .small(),
                                        );
                                    }
                                });
                            }
                        });
                    }

                    CustomSensorKind::Offset {
                        source_id,
                        offset,
                        proportional,
                    } => {
                        let filtered: Vec<_> = all_sources
                            .iter()
                            .filter(|(id, _, _)| id != &cs.id)
                            .cloned()
                            .collect();
                        changed |= Self::render_sensor_selector(
                            ui,
                            &format!("cs_off_{}", cs.id),
                            source_id,
                            &filtered,
                        );
                        ui.horizontal(|ui| {
                            ui.label("Offset:");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(offset)
                                        .range(-50.0..=50.0)
                                        .speed(0.5),
                                )
                                .changed();
                            changed |= ui.checkbox(proportional, "Proportional %").changed();
                        });
                    }

                    CustomSensorKind::File { path } => {
                        ui.horizontal(|ui| {
                            ui.label("Path:");
                            if ui.text_edit_singleline(path).changed() {
                                changed = true;
                            }
                        });
                    }
                }
            });
            ui.add_space(8.0);
        }

        if let Some(idx) = delete_idx {
            self.working_config.custom_sensors.remove(idx);
            changed = true;
        }
        if changed {
            self.push_config_changes(false);
        }

        // Hardware `/sys/class/hwmon` Sensor Card
        Self::card_frame().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("🐧 Linux /sys/class/hwmon Telemetry")
                        .color(CACHY_TEAL)
                        .strong(),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        RichText::new(format!(
                            "{} Temps | {} Fans",
                            self.snapshot.hw_temps.len(),
                            self.snapshot.hw_fans.len()
                        ))
                        .small()
                        .weak(),
                    );
                });
            });
            ui.separator();

            for f in &self.snapshot.hw_fans {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("🌀").color(CACHY_BLUE));
                    ui.label(&f.name);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let txt = f
                            .rpm
                            .map(|r| format!("{:.0} RPM", r))
                            .unwrap_or_else(|| "N/A".to_string());
                        ui.label(RichText::new(txt).color(CACHY_BLUE).strong());
                    });
                });
            }

            if !self.snapshot.hw_fans.is_empty() {
                ui.separator();
            }

            for t in &self.snapshot.hw_temps {
                ui.horizontal(|ui| {
                    ui.label(&t.name);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if let Some(v) = t.value {
                            let color = if v >= 80.0 {
                                CACHY_RED
                            } else if v >= 65.0 {
                                CACHY_AMBER
                            } else {
                                CACHY_TEAL
                            };
                            ui.label(RichText::new(format!("{:.1} °C", v)).color(color).strong());
                        } else {
                            ui.label(RichText::new("N/A").weak());
                        }
                    });
                });
            }
        });
    }
}

impl eframe::App for CachyFanControlApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.last_poll.elapsed() >= Duration::from_millis(1000) {
            self.poll_backend(false);
        }
        ctx.request_repaint_after(Duration::from_millis(500));

        egui::TopBottomPanel::top("top_bar")
            .frame(
                Frame::new()
                    .fill(Color32::from_rgb(22, 25, 32))
                    .inner_margin(Margin::symmetric(14, 10)),
            )
            .show(ctx, |ui| {
                self.render_top_bar(ui);
            });

        egui::TopBottomPanel::bottom("status_bar")
            .frame(
                Frame::new()
                    .fill(Color32::from_rgb(20, 22, 28))
                    .inner_margin(Margin::symmetric(14, 6)),
            )
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(&self.status_banner).small().color(CACHY_TEAL));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(
                            RichText::new(format!(
                                "1 Hz Engine Tick #{} | Wayland Native",
                                self.snapshot.tick_count
                            ))
                            .small()
                            .weak(),
                        );
                    });
                });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            let avail_w = ui.available_width();
            let col_w = ((avail_w - 24.0) / 3.0).max(280.0);

            ui.columns(3, |cols| {
                cols[0].set_min_width(col_w * 0.9);
                ScrollArea::vertical()
                    .id_salt("scroll_controls")
                    .auto_shrink([false, false])
                    .show(&mut cols[0], |ui| {
                        self.render_controls_column(ui);
                    });

                cols[1].set_min_width(col_w * 0.9);
                ScrollArea::vertical()
                    .id_salt("scroll_curves")
                    .auto_shrink([false, false])
                    .show(&mut cols[1], |ui| {
                        self.render_curves_column(ui);
                    });

                cols[2].set_min_width(col_w * 0.9);
                ScrollArea::vertical()
                    .id_salt("scroll_sensors")
                    .auto_shrink([false, false])
                    .show(&mut cols[2], |ui| {
                        self.render_sensors_column(ui);
                    });
            });
        });
    }
}

fn main() -> eframe::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("CachyFanControl — Native Linux Fan Control")
            .with_app_id("org.hearnelabs.cachyfancontrol")
            .with_inner_size(Vec2::new(1360.0, 860.0))
            .with_min_inner_size(Vec2::new(960.0, 600.0)),
        ..Default::default()
    };

    eframe::run_native(
        "CachyFanControl",
        native_options,
        Box::new(|cc| Ok(Box::new(CachyFanControlApp::new(cc)))),
    )
}
