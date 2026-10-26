# Contributing to CachyFanControl

Thank you for your interest in contributing to **CachyFanControl**!

## Repository Layout

```text
CachyFanControl/
├── Cargo.toml                        # Rust workspace manifest
├── PKGBUILD                          # Arch Linux / CachyOS package build script
├── cachyfancontrol.install           # Pacman post_install / pre_remove hooks
├── assets/                           # Application icons (SVG & PNG)
├── docs/                             # Detailed hardware & mathematical guides
├── packaging/                        # systemd unit, sysusers, tmpfiles, udev, .desktop
└── crates/
    ├── cachyfancontrol-core/         # Core 1 Hz engine, curves, OLS PID, calibration, hwmon HAL
    ├── cachyfancontrold/             # Privileged systemd daemon with Unix socket IPC & BIOS fallback
    └── cachyfancontrol-gui/          # Native Wayland card-based GUI (eframe/egui + egui_plot)
```

## Development Workflow

1. **Run the Unit Test Suite**:
   ```bash
   cargo test --workspace
   ```

2. **Test Hardware Discovery & 1 Hz Engine Snapshot (Dry-Run / Single Tick)**:
   ```bash
   cargo run -p cachyfancontrold -- --once
   ```

3. **Run the Wayland GUI in Development Mode**:
   ```bash
   cargo run -p cachyfancontrol-gui
   ```

4. **Test Packaging with `makepkg`**:
   ```bash
   makepkg -f
   ```

## Hardware Safety Guidelines for Contributors
- **Never block the 1 Hz engine loop**: All `/sys/class/hwmon` and `/sys/class/thermal` file operations must use `O_NONBLOCK` (`read_sysfs_trimmed` / `write_sysfs_str`).
- **Always preserve BIOS fallback**: Any new hardware control backend added to `hwmon.rs` must record its initial firmware state before modification and restore it inside `HwmonManager::restore_saved_state`.
