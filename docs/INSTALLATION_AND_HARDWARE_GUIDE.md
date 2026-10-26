# CachyFanControl — Installation & Hardware Configuration Guide

This guide covers installing **CachyFanControl** on **CachyOS** and **Arch Linux**, configuring the `cachyfancontrold` systemd daemon, and enabling kernel modules/parameters for desktop motherboards, GPUs, and laptops.

---

## 1. Installing on CachyOS / Arch Linux (`makepkg`)

### Prerequisites
Ensure base build tools, `lm_sensors`, and Wayland runtime libraries are installed:
```bash
sudo pacman -S --needed base-devel rust lm_sensors wayland libxkbcommon libglvnd
```

### Build and Install the Package
Clone the repository and build the native Arch package using `makepkg`:
```bash
git clone https://github.com/brisbinchicken/CachyFanControl.git
cd CachyFanControl
makepkg -si
```

### Post-Installation Setup
1. **Add your user to the `cachyfan` group** so the unprivileged Wayland GUI (`cachyfancontrol-gui`) can communicate with `cachyfancontrold` over `/run/cachyfancontrol/daemon.sock` without root password prompts:
   ```bash
   sudo usermod -aG cachyfan $USER
   ```
   *(Log out and back in—or run `newgrp cachyfan`—for group membership to take effect.)*

2. **Enable and start the hardware fan control daemon**:
   ```bash
   sudo systemctl enable --now cachyfancontrold.service
   ```

3. **Launch the Wayland GUI**:
   ```bash
   cachyfancontrol-gui
   ```
   *(Or launch **CachyFanControl** directly from your application launcher in KDE Plasma, Hyprland, Niri, or GNOME.)*

---

## 2. Detecting Motherboard Super I/O Sensors (`lm_sensors`)

On desktop motherboards, CPU/GPU temperatures are loaded automatically (`coretemp`, `k10temp`, `amdgpu`), while motherboard fan headers (`CPU_FAN`, `CHA_FAN1..4`, `PUMP_FAN`) are typically managed by a Super I/O chip (Nuvoton `nct6775`/`nct6687` or ITE `it87`).

Run `sensors-detect` once to identify and autoload your board's Super I/O kernel modules:
```bash
sudo sensors-detect --auto
sudo systemctl restart lm_sensors.service
sudo systemctl restart cachyfancontrold.service
```

---

## 3. Kernel Boot Parameters & Driver Notes

### A. Nuvoton (`nct6775` / `nct6687d`) & ITE (`it87`) Motherboards
If your motherboard fan headers do not appear under `/sys/class/hwmon/` and `sudo dmesg | grep -iE 'nct|it87|acpi'` shows:
```text
ACPI Warning: SystemIO range ... conflicts with OpRegion ...
Resource conflict(s) found affecting hwmon
```
Add `acpi_enforce_resources=lax` to your kernel command line:
- **systemd-boot** (`/boot/loader/entries/*.conf`): append `acpi_enforce_resources=lax` to the `options` line.
- **Limine** (`/boot/limine.conf`): append `acpi_enforce_resources=lax` to `KERNEL_CMDLINE`.
- **GRUB** (`/etc/default/grub`): append `acpi_enforce_resources=lax` to `GRUB_CMDLINE_LINUX_DEFAULT` and run `sudo grub-mkconfig -o /boot/grub/grub.cfg`.

> **ASUS / MSI / Gigabyte B550, X670, Z690, Z790 Boards with Nuvoton NCT6687-R**:
> Install the DKMS driver from the CachyOS/AUR repository:
> ```bash
> paru -S nct6687d-dkms-git
> ```

### B. AMD Radeon GPUs (`amdgpu`)
Modern AMD GPUs expose `temp1_input` (Edge), `temp2_input` (Junction/Hotspot), `fan1_input`, and `pwm1` directly under `/sys/class/hwmon/`. If manual `pwm1` writes are locked by firmware on specific RDNA cards, add:
```text
amdgpu.ppfeaturemask=0xffffffff
```
to your kernel command line and reboot.

### C. Laptops (HP, Lenovo ThinkPad, ASUS ROG/TUF)
`CachyFanControl` supports three distinct Linux fan control interfaces out-of-the-box:
1. **Standard `hwmon` PWM** (`/sys/class/hwmon/hwmonX/pwmY` `0..=255` + `pwmY_enable`).
2. **WMI `pwm1_enable`-only controllers** (such as `hp-wmi` on HP OMEN/Victus/EliteBook laptops, where `pwm1_enable = 0` engages full-blast cooling and `pwm1_enable = 2` restores BIOS automatic control).
3. **ACPI Thermal `Fan` Cooling Devices** (`/sys/class/thermal/cooling_device*` where `type == "Fan"`, mapping `0..=100%` to `0..=max_state`).

For **Lenovo ThinkPads**, enable manual fan control in the `thinkpad_acpi` module:
```bash
echo "options thinkpad_acpi fan_control=1" | sudo tee /etc/modprobe.d/thinkpad_acpi.conf
```

---

## 4. Emergency Hardware Fallback & Troubleshooting

### Returning All Fans to BIOS Control Immediately
At any time, you can click **"🛡 Relinquish All to BIOS"** in the top bar of `cachyfancontrol-gui`, or run:
```bash
sudo cachyfancontrold --restore-bios
```
This immediately writes `2` (Automatic BIOS/EC control) to every `/sys/class/hwmon/hwmon*/pwm*_enable` file on the system.

### Inspecting Live Hardware JSON Telemetry from the CLI
To print a single JSON snapshot of all discovered sensors, curves, and controls without starting the background daemon:
```bash
cachyfancontrold --once
```
