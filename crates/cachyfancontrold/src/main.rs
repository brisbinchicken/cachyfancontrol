//! `cachyfancontrold` — Privileged background daemon for CachyFanControl.
//!
//! - Runs the reverse-engineered 1 Hz `FanControlEngine` tick loop.
//! - Exposes a non-blocking Unix Domain Socket JSON-RPC server for `cachyfancontrol-gui`.
//! - Guarantees hardware safety by restoring all `/sys/class/hwmon/hwmon*/pwm*_enable`
//!   channels back to automatic BIOS/firmware control (`2`) on `SIGINT`, `SIGTERM`,
//!   `SIGQUIT`, panic, or via `--restore-bios` (`systemd` `ExecStopPost`).

use cachyfancontrol_core::engine::{
    AppConfig, DaemonRequest, DaemonResponse, FanControlEngine, DEFAULT_CONFIG_PATH,
    DEFAULT_SOCKET_PATH,
};
use cachyfancontrol_core::hwmon::HwmonManager;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

static SHUTDOWN_REQUESTED: AtomicBool = AtomicBool::new(false);

extern "C" fn handle_termination_signal(_sig: libc::c_int) {
    SHUTDOWN_REQUESTED.store(true, Ordering::SeqCst);
}

fn install_signal_handlers() {
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = handle_termination_signal as *const () as usize;
        libc::sigemptyset(&mut sa.sa_mask);
        sa.sa_flags = 0;

        libc::sigaction(libc::SIGINT, &sa, std::ptr::null_mut());
        libc::sigaction(libc::SIGTERM, &sa, std::ptr::null_mut());
        libc::sigaction(libc::SIGQUIT, &sa, std::ptr::null_mut());
        libc::sigaction(libc::SIGHUP, &sa, std::ptr::null_mut());
    }
}

/// Emergency fail-safe (`cachyfancontrold --restore-bios`) invoked by systemd `ExecStopPost`
/// in case the daemon was ever killed with `SIGKILL`.
fn emergency_restore_all_hwmon_to_bios() {
    let hwmon_root = Path::new("/sys/class/hwmon");
    let Ok(entries) = fs::read_dir(hwmon_root) else {
        return;
    };
    for entry in entries.flatten() {
        let dir = entry.path();
        let Ok(files) = fs::read_dir(&dir) else {
            continue;
        };
        for f in files.flatten() {
            let fname = f.file_name().to_string_lossy().into_owned();
            if fname.starts_with("pwm") && fname.ends_with("_enable") {
                let p = f.path();
                if let Ok(mut file) = OpenOptions::new()
                    .write(true)
                    .custom_flags(libc::O_NONBLOCK)
                    .open(&p)
                {
                    let _ = file.write_all(b"2\n");
                }
            }
        }
    }
}

/// Acquires the engine mutex with a 200 ms timeout (`Monitor.TryEnter(_lock, 200)`).
fn try_lock_timeout<T>(mutex: &Mutex<T>, timeout: Duration) -> Option<std::sync::MutexGuard<'_, T>> {
    let start = Instant::now();
    loop {
        if let Ok(guard) = mutex.try_lock() {
            return Some(guard);
        }
        if start.elapsed() >= timeout {
            return None;
        }
        thread::sleep(Duration::from_millis(5));
    }
}

fn handle_client(
    mut stream: UnixStream,
    engine: Arc<Mutex<FanControlEngine>>,
    config_path: PathBuf,
) {
    let _ = stream.set_read_timeout(Some(Duration::from_secs(3)));
    let _ = stream.set_write_timeout(Some(Duration::from_secs(3)));

    let Ok(reader_stream) = stream.try_clone() else {
        return;
    };
    let mut reader = BufReader::new(reader_stream);
    let mut line = String::new();

    while let Ok(bytes) = reader.read_line(&mut line) {
        if bytes == 0 {
            break;
        }
        let trimmed = line.trim();
        if trimmed.is_empty() {
            line.clear();
            continue;
        }

        let resp = match serde_json::from_str::<DaemonRequest>(trimmed) {
            Ok(req) => {
                if let Some(mut eng) = try_lock_timeout(&engine, Duration::from_millis(400)) {
                    match req {
                        DaemonRequest::GetSnapshot => DaemonResponse::Ok {
                            snapshot: Box::new(eng.snapshot()),
                            message: None,
                        },
                        DaemonRequest::UpdateConfig {
                            config,
                            save_to_disk,
                        } => {
                            eng.apply_config(config);
                            let mut msg = Some("Configuration applied.".to_string());
                            if save_to_disk {
                                if let Err(e) = eng.config.save_atomic(&config_path) {
                                    msg = Some(format!(
                                        "Applied in memory, but failed saving {}: {}",
                                        config_path.display(),
                                        e
                                    ));
                                } else {
                                    msg = Some(format!(
                                        "Configuration applied & saved to {}",
                                        config_path.display()
                                    ));
                                }
                            }
                            DaemonResponse::Ok {
                                snapshot: Box::new(eng.snapshot()),
                                message: msg,
                            }
                        }
                        DaemonRequest::RefreshHardware => {
                            eng.hwmon.discover();
                            eng.tick();
                            DaemonResponse::Ok {
                                snapshot: Box::new(eng.snapshot()),
                                message: Some("Hardware sensors re-scanned.".to_string()),
                            }
                        }
                        DaemonRequest::CalibrateControl { control_id } => {
                            match eng.calibrate_control(&control_id) {
                                Ok(info) => {
                                    let _ = eng.config.save_atomic(&config_path);
                                    DaemonResponse::Ok {
                                        snapshot: Box::new(eng.snapshot()),
                                        message: Some(info),
                                    }
                                }
                                Err(e) => DaemonResponse::Error { message: e },
                            }
                        }
                        DaemonRequest::RelinquishAllToBios => {
                            for ctrl in &mut eng.controls {
                                ctrl.config.enabled = false;
                            }
                            for ctrl in &mut eng.config.controls {
                                ctrl.enabled = false;
                            }
                            eng.hwmon.restore_all_to_bios();
                            eng.tick();
                            DaemonResponse::Ok {
                                snapshot: Box::new(eng.snapshot()),
                                message: Some(
                                    "All fans relinquished back to BIOS/firmware automatic mode."
                                        .to_string(),
                                ),
                            }
                        }
                    }
                } else {
                    DaemonResponse::Error {
                        message: "Engine lock busy; retry on next tick.".to_string(),
                    }
                }
            }
            Err(e) => DaemonResponse::Error {
                message: format!("Invalid IPC request JSON: {}", e),
            },
        };

        if let Ok(mut encoded) = serde_json::to_string(&resp) {
            encoded.push('\n');
            if stream.write_all(encoded.as_bytes()).is_err() {
                break;
            }
            let _ = stream.flush();
        }
        line.clear();
    }
}

fn resolve_default_paths() -> (PathBuf, PathBuf) {
    let is_root = unsafe { libc::geteuid() == 0 };
    if is_root {
        (
            PathBuf::from(DEFAULT_SOCKET_PATH),
            PathBuf::from(DEFAULT_CONFIG_PATH),
        )
    } else {
        let runtime_dir = env::var("XDG_RUNTIME_DIR").unwrap_or_else(|_| "/tmp".to_string());
        let config_dir = env::var("XDG_CONFIG_HOME").unwrap_or_else(|_| {
            let home = env::var("HOME").unwrap_or_else(|_| "/tmp".to_string());
            format!("{}/.config", home)
        });
        (
            PathBuf::from(format!("{}/cachyfancontrol.sock", runtime_dir)),
            PathBuf::from(format!("{}/cachyfancontrol/config.json", config_dir)),
        )
    }
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args: Vec<String> = env::args().collect();
    if args.iter().any(|a| a == "--restore-bios") {
        log::info!("Executing emergency BIOS PWM fallback (--restore-bios)...");
        emergency_restore_all_hwmon_to_bios();
        return;
    }

    let run_once = args.iter().any(|a| a == "--once");
    let (mut socket_path, mut config_path) = resolve_default_paths();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--socket" if i + 1 < args.len() => {
                socket_path = PathBuf::from(&args[i + 1]);
                i += 1;
            }
            "--config" if i + 1 < args.len() => {
                config_path = PathBuf::from(&args[i + 1]);
                i += 1;
            }
            _ => {}
        }
        i += 1;
    }

    install_signal_handlers();

    let hwmon = HwmonManager::new();
    log::info!(
        "Discovered {} temperature sensors, {} fan tachometers, and {} PWM/fan controls.",
        hwmon.temp_sensors.len(),
        hwmon.fan_sensors.len(),
        hwmon.pwm_controls.len()
    );

    let config = AppConfig::load_or_create(&config_path, &hwmon);
    let engine = Arc::new(Mutex::new(FanControlEngine::new(hwmon, config)));

    // Install panic hook to guarantee BIOS fan restoration if a panic ever occurs
    {
        let engine_for_panic = Arc::clone(&engine);
        let default_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            log::error!("Daemon panic detected! Restoring all fans to BIOS automatic control...");
            if let Ok(mut guard) = engine_for_panic.try_lock() {
                guard.hwmon.restore_all_to_bios();
            } else {
                emergency_restore_all_hwmon_to_bios();
            }
            default_hook(info);
        }));
    }

    if run_once {
        if let Ok(mut guard) = engine.lock() {
            guard.tick();
            let snap = guard.snapshot();
            println!(
                "{}",
                serde_json::to_string_pretty(&snap).unwrap_or_default()
            );
            guard.hwmon.restore_all_to_bios();
        }
        return;
    }

    if let Some(parent) = socket_path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    let _ = fs::remove_file(&socket_path);

    let listener = match UnixListener::bind(&socket_path) {
        Ok(l) => l,
        Err(e) => {
            log::error!("Failed to bind Unix socket {}: {}", socket_path.display(), e);
            return;
        }
    };
    let _ = listener.set_nonblocking(true);
    // Allow GUI user process to connect to the socket
    let _ = fs::set_permissions(&socket_path, fs::Permissions::from_mode(0o666));

    log::info!(
        "cachyfancontrold listening on {} (config: {})",
        socket_path.display(),
        config_path.display()
    );

    // Spawn 1 Hz ApplicationClock tick thread (`FanControlEngine.Update`)
    let engine_for_clock = Arc::clone(&engine);
    let clock_handle = thread::spawn(move || {
        while !SHUTDOWN_REQUESTED.load(Ordering::SeqCst) {
            let tick_start = Instant::now();
            let mut interval_ms = 1000_u64;

            // Match FanControlEngine.Update() 200ms Monitor.TryEnter timeout
            if let Some(mut eng) = try_lock_timeout(&engine_for_clock, Duration::from_millis(200)) {
                eng.tick();
                interval_ms = eng.config.tick_interval_ms.clamp(250, 5000);
            }

            let elapsed = tick_start.elapsed();
            let target_dur = Duration::from_millis(interval_ms);
            if elapsed < target_dur {
                let remaining = target_dur - elapsed;
                let step = Duration::from_millis(50);
                let mut slept = Duration::ZERO;
                while slept < remaining && !SHUTDOWN_REQUESTED.load(Ordering::SeqCst) {
                    let s = step.min(remaining - slept);
                    thread::sleep(s);
                    slept += s;
                }
            }
        }
    });

    // Accept IPC connections until shutdown signal
    while !SHUTDOWN_REQUESTED.load(Ordering::SeqCst) {
        match listener.accept() {
            Ok((stream, _addr)) => {
                let eng_clone = Arc::clone(&engine);
                let cfg_clone = config_path.clone();
                thread::spawn(move || handle_client(stream, eng_clone, cfg_clone));
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(40));
            }
            Err(e) => {
                log::warn!("Socket accept error: {}", e);
                thread::sleep(Duration::from_millis(100));
            }
        }
    }

    log::info!("Shutting down cachyfancontrold and restoring fans to BIOS firmware control...");
    let _ = clock_handle.join();
    if let Ok(mut guard) = engine.lock() {
        guard.hwmon.restore_all_to_bios();
    } else {
        emergency_restore_all_hwmon_to_bios();
    }
    let _ = fs::remove_file(&socket_path);
}
