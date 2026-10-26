//! Core mathematical utilities reverse-engineered from `FanControl.Domain.Utilities`
//! in `FanControl.Library.dll` (V284).
//!
//! Includes:
//! - [`ResponseTime`] (token `0x02000084`)
//! - [`Hysteresis`] & [`HysteresisConfig`] (token `0x02000080`)
//! - [`Trend`] & [`LinearRegressionTrend`] (token `0x02000082`)

use serde::{Deserialize, Serialize};

/// Discrete tick counter reverse-engineered from `FanControl.Domain.Utilities.ResponseTime` (`0x02000084`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponseTime {
    value: i32,
    #[serde(skip)]
    count: i32,
    automatic_reset: bool,
}

impl ResponseTime {
    pub fn new(automatic_reset: bool) -> Self {
        Self {
            value: 1,
            count: 0,
            automatic_reset,
        }
    }

    pub fn with_value(automatic_reset: bool, value: i32) -> Self {
        Self {
            value: value.max(1),
            count: 0,
            automatic_reset,
        }
    }

    pub fn value(&self) -> i32 {
        self.value
    }

    pub fn set_value(&mut self, val: i32) {
        self.value = val;
    }

    pub fn count(&self) -> i32 {
        self.count
    }

    /// Resets the internal tick counter (`0x060003bc`).
    pub fn reset(&mut self) {
        self.count = 0;
    }

    /// Increments the tick counter by 1 and checks if `count >= value` (`0x060003bd`).
    /// If `automatic_reset && triggered`, resets `count` to `0`.
    pub fn trigger(&mut self) -> bool {
        self.count += 1;
        let triggered = self.count >= self.value;
        if self.automatic_reset && triggered {
            self.reset();
        }
        triggered
    }
}

/// Configuration for [`Hysteresis`] matching `FanControl.Serializable.Interfaces.IHysteresisConfig`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HysteresisConfig {
    pub hysteresis_value_up: f32,
    pub hysteresis_value_down: f32,
    pub response_time_up: i32,
    pub response_time_down: i32,
    pub ignore_hysteresis_at_limits: bool,
}

impl Default for HysteresisConfig {
    fn default() -> Self {
        Self {
            hysteresis_value_up: 2.0,
            hysteresis_value_down: 2.0,
            response_time_up: 1,
            response_time_down: 1,
            ignore_hysteresis_at_limits: true,
        }
    }
}

/// Sentinel value `-float.MaxValue` (`-3.4028234663852886e+38`) used in `Hysteresis.Reset()` (`0x060003ac`).
const RESET_SENTINEL: f64 = -(f32::MAX as f64);

/// Asymmetric temperature hysteresis + response-time filter reverse-engineered from
/// `FanControl.Domain.Utilities.Hysteresis` (`0x02000080`).
#[derive(Debug, Clone)]
pub struct Hysteresis {
    up: ResponseTime,
    down: ResponseTime,
    previous_value: f64,
    config: HysteresisConfig,
}

impl Hysteresis {
    pub fn new(config: HysteresisConfig) -> Self {
        let mut h = Self {
            up: ResponseTime::new(true),
            down: ResponseTime::new(true),
            previous_value: f64::MIN,
            config,
        };
        h.on_update_config(config);
        h
    }

    pub fn config(&self) -> &HysteresisConfig {
        &self.config
    }

    pub fn previous_value(&self) -> Option<f32> {
        if self.previous_value <= RESET_SENTINEL {
            None
        } else {
            Some(self.previous_value as f32)
        }
    }

    /// Updates hysteresis thresholds and response times (`0x060003ae`).
    pub fn on_update_config(&mut self, config: HysteresisConfig) {
        self.config = config;
        self.up.set_value(config.response_time_up);
        self.down.set_value(config.response_time_down);
        self.reset();
    }

    /// Resets both response timers (`0x060003ad`).
    pub fn reset_response_time(&mut self) {
        self.up.reset();
        self.down.reset();
    }

    /// Resets the latched temperature and response timers (`0x060003ac`).
    pub fn reset(&mut self) {
        self.previous_value = RESET_SENTINEL;
        self.reset_response_time();
    }

    /// Evaluates whether a temperature change should pass through the hysteresis & response-time filter (`0x060003ab`).
    pub fn trigger(&mut self, current_temp: f32, is_at_limit: bool) -> bool {
        // First sample after initialization or Reset()
        if self.previous_value <= RESET_SENTINEL {
            self.previous_value = current_temp as f64;
            return true;
        }

        let mut up_flag = false;
        let mut down_flag = false;
        let cur = current_temp as f64;

        if is_at_limit && self.config.ignore_hysteresis_at_limits {
            up_flag = self.up.trigger();
            down_flag = self.down.trigger();
        } else if cur - self.previous_value >= self.config.hysteresis_value_up as f64 {
            up_flag = self.up.trigger();
            self.down.reset();
        } else if self.previous_value - cur >= self.config.hysteresis_value_down as f64 {
            down_flag = self.down.trigger();
            self.up.reset();
        } else {
            self.reset_response_time();
        }

        if up_flag || down_flag {
            self.previous_value = cur;
            true
        } else {
            false
        }
    }
}

/// Discrete thermal trend state reverse-engineered from `FanControl.Domain.Utilities.LinearRegressionTrend+Trend` (`0x02000105`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i32)]
pub enum Trend {
    Unknown = -999,
    Down = -1,
    Neutral = 0,
    Up = 1,
}

/// Ordinary Least Squares (OLS) linear regression slope detector + integral creep accumulator
/// reverse-engineered from `FanControl.Domain.Utilities.LinearRegressionTrend` (`0x02000082`).
#[derive(Debug, Clone)]
pub struct LinearRegressionTrend {
    window_size: usize,
    half_window_size: f32,
    tolerance: f32,
    circular_buffer: Vec<f32>,
    creap: f32,
    i: isize,
    first_pass_completed: bool,
    sum_x: f32,
    sum_x_squared: f32,
    sum_x_all_squared: f32,
}

impl LinearRegressionTrend {
    /// Initializes the OLS window and precomputes `sumX`, `sum_XSquared`, and `sumX_Squared` (`0x060003b0`).
    pub fn new(window_size: usize, tolerance: f32) -> Self {
        let window_size = window_size.max(2);
        let half_window_size = 0.5 * (window_size as f32);
        let sum_x: i64 = (0..window_size as i64).sum();
        let sum_x_squared: i64 = (0..window_size as i64).map(|x| x * x).sum();
        let sum_x_f = sum_x as f32;

        Self {
            window_size,
            half_window_size,
            tolerance,
            circular_buffer: vec![0.0; window_size],
            creap: 0.0,
            i: -1,
            first_pass_completed: false,
            sum_x: sum_x_f,
            sum_x_squared: sum_x_squared as f32,
            sum_x_all_squared: sum_x_f * sum_x_f,
        }
    }

    pub fn reset(&mut self) {
        self.i = -1;
        self.first_pass_completed = false;
        self.creap = 0.0;
    }

    /// Pushes a new temperature sample and computes the current [`Trend`] (`0x060003b3`).
    pub fn update(&mut self, value: f32) -> Trend {
        let n = self.window_size as isize;
        self.i = (self.i + 1) % n;
        self.circular_buffer[self.i as usize] = value;

        if !self.first_pass_completed {
            if self.i == n - 1 {
                self.first_pass_completed = true;
            } else {
                return Trend::Unknown;
            }
        }

        let slope = self.calculate_linear_regression_slope();
        if (slope * self.half_window_size).abs() > self.tolerance {
            if slope >= 0.0 {
                Trend::Up
            } else {
                Trend::Down
            }
        } else {
            self.creap += slope;
            if self.creap.abs() > self.tolerance {
                let old_creap = self.creap;
                self.creap = 0.0;
                if old_creap >= 0.0 {
                    Trend::Up
                } else {
                    Trend::Down
                }
            } else {
                Trend::Neutral
            }
        }
    }

    /// Computes exact OLS slope `m` over the circular buffer from oldest (`k = 0`) to newest (`k = N - 1`) (`0x060003b4`).
    pub fn calculate_linear_regression_slope(&self) -> f32 {
        let n = self.window_size;
        let mut sum_y = 0.0_f32;
        let mut sum_xy = 0.0_f32;
        let start = (self.i as usize) + 1;

        for k in 0..n {
            let idx = (start + k) % n;
            let y = self.circular_buffer[idx];
            sum_y += y;
            sum_xy += y * (k as f32);
        }

        let n_f = n as f32;
        let denom = n_f * self.sum_x_squared - self.sum_x_all_squared;
        if denom.abs() < f32::EPSILON {
            return 0.0;
        }
        (n_f * sum_xy - self.sum_x * sum_y) / denom
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_response_time_auto_reset() {
        let mut rt = ResponseTime::with_value(true, 3);
        assert!(!rt.trigger());
        assert!(!rt.trigger());
        assert!(rt.trigger()); // 3rd tick triggers and auto-resets
        assert!(!rt.trigger());
    }

    #[test]
    fn test_hysteresis_deadband_and_limits() {
        let cfg = HysteresisConfig {
            hysteresis_value_up: 3.0,
            hysteresis_value_down: 2.0,
            response_time_up: 2,
            response_time_down: 2,
            ignore_hysteresis_at_limits: true,
        };
        let mut h = Hysteresis::new(cfg);
        // First sample always latches immediately
        assert!(h.trigger(50.0, false));
        // Inside deadband (+2.0 < 3.0)
        assert!(!h.trigger(52.0, false));
        // Excursion up (+3.5 >= 3.0), tick 1 of 2
        assert!(!h.trigger(53.5, false));
        // Excursion up sustained, tick 2 of 2 -> triggers
        assert!(h.trigger(53.5, false));
        assert_eq!(h.previous_value(), Some(53.5));

        // At limit with ignore_hysteresis_at_limits: even +0.1 triggers after 2 ticks
        assert!(!h.trigger(53.6, true));
        assert!(h.trigger(53.6, true));
    }

    #[test]
    fn test_linear_regression_trend_and_creep() {
        let mut trend = LinearRegressionTrend::new(4, 1.0);
        // Until 4 samples are pushed, returns Unknown
        assert_eq!(trend.update(40.0), Trend::Unknown);
        assert_eq!(trend.update(40.0), Trend::Unknown);
        assert_eq!(trend.update(40.0), Trend::Unknown);
        assert_eq!(trend.update(40.0), Trend::Neutral);

        // Rapid climb: 40, 42, 44, 46 -> slope = 2.0, half_window = 2.0 -> |4.0| > 1.0 -> Up
        trend.update(42.0);
        trend.update(44.0);
        assert_eq!(trend.update(46.0), Trend::Up);
    }
}
