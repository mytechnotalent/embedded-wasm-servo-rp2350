//! SPDX-License-Identifier: MIT
//!
//! Copyright (c) 2026 Kevin Thomas
//!
//! # SG90 Servo Driver for RP2350 (Pico 2)
//!
//! Implements SG90 control using RP2350 hardware PWM at 50 Hz on PWM3A
//! (GPIO6). Uses raw PAC register writes to set the PWM compare value,
//! mirroring the Embedded-Hacking C driver's `pwm_set_chan_level` behavior.
//! Pulse range: 1000 us to 2000 us mapped over 0 to 180 degrees.

#![allow(dead_code)]

/// RP2350 HAL shorthand.
use rp235x_hal as hal;

/// Default minimum pulse width in microseconds (0 degrees).
pub const SERVO_DEFAULT_MIN_US: u16 = 1000;
/// Default maximum pulse width in microseconds (180 degrees).
pub const SERVO_DEFAULT_MAX_US: u16 = 2000;
/// Default PWM wrap value for 50 Hz servo (20_000 - 1).
pub const SERVO_WRAP: u32 = 20_000 - 1;
/// Default servo frequency in Hz.
pub const SERVO_HZ: f32 = 50.0;
/// PWM slice index for GPIO6 (PWM3A).
const PWM_SLICE: usize = 3;

/// Convert a pulse width in microseconds to a PWM counter level.
///
/// # Arguments
///
/// * `pulse_us` - Pulse width in microseconds.
/// * `wrap` - PWM counter wrap value.
/// * `hz` - PWM frequency in Hz.
///
/// # Returns
///
/// PWM level suitable for the channel compare register.
pub fn pulse_us_to_level(pulse_us: u32, wrap: u32, hz: f32) -> u32 {
    let period_us = 1_000_000.0f32 / hz;
    let counts_per_us = (wrap + 1) as f32 / period_us;
    (pulse_us as f32 * counts_per_us + 0.5f32) as u32
}

/// Clamp a pulse width to the valid SG90 range.
///
/// # Arguments
///
/// * `pulse_us` - Raw pulse width in microseconds.
/// * `min_us` - Minimum allowed pulse width.
/// * `max_us` - Maximum allowed pulse width.
///
/// # Returns
///
/// Clamped pulse width.
pub fn clamp_pulse_us(pulse_us: u16, min_us: u16, max_us: u16) -> u16 {
    if pulse_us < min_us {
        min_us
    } else if pulse_us > max_us {
        max_us
    } else {
        pulse_us
    }
}

/// Map a servo angle in degrees to a pulse width in microseconds.
///
/// # Arguments
///
/// * `degrees` - Angle in degrees (0.0 to 180.0).
/// * `min_us` - Pulse width at 0 degrees.
/// * `max_us` - Pulse width at 180 degrees.
///
/// # Returns
///
/// Pulse width in microseconds corresponding to the given angle.
pub fn angle_to_pulse_us(degrees: f32, min_us: u16, max_us: u16) -> u16 {
    let d = degrees.clamp(0.0, 180.0);
    let ratio = d / 180.0;
    let span = (max_us - min_us) as f32;
    (min_us as f32 + ratio * span + 0.5) as u16
}

/// Compute the PWM clock divider for the servo frequency.
///
/// # Arguments
///
/// * `sys_hz` - System clock frequency in Hz.
/// * `servo_hz` - Desired servo PWM frequency in Hz.
/// * `wrap` - PWM counter wrap value.
///
/// # Returns
///
/// Clock divider value.
pub fn calc_clk_div(sys_hz: u32, servo_hz: f32, wrap: u32) -> f32 {
    sys_hz as f32 / (servo_hz * (wrap + 1) as f32)
}

/// Writes a PWM level directly to the CC.A register for PWM slice 3.
///
/// This mirrors the C SDK's `pwm_set_chan_level(slice, chan, level)`.
///
/// # Arguments
///
/// * `level` - PWM compare value to write to channel A.
///
/// # Safety
///
/// Accesses the PWM peripheral registers via raw pointer. Safe to call
/// after `init_hardware` has configured PWM slice 3.
fn write_cc_a(level: u16) {
    unsafe {
        (*hal::pac::PWM::ptr())
            .ch(PWM_SLICE)
            .cc()
            .modify(|_, w| w.a().bits(level));
    }
}

/// Sets SG90 angle by writing the PWM compare register directly.
///
/// # Arguments
///
/// * `_gpio_num` - Servo signal pin number (reserved for API compatibility).
/// * `angle_deg` - Requested angle in degrees.
pub fn set_angle(_gpio_num: u8, angle_deg: u16) {
    let pulse = angle_to_pulse_us(angle_deg as f32, SERVO_DEFAULT_MIN_US, SERVO_DEFAULT_MAX_US);
    let clamped = clamp_pulse_us(pulse, SERVO_DEFAULT_MIN_US, SERVO_DEFAULT_MAX_US) as u32;
    let level = pulse_us_to_level(clamped, SERVO_WRAP, SERVO_HZ) as u16;
    write_cc_a(level);
}
