//! SPDX-License-Identifier: MIT
//!
//! Copyright (c) 2026 Kevin Thomas
//!
//! # Wasm SG90 Servo Sweep Component
//!
//! A minimal WebAssembly component that sweeps an SG90 servo on an RP2350
//! Pico 2 by calling host-provided servo and delay functions through typed
//! WIT interfaces. Servo pins are addressed by their hardware pin number
//! (e.g., 6 for the SG90 signal wire).

#![no_std]

// Enable the global allocator for heap-backed collections.
extern crate alloc;

use core::panic::PanicInfo; // Panic handler signature type.

/// Global heap allocator required by the canonical ABI's `cabi_realloc`.
#[global_allocator]
static ALLOC: dlmalloc::GlobalDlmalloc = dlmalloc::GlobalDlmalloc;

use embedded::platform::{servo, timing}; // Host-provided servo and timing imports.

// Generate guest-side bindings for the `servo-sweep` WIT world.
wit_bindgen::generate!({
    world: "servo-sweep",
    path: "../wit",
});

/// Wasm guest component implementing the `servo` world.
struct ServoApp;

// Register `ServoApp` as the component's exported implementation.
export!(ServoApp);

/// Sweeps SG90 from 0 to 180 degrees in 10-degree steps.
///
/// # Arguments
///
/// * `pin` - Servo signal pin number.
fn sweep_up(pin: u32) {
    let mut angle = 0u32;
    while angle <= 180 {
        servo::set_angle(pin, angle);
        timing::delay_ms(200);
        angle += 10;
    }
}

/// Sweeps SG90 from 180 to 0 degrees in 10-degree steps.
///
/// # Arguments
///
/// * `pin` - Servo signal pin number.
fn sweep_down(pin: u32) {
    let mut angle = 180u32;
    while angle > 0 {
        servo::set_angle(pin, angle);
        timing::delay_ms(200);
        angle -= 10;
    }
    servo::set_angle(pin, 0);
    timing::delay_ms(200);
}

impl Guest for ServoApp {
    /// Initializes SG90 on GPIO6 and continuously sweeps 0-180-0.
    fn run() {
        /// Hardware GPIO pin number for the SG90 signal wire.
        const SERVO_PIN: u32 = 6;
        loop {
            sweep_up(SERVO_PIN);
            sweep_down(SERVO_PIN);
        }
    }
}

/// Panic handler for the Wasm environment that halts in an infinite loop.
///
/// # Arguments
///
/// * `_info` - Panic information (unused in the Wasm environment).
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
