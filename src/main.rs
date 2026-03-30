//! SPDX-License-Identifier: MIT
//!
//! Copyright (c) 2026 Kevin Thomas
//!
//! # Wasm SG90 Servo Firmware for RP2350 (Pico 2)
//!
//! This firmware runs a WebAssembly Component Model runtime on the RP2350
//! bare-metal using wasmtime with the Pulley interpreter. A precompiled Wasm
//! component controls an SG90 servo on GPIO6 through typed WIT interfaces
//! (`embedded:platform/servo` and `embedded:platform/timing`).

#![no_std]
#![no_main]

// Enable the global allocator for heap-backed collections.
extern crate alloc;

/// WIT host-import implementations for GPIO and timing.
mod platform;
/// SG90 servo control abstraction.
mod servo;
/// UART peripheral setup and I/O helpers.
mod uart;

/// Panic handler signature type.
use core::panic::PanicInfo;
/// Linked-list first-fit heap allocator.
use embedded_alloc::LlffHeap as Heap;
/// Clock trait for reading system clock frequency.
use hal::Clock;
/// RP2350 HAL shorthand.
use rp235x_hal as hal;
/// Component Model loader and linker traits.
use wasmtime::component::{Component, HasSelf};
/// Wasmtime runtime core types.
use wasmtime::{Config, Engine, Store};

// Generate host-side bindings for the `servo-sweep` WIT world.
wasmtime::component::bindgen!({
    world: "servo-sweep",
    path: "wit",
});

/// Global heap allocator backed by a statically allocated memory region.
///
/// Uses the linked-list first-fit allocation strategy from `embedded-alloc`.
#[global_allocator]
static HEAP: Heap = Heap::empty();

/// External crystal oscillator frequency in Hz.
const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

/// Heap size in bytes (256 KiB of the available 512 KiB RAM).
const HEAP_SIZE: usize = 262_144;

/// Precompiled Pulley bytecode for the Wasm component, embedded at build time.
const WASM_BINARY: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/servo.cwasm"));

/// Hardware GPIO pin number for the SG90 servo signal wire.
const SERVO_PIN: u8 = 6;

/// RP2350 boot metadata placed in the `.start_block` section for the Boot ROM.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Host state providing WIT interface implementations via the wasmtime store.
///
/// Holds a SysTick-based delay timer for accurate host-side timing,
/// matching the reference Rust driver's `cortex_m::delay::Delay` approach.
struct HostState {
    /// SysTick-based blocking delay provider.
    delay: cortex_m::delay::Delay,
}

impl embedded::platform::servo::Host for HostState {
    /// Sets the specified SG90 servo angle and logs it to UART0.
    ///
    /// # Arguments
    ///
    /// * `pin` - Hardware GPIO pin number carrying the servo PWM signal.
    /// * `angle_deg` - Target angle in degrees.
    fn set_angle(&mut self, pin: u32, angle_deg: u32) {
        servo::set_angle(pin as u8, angle_deg as u16);
        write_servo_msg(pin as u8, angle_deg);
    }
}

impl embedded::platform::timing::Host for HostState {
    /// Blocks execution for the specified duration via SysTick timer.
    ///
    /// # Arguments
    ///
    /// * `ms` - Delay duration in milliseconds.
    fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }
}

/// Panic handler that outputs a diagnostic message over UART0.
///
/// Initializes UART0 from scratch (in case it was never set up) and
/// writes the panic location and message to UART0, then halts.
///
/// # Arguments
///
/// * `info` - Panic information containing the location and message.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    uart::panic_init();
    uart::panic_write(b"\n!!! PANIC !!!\n");
    if let Some(location) = info.location() {
        uart::panic_write(b"Location: ");
        uart::panic_write(location.file().as_bytes());
        uart::panic_write(b"\n");
    }
    if let Some(msg) = info.message().as_str() {
        uart::panic_write(b"Message: ");
        uart::panic_write(msg.as_bytes());
        uart::panic_write(b"\n");
    }
    loop {
        cortex_m::asm::wfe();
    }
}

/// Initializes the global heap allocator from a static memory region.
///
/// # Safety
///
/// Must be called exactly once before any heap allocations occur.
/// Uses `unsafe` to initialize the allocator with a raw pointer to static memory.
fn init_heap() {
    use core::mem::MaybeUninit;
    /// Static memory region backing the global heap allocator.
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

/// Initializes system clocks and PLLs from the external crystal oscillator.
///
/// # Arguments
///
/// * `xosc` - External oscillator peripheral.
/// * `clocks` - Clocks peripheral.
/// * `pll_sys` - System PLL peripheral.
/// * `pll_usb` - USB PLL peripheral.
/// * `resets` - Resets peripheral for subsystem reset control.
/// * `watchdog` - Watchdog timer used as the clock reference.
///
/// # Returns
///
/// The configured clocks manager for peripheral clock access.
///
/// # Panics
///
/// Panics if clock initialization fails.
fn init_clocks(
    xosc: hal::pac::XOSC,
    clocks: hal::pac::CLOCKS,
    pll_sys: hal::pac::PLL_SYS,
    pll_usb: hal::pac::PLL_USB,
    resets: &mut hal::pac::RESETS,
    watchdog: &mut hal::Watchdog,
) -> hal::clocks::ClocksManager {
    hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        xosc,
        clocks,
        pll_sys,
        pll_usb,
        resets,
        watchdog,
    )
    .ok()
    .unwrap()
}

/// Writes a servo angle message to UART0.
///
/// # Arguments
///
/// * `pin` - GPIO pin number carrying the servo signal.
/// * `angle` - Servo angle in degrees.
fn write_servo_msg(pin: u8, angle: u32) {
    uart::write_msg(b"GPIO");
    let mut buf = [0u8; 3];
    let len = fmt_u8(pin, &mut buf);
    uart::write_msg(&buf[..len]);
    uart::write_msg(b" Angle ");
    let mut angle_buf = [0u8; 3];
    let angle_len = fmt_u8(angle as u8, &mut angle_buf);
    uart::write_msg(&angle_buf[..angle_len]);
    uart::write_msg(b"\n");
}

/// Formats a `u8` as decimal digits into the provided buffer.
///
/// # Arguments
///
/// * `n` - The number to format.
/// * `buf` - Output buffer (must be at least 3 bytes).
///
/// # Returns
///
/// The number of digits written to `buf`.
fn fmt_u8(mut n: u8, buf: &mut [u8; 3]) -> usize {
    if n == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut i = 0;
    let mut tmp = [0u8; 3];
    while n > 0 {
        tmp[i] = b'0' + (n % 10);
        n /= 10;
        i += 1;
    }
    for j in 0..i {
        buf[j] = tmp[i - 1 - j];
    }
    i
}

/// Initializes all RP2350 hardware peripherals.
///
/// Sets up the watchdog, clocks, SIO, and GPIO pins. Passes only GPIO0
/// (TX) and GPIO1 (RX) to `uart::init()`, keeping all other pins under
/// `main.rs` control. Configures PWM3A on GPIO6 for SG90 control using
/// the same clock divider computation as the C reference driver.
///
/// # Returns
///
/// A SysTick-based blocking delay timer for use by the Wasm host.
///
/// # Panics
///
/// Panics if the hardware peripherals have already been taken.
fn init_hardware() -> cortex_m::delay::Delay {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks(
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    );
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let sys_hz = clocks.system_clock.freq().to_Hz();
    let div = servo::calc_clk_div(sys_hz, servo::SERVO_HZ, servo::SERVO_WRAP);
    let div_int = div as u8;
    let div_frac = (((div - div_int as f32) * 16.0) as u8).min(15);
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm3 = pwm_slices.pwm3;
    pwm3.set_div_int(div_int);
    pwm3.set_div_frac(div_frac);
    pwm3.set_top(servo::SERVO_WRAP as u16);
    pwm3.enable();
    pwm3.channel_a.output_to(pins.gpio6);
    let uart_dev = uart::init(pac.UART0, &mut pac.RESETS, &clocks, pins.gpio0, pins.gpio1);
    uart::store_global(uart_dev);
    servo::set_angle(SERVO_PIN, 0);
    cortex_m::delay::Delay::new(core.SYST, sys_hz)
}

/// Creates a wasmtime engine configured for Pulley on bare-metal.
///
/// Explicitly targets `pulley32` to match the AOT cross-compilation in
/// `build.rs`. All settings must be identical between build-time and
/// runtime engines or `Component::deserialize` will fail. OS-dependent
/// features are disabled and memory limits are tuned for the RP2350's
/// 512 KiB RAM.
///
/// # Returns
///
/// A configured wasmtime `Engine` targeting the Pulley 32-bit interpreter.
///
/// # Panics
///
/// Panics if the engine configuration fails.
fn create_engine() -> Engine {
    let mut config = Config::new();
    config.target("pulley32").expect("set pulley32 target");
    config.signals_based_traps(false);
    config.memory_init_cow(false);
    config.memory_reservation(0);
    config.memory_guard_size(0);
    config.memory_reservation_for_growth(0);
    config.guard_before_linear_memory(false);
    config.max_wasm_stack(16384);
    Engine::new(&config).expect("create Pulley engine")
}

/// Deserializes the precompiled Pulley component from embedded bytes.
///
/// # Safety
///
/// Uses `unsafe` to call `Component::deserialize` which requires that the
/// embedded bytes are a valid serialized wasmtime component. This invariant
/// is upheld because the bytes are produced by our build script.
///
/// # Arguments
///
/// * `engine` - Engine with matching Pulley configuration.
///
/// # Returns
///
/// The deserialized wasmtime `Component`.
///
/// # Panics
///
/// Panics if the embedded Pulley bytecode is invalid.
fn create_component(engine: &Engine) -> Component {
    unsafe { Component::deserialize(engine, WASM_BINARY) }.expect("valid Pulley component")
}

/// Builds the component linker with all WIT interface bindings registered.
///
/// Uses the `bindgen!`-generated `ServoSweep::add_to_linker` to register
/// host implementations for `embedded:platform/servo` and
/// `embedded:platform/timing`.
///
/// # Arguments
///
/// * `engine` - Wasm engine that the linker is associated with.
///
/// # Returns
///
/// A configured component `Linker` with all WIT interfaces registered.
///
/// # Panics
///
/// Panics if any interface fails to register.
fn build_linker(engine: &Engine) -> wasmtime::component::Linker<HostState> {
    let mut linker = wasmtime::component::Linker::new(engine);
    ServoSweep::add_to_linker::<HostState, HasSelf<HostState>>(
        &mut linker,
        |state: &mut HostState| state,
    )
    .expect("register WIT interfaces");
    linker
}

/// Instantiates the Wasm component and executes the exported `run` function.
///
/// # Arguments
///
/// * `store` - Wasm store holding the host state.
/// * `linker` - Component linker with WIT interfaces registered.
/// * `component` - Precompiled Wasm component to instantiate.
///
/// # Panics
///
/// Panics if instantiation fails or the `run` export is not found.
fn execute_wasm(
    store: &mut Store<HostState>,
    linker: &wasmtime::component::Linker<HostState>,
    component: &Component,
) {
    let servo =
        ServoSweep::instantiate(&mut *store, component, linker).expect("instantiate component");
    servo.call_run(&mut *store).expect("execute run");
}

/// Loads and runs the Wasm SG90 servo component.
///
/// # Arguments
///
/// * `delay` - SysTick-based delay timer for host-side timing.
fn run_wasm(delay: cortex_m::delay::Delay) -> ! {
    let engine = create_engine();
    let component = create_component(&engine);
    let mut store = Store::new(&engine, HostState { delay });
    let linker = build_linker(&engine);
    uart::write_msg(b"Servo driver initialized on GPIO 6\r\n");
    uart::write_msg(b"Sweeping 0 -> 180 -> 0 degrees in 10-degree steps\r\n");
    execute_wasm(&mut store, &linker, &component);
    loop {
        cortex_m::asm::wfe();
    }
}

/// Firmware entry point that initializes hardware and runs the Wasm servo.
#[hal::entry]
fn main() -> ! {
    init_heap();
    let delay = init_hardware();
    run_wasm(delay)
}
