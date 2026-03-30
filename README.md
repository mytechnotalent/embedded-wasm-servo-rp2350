# Embedded Wasm Servo
## WebAssembly Component Model on RP2350 Pico 2

> Part of the [embedded-wasm](https://github.com/mytechnotalent/embedded-wasm) collection — a set of repos that runs a WebAssembly Component Model runtime (Wasmtime + Pulley interpreter) directly on the RP2350 bare-metal with hardware capabilities exposed through WIT.

A pure Embedded Rust project that runs a **WebAssembly Component Model** runtime (Wasmtime + Pulley interpreter) directly on the RP2350 (Raspberry Pi Pico 2) bare-metal. Hardware capabilities are exposed through typed **WIT** (WebAssembly Interface Type) definitions (`embedded:platform/servo` and `embedded:platform/timing`), enabling hardware-agnostic guest programs that are AOT-compiled to Pulley bytecode and executed on the device to control an SG90 servo on GPIO6 — no operating system and no standard library.

<br>

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Project Structure](#project-structure)
- [Source Files](#source-files)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Flashing](#flashing)
- [Testing](#testing)
- [How It Works](#how-it-works)
- [WIT Interface](#wit-interface)
- [Memory Layout](#memory-layout)
- [Extending the Project](#extending-the-project)
- [Troubleshooting](#troubleshooting)
- [Tutorial](#tutorial)
- [License](#license)

<br>

## Overview

This project demonstrates that WebAssembly is not just for browsers — it can run on a microcontroller with 512 KB of RAM. The firmware uses [Wasmtime](https://github.com/bytecodealliance/Wasmtime) with the **Pulley interpreter** (a portable, `no_std`-compatible WebAssembly runtime) and the **WebAssembly Component Model** to execute a precompiled Wasm component that sweeps an SG90 servo from 0 -> 180 -> 0 degrees in 10-degree steps.

**Key properties:**

- **Component Model** — typed WIT interfaces replace raw `env` imports; hardware-agnostic guest programs
- **Pure Rust** — zero C code, zero C bindings, zero FFI
- **Minimal unsafe** — only unavoidable sites (heap init, boot metadata, component deserialize, panic handler UART)
- **AOT compilation** — Wasm is compiled to Pulley bytecode on the host, no compilation on device
- **Industry-standard runtime** — Wasmtime is the reference WebAssembly implementation
- **UART diagnostics** — LED state changes are logged to UART0, panics output file/message over serial

## Architecture

```
┌───────────────────────────────────────────────────────┐
│                 RP2350 (Pico 2)                       │
│                                                       │
│  ┌───────────────────────────────────────────────┐    │
│  │            Firmware (src/main.rs)             │    │
│  │                                               │    │
│  │  ┌─────────┐  ┌────────┐  ┌───────────┐       │    │
│  │  │  Heap   │  │Wasmtime│  │ WIT Host  │       │    │
│  │  │ 256 KiB │  │ Pulley │  │ Trait Impl│       │    │
│  │  └─────────┘  └───┬────┘  └─────┬─────┘       │    │
│  │                   │             │             │    │
│  │  ┌────────┐  ┌────┴─────────────┴──────────┐  │    │
│  │  │ led.rs │  │ Pulley Bytecode (.cwasm)    │  │    │
│  │  │uart.rs │  │                             │  │    │
│  │  └────────┘  │  imports:                   │  │    │
│  │              │    embedded:platform/gpio   │  │    │
│  │              │      set-high(pin: u32)     │  │    │
│  │              │      set-low(pin: u32)      │  │    │
│  │              │    embedded:platform/timing │  │    │
│  │              │      delay-ms(ms: u32)      │  │    │
│  │              │                             │  │    │
│  │              │  exports:                   │  │    │
│  │              │    run()                    │  │    │
│  │              └─────────────────────────────┘  │    │
│  └───────────────────────────────────────────────┘    │
│                                                       │
│  GPIO6 (Onboard LED) -> servo::set_high/set_low(pin)  │
│  GPIO0/1 (UART0) -> uart::write_msg (diag)            │
└───────────────────────────────────────────────────────┘
```

## Project Structure

```
embedded-wasm-servo-rp2350/
├── .cargo/
│   └── config.toml        # ARM Cortex-M33 target, linker flags, picotool runner
├── .vscode/
│   ├── extensions.json    # Recommended VS Code extensions
│   └── settings.json      # Rust-analyzer target configuration
├── wit/
│   └── world.wit          # WIT interface definitions (embedded:platform)
├── wasm-app/              # Wasm servo component (compiled to .wasm)
│   ├── .cargo/
│   │   └── config.toml    # Wasm linker flags (minimal memory)
│   ├── Cargo.toml
│   └── src/
│       └── lib.rs         # Servo logic: wit-bindgen Guest trait, exports run()
├── wasm-tests/            # Integration tests for the Wasm component
│   ├── Cargo.toml
│   ├── build.rs           # Encodes core Wasm as component via ComponentEncoder
│   └── tests/
│       └── integration.rs # 19 tests: component loading, WIT, blink, fuel, pin, size
├── src/
│   ├── main.rs            # Firmware: hardware init, Wasmtime runtime, WIT Host impls
│   ├── led.rs             # GPIO output driver — multi-pin, keyed by pin number
│   ├── uart.rs            # UART0 driver (shared plug-and-play module)
│   └── platform.rs        # Platform TLS glue for Wasmtime no_std
├── build.rs               # Compiles Wasm app, ComponentEncoder, AOT Pulley bytecode
├── Cargo.toml             # Firmware dependencies
├── rp2350.x               # RP2350 memory layout linker script
├── SKILLS.md              # Project conventions and lessons learned
└── README.md              # This file
```

## Source Files

### `wit/world.wit` — WIT Interface Definitions

Defines the `embedded:platform` package with two interfaces (`gpio` and `timing`) and the `servo` world. This is the contract between guest and host — the guest calls `gpio.set-high(pin)` and `timing.delay-ms(ms)` without knowing anything about the hardware. The host maps those calls to real GPIO registers and CPU cycles.

### `wasm-app/src/lib.rs` — Wasm Guest Component

The Wasm component compiled to `wasm32-unknown-unknown`. Uses `wit-bindgen` to generate typed bindings from the WIT definitions. Implements the `Guest` trait with a `run()` function that blinks the LED in an infinite loop at 500ms intervals. GPIO pins are addressed by their hardware number (e.g., 25 for the onboard LED). Requires `dlmalloc` as a global allocator for the canonical ABI's `cabi_realloc`.

### `src/main.rs` — Firmware Entry Point

Orchestrates everything: initializes the heap (256 KiB), clocks, and hardware peripherals, then boots the Wasmtime Pulley engine. Uses `wasmtime::component::bindgen!()` to generate host-side WIT traits, implements `gpio::Host` and `timing::Host` on `HostState`, deserializes the embedded `.cwasm` bytecode as a `Component`, and calls the exported `run()` function. The panic handler uses `uart::panic_init()` and `uart::panic_write()` to output diagnostics over UART0.

### `src/servo.rs` — GPIO Output Driver (Shared Module)

Controls any number of GPIO output pins via a `critical_section::Mutex<RefCell<BTreeMap>>`. Pins are stored by their hardware GPIO number so Wasm code can address them directly (e.g., `gpio::set_high(25)`). The `servo::store_pin(25, pin)` registers a pin, `servo::set_high(25)` / `servo::set_low(25)` toggles it. Accepts any type implementing `embedded_hal::digital::OutputPin` — no dependency on `rp235x-hal`. Marked `#![allow(dead_code)]` — shared plug-and-play module.

### `src/uart.rs` — UART0 Driver (Shared Module)

Provides both HAL-based and raw-register UART0 access. The `uart::init()` accepts only the GPIO0 (TX) and GPIO1 (RX) pins and configures UART0 at 115200 baud, returning just the UART peripheral. Callers retain ownership of all other pins. The `uart::store_global()` stores the UART in a `critical_section::Mutex`. HAL functions: `write_msg()`, `read_byte()`, `write_byte()`. Panic functions (raw registers, no HAL): `panic_init()`, `panic_write()`. Marked `#![allow(dead_code)]` — shared module, identical across repos.

### `src/platform.rs` — Wasmtime TLS Glue

Implements `wasmtime_tls_get()` and `wasmtime_tls_set()` using a global `AtomicPtr`. Required by Wasmtime on `no_std` platforms. On this single-threaded MCU, TLS is just a single atomic pointer.

### `build.rs` — AOT Build Script

Copies the linker script (`rp2350.x` → `memory.x`), spawns a child `cargo build` to compile `wasm-app/` to a core `.wasm` binary, encodes it as a Wasm component via `ComponentEncoder` (using the `wit-bindgen` metadata embedded in the binary), then AOT-compiles the component to Pulley bytecode via Cranelift. Strips `CARGO_ENCODED_RUSTFLAGS` from the child build to prevent ARM linker flags from leaking into the Wasm compilation.

## Prerequisites

### Toolchain

```bash
# Rust (stable)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Required compilation targets
rustup target add thumbv8m.main-none-eabihf # RP2350 ARM Cortex-M33
rustup target add wasm32-unknown-unknown # WebAssembly
```

### Flashing Tool

```bash
# macOS
brew install picotool

# Linux (build from source)
# See https://github.com/raspberrypi/picotool
```

### Serial Terminal (for UART diagnostics)

```bash
# macOS
screen /dev/tty.usbserial* 115200

# Linux
minicom -D /dev/ttyACM0 -b 115200
```

### Optional (Debugging)

```bash
cargo install probe-rs-tools
```

## Building

```bash
cargo build --release
```

This single command does everything:

1. `build.rs` compiles `wasm-app/` to `wasm32-unknown-unknown` → produces `wasm_app.wasm` (core module)
2. `build.rs` encodes the core module as a Wasm component via `ComponentEncoder`
3. `build.rs` AOT-compiles the component to Pulley bytecode via Cranelift → produces `servo.cwasm`
4. The firmware compiles for `thumbv8m.main-none-eabihf`, embedding the Pulley bytecode via `include_bytes!`
5. The result is an ELF at `target/thumbv8m.main-none-eabihf/release/embedded-wasm-servo-rp2350`

## Flashing

```bash
cargo run --release
```

This builds the firmware and flashes it to the Pico 2 via `picotool` (configured as the cargo runner in `.cargo/config.toml`).

> **Note:** Hold the **BOOTSEL** button on the Pico 2 while plugging in the USB cable to enter bootloader mode. Release once connected.

After flashing, the LED on GPIO6 will begin blinking at 500ms intervals. If a USB-to-serial adapter is connected to GPIO0/GPIO1, you will see `GPIO6 On` and `GPIO6 Off` messages at 115200 baud.

## Debugging (VS Code + probe-rs)

### Prerequisites

- [probe-rs](https://probe.rs/) installed
- [probe-rs VS Code extension](https://marketplace.visualstudio.com/items?itemName=probe-rs.probe-rs-debugger) installed
- Debug probe connected to the Pico 2 SWD pins (SWCLK, SWDIO, GND)

### Usage

1. Open the project in VS Code
2. Set breakpoints in `src/main.rs` (or any source file)
3. Press **F5** or select **Run -> Start Debugging**
4. The `debuggable` profile builds with `release` optimizations + full debug symbols (`debug = 2`)
5. probe-rs flashes the ELF and halts at your first breakpoint

The pre-launch task runs:

```bash
cargo build --profile debuggable
```

This produces an ELF at `target/thumbv8m.main-none-eabihf/debuggable/embedded-wasm-servo-rp2350.elf`.

### Variables Panel

> **Warning:** Do **NOT** expand the **Static** dropdown in the Variables panel. It attempts to enumerate every static variable in the binary — including thousands from Wasmtime internals — over the SWD link, causing an infinite spin. Use the **Locals** and **Registers** dropdowns instead.

## Testing

```bash
cd wasm-tests && cargo test
```

Runs all 19 integration tests validating component loading, WIT interface contracts, blink sequencing, timing, pin targeting, binary size, fuel-based execution limits, and error handling.

## How It Works

### 1. The WIT Interface (`wit/world.wit`)

Defines the contract between guest and host:

```wit
package embedded:platform;

interface servo {
    set-angle: func(pin: u32, angle-deg: u32);
}

interface timing {
    delay-ms: func(ms: u32);
}

world servo {
    import gpio;
    import timing;
    export run: func();
}
```

Pin numbers are a guest-side decision. The host maps them to real hardware — the WIT interface is hardware-agnostic.

### 2. The Wasm Guest (`wasm-app/src/lib.rs`)

The guest implements the `Guest` trait generated by `wit-bindgen`:

```rust
wit_bindgen::generate!({ world: "servo", path: "../wit" });

struct BlinkyApp;
export!(BlinkyApp);

impl Guest for BlinkyApp {
    fn run() {
        const LED_PIN: u32 = 25;
        loop {
            gpio::set_high(LED_PIN);
            timing::delay_ms(500);
            gpio::set_low(LED_PIN);
            timing::delay_ms(500);
        }
    }
}
```

No `unsafe`, no register addresses, no HAL — just typed function calls.

### 3. The Firmware Runtime (`src/main.rs`)

The firmware boots in this sequence:

1. **`init_heap()`** — 256 KiB heap for Wasmtime via `embedded-alloc`.
2. **`init_hardware()`** — Clocks, SIO, GPIO, UART0, LED:
   - `uart::init(gpio0, gpio1)` → configures UART0 at 115200 baud (takes only TX/RX pins)
   - `uart::store_global()` → stores UART in mutex
   - `servo::store_pin(25, ...)` → registers GPIO6 as LED output
3. **`run_wasm()`** — Boots the Wasm runtime:
   ```
   create_engine()    → Config::target("pulley32"), bare-metal settings
   create_component() → Component::deserialize(embedded .cwasm bytes)
   Store::new()       → Holds HostState (implements WIT Host traits)
   build_linker()     → Servo::add_to_linker (registers gpio + timing)
   execute_wasm()     → Servo::instantiate() → servo.call_run()
   ```

### 4. The Call Chain

```
Wasm run()
  → gpio::set_high(25)                     [WIT interface call]
    → component model dispatch             [Wasmtime canonical ABI]
      → HostState::set_high(pin: 25)       [gpio::Host trait impl]
        → servo::set_high(25)              [led.rs — HAL pin.set_high()]
        → uart::write_msg("GPIO6 On\n")    [uart.rs — serial output]
  → timing::delay_ms(500)                  [WIT interface call]
    → component model dispatch
      → HostState::delay_ms(ms: 500)       [timing::Host trait impl]
        → cortex_m::asm::delay(75_000_000) [CPU cycle spin]
  → gpio::set_low(25)                      [WIT interface call]
    → ... same pattern ...
```

### 5. The Build Pipeline (`build.rs`)

```
cargo build --release
       │
       ▼
   build.rs runs:
       │
       ├── 1. Copy rp2350.x → OUT_DIR/memory.x (linker script)
       │
       ├── 2. Spawn: cargo build --release --target wasm32-unknown-unknown
       │         └── wasm-app/ compiles → wasm_app.wasm (core module)
       │
       ├── 3. ComponentEncoder encodes core module as Wasm component
       │         └── Uses wit-bindgen metadata embedded in the binary
       │
       ├── 4. AOT-compile component to Pulley bytecode via Cranelift:
       │         └── engine.precompile_component(&component) → servo.cwasm
       │
       └── 5. Main firmware compiles:
               └── include_bytes!("servo.cwasm") embeds the Pulley bytecode
               └── Links against memory.x for RP2350 memory layout
```

Critical detail: `CARGO_ENCODED_RUSTFLAGS` (ARM flags like `--nmagic`, `-Tlink.x`) must be stripped from the child Wasm build via `.env_remove("CARGO_ENCODED_RUSTFLAGS")`.

### 6. Creating a New Project from This Template

1. Copy the repo and rename it.
2. Drop in `uart.rs` and `platform.rs` unchanged — they are plug-and-play.
3. Drop in `led.rs` if your project uses GPIO outputs (any pin, not hardcoded).
4. Edit `wit/world.wit`:
   - Add new interfaces under `package embedded:platform`
   - Import them in your world
5. Edit `wasm-app/src/lib.rs`:
   - `wit_bindgen::generate!()` picks up the new WIT interfaces automatically
   - Implement `Guest::run()` using the generated bindings
6. Edit `src/main.rs`:
   - Implement the new `Host` traits on `HostState`
   - The `bindgen!()` macro and `Servo::add_to_linker()` handle registration
7. `cargo build --release` → `cargo run --release` to flash.

## WIT Interface

| Interface                  | Function   | Signature         | Description                                                     |
| -------------------------- | ---------- | ----------------- | --------------------------------------------------------------- |
| `embedded:platform/gpio`   | `set-high` | `(pin: u32) → ()` | Sets the specified GPIO pin high and logs "GPIO{N} On" to UART0 |
| `embedded:platform/gpio`   | `set-low`  | `(pin: u32) → ()` | Sets the specified GPIO pin low and logs "GPIO{N} Off" to UART0 |
| `embedded:platform/timing` | `delay-ms` | `(ms: u32) → ()`  | Blocks execution for N milliseconds (via CPU cycle counting)    |

## Memory Layout

| Region             | Address      | Size            | Usage                                              |
| ------------------ | ------------ | --------------- | -------------------------------------------------- |
| Flash              | `0x10000000` | 2 MiB           | Firmware code + embedded Wasm component            |
| RAM (striped)      | `0x20000000` | 512 KiB         | Stack + heap + data                                |
| Heap (allocated)   | —            | 256 KiB         | Wasmtime engine, store, component, Wasm linear mem |
| Wasm linear memory | —            | 64 KiB (1 page) | Wasm component's addressable memory                |
| Wasm stack         | —            | 4 KiB           | Wasm call stack                                    |

> **Important:** The default Wasm linker allocates 1 MB of linear memory (16 pages). This exceeds the RP2350's total RAM. The `wasm-app/.cargo/config.toml` explicitly sets `--initial-memory=65536` (1 page) and `stack-size=4096`.

## Extending the Project

### Adding New WIT Interfaces

1. Add the interface in `wit/world.wit`:
   ```wit
   interface serial {
       write: func(data: list<u8>);
   }
   ```

2. Import it in the world:
   ```wit
   world servo {
       import gpio;
       import timing;
       import serial;
       export run: func();
   }
   ```

3. Implement the `Host` trait in `src/main.rs`:
   ```rust
   impl embedded::platform::serial::Host for HostState {
       fn write(&mut self, data: Vec<u8>) {
           uart::write_msg(&data);
       }
   }
   ```

4. The guest can immediately use `serial::write(&data)` — no linker registration needed, `Servo::add_to_linker()` picks up all WIT traits automatically.

### Changing Blink Speed

Edit the delay values in `wasm-app/src/lib.rs`:

```rust
impl Guest for BlinkyApp {
    fn run() {
        const LED_PIN: u32 = 25;
        loop {
            gpio::set_high(LED_PIN);
            timing::delay_ms(100); // 100ms on
            gpio::set_low(LED_PIN);
            timing::delay_ms(900); // 900ms off
        }
    }
}
```

Rebuild and reflash — only the Wasm component changes.

## Troubleshooting

| Symptom                                         | Cause                                  | Fix                                                                              |
| ----------------------------------------------- | -------------------------------------- | -------------------------------------------------------------------------------- |
| LED not blinking after flash                    | Wasm linear memory too large for heap  | Ensure `wasm-app/.cargo/config.toml` has `--initial-memory=65536`                |
| No UART output                                  | Wiring or baud rate wrong              | GPIO0→adapter RX, GPIO1→adapter TX, 115200 8N1                                   |
| `Component::deserialize` panics                 | Config mismatch build vs device        | Both engines must have identical `Config` settings                               |
| `Component::deserialize` panics                 | `default-features` mismatch            | Both `[dependencies]` and `[build-dependencies]` need `default-features = false` |
| Build fails with `unknown argument: --nmagic`   | Parent rustflags leaking to Wasm build | Ensure `build.rs` has `.env_remove("CARGO_ENCODED_RUSTFLAGS")`                   |
| Build fails with `extern blocks must be unsafe` | Rust 2024 edition                      | Use `unsafe extern { ... }` with `safe fn` declarations                          |
| `picotool` can't find device                    | Not in bootloader mode                 | Hold BOOTSEL while plugging in USB                                               |
| `cargo build` doesn't pick up Wasm changes      | Cached build artifacts                 | Run `cargo clean && cargo build --release`                                       |
| ComponentEncoder fails                          | wit-bindgen metadata missing           | Ensure wasm-app uses `wit-bindgen` with `macros` + `realloc` features            |

## Tutorial

A complete line-by-line code walkthrough is available in [TUTORIAL.md](TUTORIAL.md). It covers every Rust source file and every function in the project, written as a teaching resource for learning embedded Wasm development from scratch.

<br>

## License

- [MIT License](LICENSE)
