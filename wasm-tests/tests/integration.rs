//! SPDX-License-Identifier: MIT
//!
//! Copyright (c) 2026 Kevin Thomas
//!
//! # Integration Tests for Wasm Servo Component
//!
//! Validates that the compiled Wasm component loads through the Component Model,
//! imports `embedded:platform/servo` and `embedded:platform/timing`, exports
//! `run`, and produces SG90 sweep calls on GPIO6.

use wasmtime::component::{Component, HasSelf};
use wasmtime::{Config, Engine, Store};

wasmtime::component::bindgen!({
    world: "servo-sweep",
    path: "../wit",
});

/// Compiled Wasm servo component embedded at build time.
const WASM_BINARY: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/servo.wasm"));

/// Represents a single host function call recorded during Wasm execution.
#[derive(Debug, PartialEq)]
enum HostCall {
    /// The `servo.set-angle` WIT function was called with pin and angle.
    ServoSetAngle(u32, u32),
    /// The `timing.delay-ms` WIT function was called with the given value.
    DelayMs(u32),
}

/// Host state that records all function calls made by the Wasm guest.
struct TestHostState {
    /// Ordered log of every host function call.
    calls: Vec<HostCall>,
}

impl embedded::platform::servo::Host for TestHostState {
    /// Records a `set-angle` call with pin and angle.
    ///
    /// # Arguments
    ///
    /// * `pin` - GPIO pin number passed by the Wasm guest.
    /// * `angle_deg` - Servo angle in degrees passed by the Wasm guest.
    fn set_angle(&mut self, pin: u32, angle_deg: u32) {
        self.calls.push(HostCall::ServoSetAngle(pin, angle_deg));
    }
}

impl embedded::platform::timing::Host for TestHostState {
    /// Records a `delay-ms` call with the given duration.
    ///
    /// # Arguments
    ///
    /// * `ms` - Delay duration in milliseconds passed by the Wasm guest.
    fn delay_ms(&mut self, ms: u32) {
        self.calls.push(HostCall::DelayMs(ms));
    }
}

/// Creates a wasmtime engine with fuel metering enabled.
///
/// # Returns
///
/// A wasmtime `Engine` configured to consume fuel.
///
/// # Panics
///
/// Panics if engine creation fails.
fn create_fuel_engine() -> Engine {
    let mut config = Config::default();
    config.consume_fuel(true);
    Engine::new(&config).expect("create fuel engine")
}

/// Compiles the embedded Wasm binary into a wasmtime component.
///
/// # Arguments
///
/// * `engine` - The wasmtime engine to compile with.
///
/// # Returns
///
/// The compiled Wasm `Component`.
///
/// # Panics
///
/// Panics if the Wasm binary is invalid.
fn compile_component(engine: &Engine) -> Component {
    Component::new(engine, WASM_BINARY).expect("valid Wasm component")
}

/// Builds a fully configured test linker with all WIT interfaces registered.
///
/// # Arguments
///
/// * `engine` - The wasmtime engine to associate the linker with.
///
/// # Returns
///
/// A component `Linker` with servo and timing host traits registered.
///
/// # Panics
///
/// Panics if WIT interface registration fails.
fn build_test_linker(engine: &Engine) -> wasmtime::component::Linker<TestHostState> {
    let mut linker = wasmtime::component::Linker::new(engine);
    ServoSweep::add_to_linker::<TestHostState, HasSelf<TestHostState>>(
        &mut linker,
        |state: &mut TestHostState| state,
    )
    .expect("register WIT interfaces");
    linker
}

/// Creates a store with an empty call log and the given fuel budget.
///
/// # Arguments
///
/// * `engine` - The wasmtime engine to create the store for.
/// * `fuel` - The amount of fuel to allocate for execution.
///
/// # Returns
///
/// A `Store` containing an empty `TestHostState` with the fuel budget set.
///
/// # Panics
///
/// Panics if fuel allocation fails.
fn create_fueled_store(engine: &Engine, fuel: u64) -> Store<TestHostState> {
    let mut store = Store::new(engine, TestHostState { calls: Vec::new() });
    store.set_fuel(fuel).expect("set fuel");
    store
}

/// Runs the Wasm `run` function until fuel is exhausted.
///
/// # Arguments
///
/// * `store` - The wasmtime store with fuel and host state.
/// * `linker` - The component linker with WIT interfaces registered.
/// * `component` - The compiled Wasm component.
///
/// # Panics
///
/// Panics if component instantiation fails.
fn run_until_out_of_fuel(
    store: &mut Store<TestHostState>,
    linker: &wasmtime::component::Linker<TestHostState>,
    component: &Component,
) {
    let servo =
        ServoSweep::instantiate(&mut *store, component, linker).expect("instantiate component");
    let _ = servo.call_run(&mut *store);
}

/// Verifies that the Wasm component binary loads without error.
///
/// # Panics
///
/// Panics if the Wasm component binary fails to compile.
#[test]
fn test_wasm_component_loads() {
    let engine = create_fuel_engine();
    let _component = compile_component(&engine);
}

/// Verifies that required WIT interface imports are present.
///
/// # Panics
///
/// Panics if required imports are missing.
#[test]
fn test_wasm_imports_match_expected() {
    let engine = create_fuel_engine();
    let component = compile_component(&engine);
    let ty = component.component_type();
    let import_names: Vec<_> = ty
        .imports(&engine)
        .map(|(name, _)| name.to_string())
        .collect();
    assert!(
        import_names.iter().any(|n| n == "embedded:platform/servo"),
        "missing embedded:platform/servo import, got {import_names:?}"
    );
    assert!(
        import_names.iter().any(|n| n == "embedded:platform/timing"),
        "missing embedded:platform/timing import, got {import_names:?}"
    );
}

/// Verifies that `run` starts SG90 sweep on GPIO6 with 10-degree stepping.
///
/// # Panics
///
/// Panics if call sequence does not match expected startup pattern.
#[test]
fn test_run_generates_servo_calls() {
    let engine = create_fuel_engine();
    let component = compile_component(&engine);
    let linker = build_test_linker(&engine);
    let mut store = create_fueled_store(&engine, 2_000_000);
    run_until_out_of_fuel(&mut store, &linker, &component);
    let calls = &store.data().calls;
    assert!(!calls.is_empty(), "guest should call host imports");
    assert_eq!(calls[0], HostCall::ServoSetAngle(6, 0));
    assert_eq!(calls[1], HostCall::DelayMs(200));
    assert_eq!(calls[2], HostCall::ServoSetAngle(6, 10));
    assert_eq!(calls[3], HostCall::DelayMs(200));
}
