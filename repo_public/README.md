# Wave-Core Architecture & Interfaces SDK (`wave-core-api`)

[![Crate Version](https://img.shields.io/badge/crate-v0.1.0-blue.svg)](Cargo.toml)
[![Rust Edition](https://img.shields.io/badge/rustc-2021%2B-orange.svg)](Cargo.toml)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-green.svg)](LICENSE-MIT)
[![Memory Policy](https://img.shields.io/badge/alloc-zero--heap%20%23!%5Bno__std%5D-brightgreen.svg)](#memory-layout--robust-type-system)
[![Test Suite](https://img.shields.io/badge/tests-100%25%20passing-success.svg)](#reproduction-steps)

> **Deterministic, Zero-Allocation Robotics Safety Arbiter Trait Definitions, 48-Beam Spherical Coordinate Structures, and Real-Time Verification Harness.**

---

## 📖 Overview

The **Wave-Core Architecture & Interfaces SDK** (`wave-core-api`) provides an open, robust, strictly typed Rust interface specification for mission-critical robotics and autonomous drone avionics. 

It defines the formal trait boundaries, cache-aligned spatial data layouts, and deterministic safety arbitration interfaces that decouple:
1. **Raw Sensor Hardware Drivers** (LiDAR point clouds, 360° laser scanners, time-of-flight arrays).
2. **Deterministic Safety Reflex Arbiters** (sub-millisecond velocity clamping, vertical vaults, and emergency braking).
3. **Flight Control Autopilots** (PX4 Autopilot, ArduPilot, MAVLink v2, micro-ROS, and bare-metal STM32 firmware).
4. **Real-Time Telemetry Audit Streams** (microsecond jitter profiling and ISO 26262 / DO-178C style audit logging).

---

## 🏛️ Architecture Block Diagram

```mermaid
graph TD
    subgraph Hardware Sensors & Ingestion
        A1["LiDAR / ToF Scanner (48 Beams)"] --> B1["SensorIngestion Trait"]
        A2["IMU / 6-DOF Odometry"] --> B1
        B1 --> C["SphericalLidarMesh [48 Beams] (align 64)"]
    end

    subgraph Perception Layout (48 Beams)
        C --> D1["Ceiling Tier (+45 deg, 8 Beams)"]
        C --> D2["Planar Tier (0 deg, 32 Beams)"]
        C --> D3["Ground Tier (-45 deg, 8 Beams)"]
    end

    subgraph Safety Arbitration Kernel
        D1 & D2 & D3 --> E["ReflexController Trait<br/>(ReferenceSafetyArbiter)"]
        G["Nominal Flight Plan (/cmd_vel)"] --> E
        E --> H{"Collision Boundary<br/>Breached?"}
        H -- "No (Clear)" --> I["Nominal Passthrough"]
        H -- "Ground < 1.0m" --> J["ReflexAction::VerticalEscape"]
        H -- "Planar < d_trigger" --> K["ReflexAction::LateralSlalom / Brake"]
    end

    subgraph Dispatch & Actuation
        I & J & K --> L["ArbiterDecision Packet"]
        L --> M1["ActuatorInterface Trait"]
        L --> M2["TelemetryStream Trait"]
        M1 --> N1["MAVLink v2 (SET_POSITION_TARGET_LOCAL_NED)"]
        M1 --> N2["ROS2 /cmd_vel (Zero-Copy C-ABI)"]
        M1 --> N3["Bare-Metal PWM / DShot ESC Mixer"]
        M2 --> O["Microsecond Latency Log & Ring Buffer"]
    end

    style C fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#fff
    style E fill:#1e1b4b,stroke:#818cf8,stroke-width:2px,color:#fff
    style L fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#fff
```

---

## 🔬 Memory Layout & Robust Type System

The type system is designed for **deterministic execution**, `#![no_std]` compliance, and maximum hardware cache locality:

1. **64-Byte CPU Cache-Line Alignment**:
   `SphericalLidarMesh` is decorated with `#[repr(C, align(64))]` to prevent false sharing in multi-core embedded systems and ensure single-cycle vector loads.
2. **48-Beam Spherical Coordinates**:
   Partitioned into three distinct spatial channels:
   - **Ceiling Channel (`[0..8]`)**: $+45^\circ$ elevation angle, $45^\circ$ azimuth spacing. Monitors power lines, tree canopies, and overhanging beams.
   - **Planar Channel (`[8..40]`)**: $0^\circ$ elevation angle, $11.25^\circ$ azimuth spacing. Full $360^\circ$ planar coverage.
   - **Ground Channel (`[40..48]`)**: $-45^\circ$ elevation angle, $45^\circ$ azimuth spacing. Detects terrain proximity, sudden sinkholes, and landing hazards.
3. **Zero Dynamic Allocation**:
   No `Vec`, `String`, or dynamic heap structures in the hot arbitration loop. All state is stored in statically sized arrays and registers.

```rust
use wave_core_api::types::{SphericalLidarMesh, BeamChannel, ArbiterDecision, Vector3};

// Pre-computed 48-beam spatial mesh fitting into L1 data cache
let mut mesh = SphericalLidarMesh::new_empty();
let raw_ranges = [25.0f32; 48];
mesh.update_ranges(&raw_ranges, 1_000_000);

assert_eq!(mesh.beams.len(), 48);
assert_eq!(mesh.beams[0].channel, BeamChannel::Ceiling);
assert_eq!(mesh.beams[8].channel, BeamChannel::Planar);
assert_eq!(mesh.beams[40].channel, BeamChannel::Ground);
```

---

## 🧩 Primary Trait Specifications

### 1. `SensorIngestion`
```rust
pub trait SensorIngestion {
    fn ingest_spherical_mesh(&mut self, mesh: &SphericalLidarMesh) -> Result<(), SensorError>;
    fn ingest_raw_ranges(&mut self, ranges: &[f32; 48], timestamp_ns: u64) -> Result<(), SensorError>;
    fn update_kinematics(&mut self, state: &KinematicState3D) -> Result<(), SensorError>;
    fn health_status(&self) -> SensorHealth;
    fn latest_mesh(&self) -> &SphericalLidarMesh;
}
```

### 2. `ReflexController`
```rust
pub trait ReflexController {
    fn arbitrate(&mut self, nominal_cmd: &Vector3, dt_seconds: f32) -> ArbiterDecision;
    fn is_intervening(&self) -> bool;
    fn set_trigger_distance(&mut self, trigger_distance_m: f32);
    fn reset(&mut self);
}
```

### 3. `TelemetryStream`
```rust
pub trait TelemetryStream {
    fn log_cycle(&mut self, decision: &ArbiterDecision, kinematics: &KinematicState3D);
    fn format_audit_packet(&self, decision: &ArbiterDecision) -> [u8; 128];
    fn statistics(&self) -> (u64, u64); // (total_cycles, intervention_cycles)
    fn flush(&mut self);
}
```

### 4. `ActuatorInterface`
```rust
pub trait ActuatorInterface {
    fn dispatch(&mut self, decision: &ArbiterDecision) -> Result<(), ActuatorError>;
    fn clamp_thrust(&self, raw_thrust: f32, max_factor: f32) -> f32;
    fn emergency_disarm(&mut self) -> Result<(), ActuatorError>;
}
```

---

## 🛠️ Hardware Support Matrix

| Hardware Platform | Target Architecture | Environment | Communication Interface | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Raspberry Pi CM4** | `aarch64-unknown-linux-gnu` | Linux / RT-PREEMPT | ROS2 Zero-Copy / UART MAVLink | ✅ **Supported** |
| **STM32F4 / F7 / H7** | `thumbv7em-none-eabihf` | Bare-Metal `#![no_std]` | SPI / CAN / UART | ✅ **Supported** |
| **Pixhawk (FMUv5X / v6X)** | `thumbv7em-none-eabi` | NuttX RTOS (PX4) | uORB / MAVLink v2 | ✅ **Supported** |
| **NVIDIA Jetson Orin** | `aarch64-unknown-linux-gnu` | Ubuntu 22.04 LTS | ROS2 Humble / Iron (`rclcpp`) | ✅ **Supported** |
| **C/C++ Foreign Systems**| `C-ABI (c_abi.rs)` | Any POSIX / Embedded | Shared library (`.so`, `.a`) | ✅ **Supported** |

---

## ⚡ C-ABI Integration (C / C++ / ROS2)

A zero-copy C-compatible Foreign Function Interface is exported in [`c_abi.rs`](src/c_abi.rs):

```c
#include <stdio.h>

// Struct signature matching wave_core_api C-ABI
typedef struct {
    uint32_t action_id;
    uint32_t flight_mode;
    float cmd_vx;
    float cmd_vy;
    float cmd_vz;
    float thrust_norm;
    float min_clearance_m;
    float latency_us;
    uint8_t is_intervening;
    uint8_t anomaly_alert;
} WaveApiOutput;

// External Rust function declarations
extern void* wave_core_api_init(const void* config);
extern WaveApiOutput wave_core_api_step(void* ctx, const float* ranges, size_t len,
                                       float vx, float vy, float vz, float dt);
extern void wave_core_api_free(void* ctx);
```

---

## 🚀 Reproduction Steps

Ensure Rust 1.75+ is installed (`rustup default stable`).

### 1. Compile the Crate
```bash
cargo check
```

### 2. Run the Full Test Suite
```bash
cargo test
```
*Output: All 5 integration tests verifying 48-beam spatial geometry, safety arbitration, and C-ABI pass with 100% success.*

### 3. Run the High-Rate Benchmark Harness
```bash
cargo run --example harness_runner --release
```
*Executes 10,000 closed-loop cycles across 3 flight hazard phases and displays real-time latency percentiles, CPU usage, and collision verification.*

---

## 📄 License

This public SDK and interface specification is dual-licensed under:
- **MIT License** ([LICENSE-MIT](LICENSE-MIT))
- **Apache License, Version 2.0** ([LICENSE-APACHE](LICENSE-APACHE))

Standard open-source compliance for the **NLnet Foundation**, academic grant audits, and community peer review.
