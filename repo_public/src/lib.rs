//! # Wave-Core Architecture & Interfaces SDK (`wave-core-api`)
//!
//! Deterministic, zero-allocation robotics safety arbiter traits, 48-beam spherical coordinate
//! structures, and real-time telemetry streaming primitives for autonomous aerial and ground systems.
//!
//! ## Core Architecture Modules
//! - [`types`]: 48-beam spherical coordinate structures, 6-DOF kinematics, and cache-line aligned layouts.
//! - [`traits`]: Modular decoupled contracts (`SensorIngestion`, `ReflexController`, `TelemetryStream`, `ActuatorInterface`).
//! - [`arbiter`]: Reference deterministic safety arbiter implementation with dynamic reaction horizons.
//! - [`telemetry`]: Telemetry ring buffer, microsecond latency metrics, and audit packet formatting.
//! - [`c_abi`]: Zero-copy C Foreign Function Interface (FFI) bindings for ROS2 and C/C++ stacks.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod arbiter;
pub mod c_abi;
pub mod telemetry;
pub mod traits;
pub mod types;

// Re-export primary types and traits at crate root
pub use arbiter::{ArbiterConfig, ReferenceSafetyArbiter};
pub use c_abi::{wave_core_api_free, wave_core_api_init, wave_core_api_step, WaveApiConfig, WaveApiOutput};
pub use telemetry::MemoryTelemetryLogger;
pub use traits::{
    ActuatorError, ActuatorInterface, ReflexController, SensorError, SensorHealth, SensorIngestion,
    TelemetryStream,
};
pub use types::{
    ArbiterDecision, BeamChannel, FlightMode3D, KinematicState3D, Quaternion, ReflexAction,
    SphericalBeam, SphericalLidarMesh, Vector3, CEILING_BEAMS, GROUND_BEAMS, PLANAR_BEAMS,
    TOTAL_3D_BEAMS,
};
