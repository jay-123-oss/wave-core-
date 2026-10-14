//! Trait Definitions & Component Interfaces.
//!
//! Provides the core traits for decoupling sensor drivers, deterministic safety
//! arbiters, telemetry loggers, and hardware flight control interfaces.
//!
//! Designed for `#![no_std]` real-time robotics architectures.

use crate::types::{
    ArbiterDecision, KinematicState3D, SphericalLidarMesh, Vector3, TOTAL_3D_BEAMS,
};

/// Error types occurring during hardware sensor acquisition and verification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorError {
    /// Sensor hardware timeout or communication bus failure (e.g. I2C/SPI dropout).
    HardwareTimeout,
    /// Data payload corrupted, parity error, or invalid CRC checksum.
    DataCorrupted,
    /// Values outside physical operating thresholds (e.g. negative distances or infinite readings).
    OutOfRange,
    /// Sensor frozen: consecutive identical frame sequence detected.
    SensorFrozen,
}

/// Overall sensor subsystem operational health.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorHealth {
    /// Nominal operation with valid timestamps and nominal noise floor.
    Healthy = 0,
    /// Operating with degraded coverage or intermittent packet loss.
    Degraded = 1,
    /// Critical hardware fault, zero valid rays, or persistent dropout.
    Fault = 2,
}

/// Interface for ingesting raw perception streams into normalized spatial representations.
pub trait SensorIngestion {
    /// Ingests a pre-formatted 48-beam volumetric spherical mesh.
    fn ingest_spherical_mesh(&mut self, mesh: &SphericalLidarMesh) -> Result<(), SensorError>;

    /// Ingests 48 raw scalar distance ranges from hardware drivers.
    fn ingest_raw_ranges(
        &mut self,
        ranges: &[f32; TOTAL_3D_BEAMS],
        timestamp_ns: u64,
    ) -> Result<(), SensorError>;

    /// Updates internal vehicle state with 6-DOF odometry / IMU fusion.
    fn update_kinematics(&mut self, state: &KinematicState3D) -> Result<(), SensorError>;

    /// Returns the current hardware sensor health indicator.
    fn health_status(&self) -> SensorHealth;

    /// Returns the latest calibrated spherical mesh.
    fn latest_mesh(&self) -> &SphericalLidarMesh;
}

/// Deterministic Safety Arbiter & Reflex Controller Interface.
///
/// Implementations of this trait intercept nominal autopilot or pilot velocity
/// requests, evaluate collision probability across 3D volumetric space, and
/// substitute safe escape interventions if a collision boundary is breached.
pub trait ReflexController {
    /// Evaluates current spatial hazards and outputs an intervention decision.
    ///
    /// # Arguments
    /// * `nominal_cmd` - Commanded linear velocity from higher-level mission planner (m/s).
    /// * `dt_seconds` - Delta time between control cycles (e.g. 0.001 for 1,000 Hz).
    ///
    /// # Returns
    /// An `ArbiterDecision` containing sanitized commands and override status.
    fn arbitrate(&mut self, nominal_cmd: &Vector3, dt_seconds: f32) -> ArbiterDecision;

    /// Returns whether the safety arbiter is currently overriding the nominal path.
    fn is_intervening(&self) -> bool;

    /// Sets the dynamic trigger threshold distance in meters.
    fn set_trigger_distance(&mut self, trigger_distance_m: f32);

    /// Resets internal temporal filters and escape state machines back to nominal.
    fn reset(&mut self);
}

/// Interface for high-rate, zero-copy telemetry streaming and audit logging.
pub trait TelemetryStream {
    /// Logs a single closed-loop control cycle with microsecond timestamps.
    fn log_cycle(&mut self, decision: &ArbiterDecision, kinematics: &KinematicState3D);

    /// Formats a standardized 128-byte binary or ASCII audit telemetry packet.
    fn format_audit_packet(&self, decision: &ArbiterDecision) -> [u8; 128];

    /// Returns total recorded cycles and intervention counts.
    fn statistics(&self) -> (u64, u64); // (total_cycles, intervention_cycles)

    /// Flushes any buffered telemetry to the underlying communication ring buffer.
    fn flush(&mut self);
}

/// Error types occurring during actuator dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorError {
    /// Commanded velocity or thrust exceeds airframe dynamic limitations.
    DynamicLimitsExceeded,
    /// Communication link with ESCs or flight controller (MAVLink/CAN) disconnected.
    LinkDisconnected,
    /// Actuator watchdog triggered.
    WatchdogAlert,
}

/// Interface for dispatching sanitized velocity / thrust commands to flight controllers.
pub trait ActuatorInterface {
    /// Dispatches an arbiter decision to the physical or simulated motor mixers.
    fn dispatch(&mut self, decision: &ArbiterDecision) -> Result<(), ActuatorError>;

    /// Clamps commanded thrust within safe airframe envelope.
    fn clamp_thrust(&self, raw_thrust: f32, max_factor: f32) -> f32;

    /// Sends an immediate emergency kill / zero-throttle signal.
    fn emergency_disarm(&mut self) -> Result<(), ActuatorError>;
}
