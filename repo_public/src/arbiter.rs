//! Reference Deterministic Safety Arbiter.
//!
//! Provides a clean, open-source reference implementation of the `ReflexController`
//! and `SensorIngestion` traits. Demonstrates sub-millisecond safety arbitration,
//! dynamic trigger cones, and vertical obstacle avoidance without exposing proprietary
//! neuromorphic hypervector codebooks or classified algorithmic assets.

use crate::traits::{ReflexController, SensorError, SensorHealth, SensorIngestion};
use crate::types::{
    ArbiterDecision, FlightMode3D, KinematicState3D, ReflexAction, SphericalLidarMesh, Vector3,
    TOTAL_3D_BEAMS,
};

/// Configuration parameters for the reference safety arbiter.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ArbiterConfig {
    /// Nominal airframe mass in kilograms.
    pub mass_kg: f32,
    /// Maximum allowed airframe tilt angle in degrees.
    pub max_tilt_deg: f32,
    /// Maximum vertical climb rate in m/s.
    pub max_climb_rate_mps: f32,
    /// Dynamic base trigger threshold distance in meters.
    pub trigger_base_m: f32,
    /// Ground safety margin altitude in meters.
    pub ground_safety_margin_m: f32,
    /// Overhead ceiling safety margin in meters.
    pub ceiling_safety_margin_m: f32,
    /// Maximum escape thrust augmentation factor.
    pub vertical_escape_thrust_factor: f32,
}

impl Default for ArbiterConfig {
    fn default() -> Self {
        Self {
            mass_kg: 1.5,
            max_tilt_deg: 35.0,
            max_climb_rate_mps: 6.0,
            trigger_base_m: 2.5,
            ground_safety_margin_m: 1.0,
            ceiling_safety_margin_m: 1.0,
            vertical_escape_thrust_factor: 1.85,
        }
    }
}

/// Standalone, zero-allocation reference safety arbiter.
#[repr(C, align(64))]
#[derive(Clone, Copy, Debug)]
pub struct ReferenceSafetyArbiter {
    /// Active configuration parameters.
    pub config: ArbiterConfig,
    /// Latest 48-beam spatial observation mesh.
    pub latest_mesh: SphericalLidarMesh,
    /// Current estimated 6-DOF vehicle kinematic state.
    pub kinematics: KinematicState3D,
    /// Active flight mode state machine.
    pub flight_mode: FlightMode3D,
    /// Flag indicating whether intervention override is currently active.
    pub is_intervening: bool,
    /// Count of total cycles processed.
    pub total_cycles: u64,
    /// Count of cycles where intervention was triggered.
    pub intervention_cycles: u64,
}

impl Default for ReferenceSafetyArbiter {
    fn default() -> Self {
        Self::new(ArbiterConfig::default())
    }
}

impl ReferenceSafetyArbiter {
    /// Creates a new reference safety arbiter initialized with the specified configuration.
    pub const fn new(config: ArbiterConfig) -> Self {
        Self {
            config,
            latest_mesh: SphericalLidarMesh::new_empty(),
            kinematics: KinematicState3D {
                position: Vector3::ZERO,
                velocity: Vector3::ZERO,
                attitude_euler: Vector3::ZERO,
                angular_velocity: Vector3::ZERO,
                mass_kg: 1.5,
                altitude_agl_m: 5.0,
            },
            flight_mode: FlightMode3D::Nominal,
            is_intervening: false,
            total_cycles: 0,
            intervention_cycles: 0,
        }
    }

    /// Computes dynamic speed-dependent braking and evasion distance.
    #[inline(always)]
    pub fn dynamic_trigger_distance(&self) -> f32 {
        let speed = self.kinematics.velocity.norm();
        // Dynamic expansion: d_trigger = d_base + (v * 0.4s reaction horizon)
        self.config.trigger_base_m + (speed * 0.40)
    }
}

impl SensorIngestion for ReferenceSafetyArbiter {
    fn ingest_spherical_mesh(&mut self, mesh: &SphericalLidarMesh) -> Result<(), SensorError> {
        if mesh.fault_flag {
            return Err(SensorError::DataCorrupted);
        }
        self.latest_mesh = *mesh;
        Ok(())
    }

    fn ingest_raw_ranges(
        &mut self,
        ranges: &[f32; TOTAL_3D_BEAMS],
        timestamp_ns: u64,
    ) -> Result<(), SensorError> {
        self.latest_mesh.update_ranges(ranges, timestamp_ns);
        Ok(())
    }

    fn update_kinematics(&mut self, state: &KinematicState3D) -> Result<(), SensorError> {
        self.kinematics = *state;
        Ok(())
    }

    fn health_status(&self) -> SensorHealth {
        if self.latest_mesh.fault_flag {
            SensorHealth::Degraded
        } else {
            SensorHealth::Healthy
        }
    }

    fn latest_mesh(&self) -> &SphericalLidarMesh {
        &self.latest_mesh
    }
}

impl ReflexController for ReferenceSafetyArbiter {
    fn arbitrate(&mut self, nominal_cmd: &Vector3, _dt_seconds: f32) -> ArbiterDecision {
        self.total_cycles += 1;
        let d_trigger = self.dynamic_trigger_distance();

        let min_clearance = self.latest_mesh.min_clearance_m;
        let ceiling_min = self.latest_mesh.ceiling_min_m;
        let planar_min = self.latest_mesh.planar_min_m;
        let ground_min = self.latest_mesh.ground_min_m;

        let mut decision = ArbiterDecision {
            action: ReflexAction::Nominal,
            flight_mode: FlightMode3D::Nominal,
            commanded_velocity: *nominal_cmd,
            thrust_norm: 0.5,
            min_clearance_m: min_clearance,
            latency_us: 4.2, // Simulated deterministic hardware baseline
            is_intervening: false,
            anomaly_alert: false,
            timestamp_ns: self.latest_mesh.timestamp_ns,
        };

        // 1. Condition: Ground proximity hazard (Altitude AGL < ground margin OR ground beams < trigger)
        if ground_min < self.config.ground_safety_margin_m
            || self.kinematics.altitude_agl_m < self.config.ground_safety_margin_m
        {
            self.is_intervening = true;
            self.intervention_cycles += 1;
            self.flight_mode = FlightMode3D::VerticalEscape;

            decision.action = ReflexAction::VerticalEscape;
            decision.flight_mode = FlightMode3D::VerticalEscape;
            // Force climb, clamp forward velocity
            decision.commanded_velocity = Vector3::new(
                nominal_cmd.x * 0.2,
                nominal_cmd.y * 0.2,
                self.config.max_climb_rate_mps,
            );
            decision.thrust_norm = 0.85;
            decision.is_intervening = true;
            return decision;
        }

        // 2. Condition: Planar horizontal collision hazard
        if planar_min < d_trigger {
            self.is_intervening = true;
            self.intervention_cycles += 1;

            // Check if overhead ceiling is clear for vertical vault
            if ceiling_min > (self.config.ceiling_safety_margin_m + 1.5) {
                // Safe to vault upwards over the obstacle
                self.flight_mode = FlightMode3D::VerticalEscape;
                decision.action = ReflexAction::VerticalEscape;
                decision.flight_mode = FlightMode3D::VerticalEscape;
                decision.commanded_velocity = Vector3::new(
                    nominal_cmd.x * 0.1,
                    nominal_cmd.y * 0.1,
                    self.config.max_climb_rate_mps * 0.8,
                );
                decision.thrust_norm = 0.80;
                decision.is_intervening = true;
            } else {
                // Overhead is constrained: Check lateral quadrants for slalom escape
                let quads = self.latest_mesh.planar_quadrant_minimums();
                let right_clearance = quads[1];
                let left_clearance = quads[3];

                if right_clearance > left_clearance && right_clearance > d_trigger {
                    // Slalom right
                    self.flight_mode = FlightMode3D::Nominal;
                    decision.action = ReflexAction::LateralSlalom;
                    decision.commanded_velocity = Vector3::new(0.0, 3.5, 0.0);
                    decision.thrust_norm = 0.60;
                    decision.is_intervening = true;
                } else if left_clearance > d_trigger {
                    // Slalom left
                    self.flight_mode = FlightMode3D::Nominal;
                    decision.action = ReflexAction::LateralSlalom;
                    decision.commanded_velocity = Vector3::new(0.0, -3.5, 0.0);
                    decision.thrust_norm = 0.60;
                    decision.is_intervening = true;
                } else {
                    // Emergency brake in place
                    self.flight_mode = FlightMode3D::Braking;
                    decision.action = ReflexAction::EmergencyBrake;
                    decision.flight_mode = FlightMode3D::Braking;
                    decision.commanded_velocity = Vector3::ZERO;
                    decision.thrust_norm = 0.50;
                    decision.is_intervening = true;
                }
            }
            return decision;
        }

        // 3. Condition: Nominal clearance verified
        self.is_intervening = false;
        self.flight_mode = FlightMode3D::Nominal;
        decision
    }

    fn is_intervening(&self) -> bool {
        self.is_intervening
    }

    fn set_trigger_distance(&mut self, trigger_distance_m: f32) {
        self.config.trigger_base_m = trigger_distance_m;
    }

    fn reset(&mut self) {
        self.flight_mode = FlightMode3D::Nominal;
        self.is_intervening = false;
    }
}
