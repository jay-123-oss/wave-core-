//! Robust, Cache-Aligned Types & 48-Beam Spherical Coordinates.
//!
//! Provides `#![no_std]` deterministic memory layouts for avionics and robotics.
//! Designed for zero-heap, zero-allocation runtime performance within L1 cache lines (64 bytes).

/// Number of upward-angled LIDAR beams monitoring ceilings, overhead wires, and overhanging structures (+45 deg).
pub const CEILING_BEAMS: usize = 8;

/// Number of planar horizontal LIDAR beams monitoring 360-degree surroundings (0 deg elevation, 11.25 deg azimuth step).
pub const PLANAR_BEAMS: usize = 32;

/// Number of downward-angled LIDAR beams monitoring ground proximity, terrain, and landing hazards (-45 deg).
pub const GROUND_BEAMS: usize = 8;

/// Total 3D spherical/cylindrical LIDAR rays across all 3 vertical tiers.
pub const TOTAL_3D_BEAMS: usize = CEILING_BEAMS + PLANAR_BEAMS + GROUND_BEAMS; // 48 beams

/// Identification of the vertical elevation tier for spatial partitioning.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BeamChannel {
    /// Overhead ceiling tier (+45 degrees elevation).
    Ceiling = 0,
    /// Horizontal 360-degree planar tier (0 degrees elevation).
    Planar = 1,
    /// Downward ground/terrain tier (-45 degrees elevation).
    Ground = 2,
}

/// Single spherical ray measurement with exact spherical geometry and validity flags.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphericalBeam {
    /// Azimuth angle around vehicle vertical Z-axis [0.0 to 2*PI radians, ENU convention].
    pub azimuth_rad: f32,
    /// Elevation angle relative to horizontal plane [-PI/4, 0, +PI/4 radians].
    pub elevation_rad: f32,
    /// Measured radial distance to surface in meters.
    pub distance_m: f32,
    /// Signal return reflectivity / intensity [0.0 to 1.0].
    pub intensity: f32,
    /// Hardware signal validity flag (false if out-of-range, saturated, or noisy).
    pub is_valid: bool,
    /// Channel tier classification.
    pub channel: BeamChannel,
    /// Explicit 2-byte alignment padding.
    pub _pad: [u8; 2],
}

impl Default for SphericalBeam {
    fn default() -> Self {
        Self {
            azimuth_rad: 0.0,
            elevation_rad: 0.0,
            distance_m: 25.0,
            intensity: 1.0,
            is_valid: true,
            channel: BeamChannel::Planar,
            _pad: [0; 2],
        }
    }
}

/// Complete 48-Beam Spherical Volumetric Mesh.
///
/// Memory Layout:
/// - 64-byte aligned (`#[repr(C, align(64))]`) matching CPU cache line boundaries.
/// - Exactly 48 deterministic spatial beams ordered as:
///   - Indices [0..8]: Ceiling (+45 deg elevation, 45 deg azimuth spacing)
///   - Indices [8..40]: Planar (0 deg elevation, 11.25 deg azimuth spacing)
///   - Indices [40..48]: Ground (-45 deg elevation, 45 deg azimuth spacing)
/// - Fits completely into embedded SRAM and L1 data cache without dynamic memory allocation.
#[repr(C, align(64))]
#[derive(Clone, Copy, Debug)]
pub struct SphericalLidarMesh {
    /// Fixed-size array containing all 48 calibrated spatial beams.
    pub beams: [SphericalBeam; TOTAL_3D_BEAMS],
    /// Monotonic hardware ingestion timestamp in nanoseconds.
    pub timestamp_ns: u64,
    /// Minimum clearance across all valid 48 beams in meters.
    pub min_clearance_m: f32,
    /// Minimum clearance in the overhead ceiling channel (+45 deg).
    pub ceiling_min_m: f32,
    /// Minimum clearance in the horizontal planar channel (0 deg).
    pub planar_min_m: f32,
    /// Minimum clearance in the downward ground channel (-45 deg).
    pub ground_min_m: f32,
    /// Sensor fault / glitch flag (e.g. lens dirty or communication dropout).
    pub fault_flag: bool,
    /// Explicit alignment padding to ensure 64-byte structural boundary.
    pub _pad: [u8; 3],
}

impl Default for SphericalLidarMesh {
    fn default() -> Self {
        Self::new_empty()
    }
}

impl SphericalLidarMesh {
    /// Creates an empty, calibrated 48-beam spherical mesh with geometric azimuth and elevation baselines.
    pub const fn new_empty() -> Self {
        // Pre-compute spherical angles at compile-time
        let mut beams = [SphericalBeam {
            azimuth_rad: 0.0,
            elevation_rad: 0.0,
            distance_m: 25.0,
            intensity: 1.0,
            is_valid: true,
            channel: BeamChannel::Planar,
            _pad: [0; 2],
        }; TOTAL_3D_BEAMS];

        // 1. Ceiling beams: 8 beams at +45 degrees (+0.785398 rad)
        let mut i = 0;
        while i < CEILING_BEAMS {
            let az = (i as f32) * (core::f32::consts::PI / 4.0); // 45 deg step
            beams[i].azimuth_rad = az;
            beams[i].elevation_rad = core::f32::consts::FRAC_PI_4; // +45 deg
            beams[i].channel = BeamChannel::Ceiling;
            i += 1;
        }

        // 2. Planar horizontal beams: 32 beams at 0 degrees elevation (11.25 deg step)
        let mut j = 0;
        while j < PLANAR_BEAMS {
            let az = (j as f32) * (core::f32::consts::PI / 16.0); // 11.25 deg step
            beams[CEILING_BEAMS + j].azimuth_rad = az;
            beams[CEILING_BEAMS + j].elevation_rad = 0.0;
            beams[CEILING_BEAMS + j].channel = BeamChannel::Planar;
            j += 1;
        }

        // 3. Ground beams: 8 beams at -45 degrees (-0.785398 rad)
        let mut k = 0;
        while k < GROUND_BEAMS {
            let az = (k as f32) * (core::f32::consts::PI / 4.0); // 45 deg step
            beams[CEILING_BEAMS + PLANAR_BEAMS + k].azimuth_rad = az;
            beams[CEILING_BEAMS + PLANAR_BEAMS + k].elevation_rad = -core::f32::consts::FRAC_PI_4; // -45 deg
            beams[CEILING_BEAMS + PLANAR_BEAMS + k].channel = BeamChannel::Ground;
            k += 1;
        }

        Self {
            beams,
            timestamp_ns: 0,
            min_clearance_m: 25.0,
            ceiling_min_m: 25.0,
            planar_min_m: 25.0,
            ground_min_m: 25.0,
            fault_flag: false,
            _pad: [0; 3],
        }
    }

    /// Ingests an array of 48 raw scalar distance ranges (meters) and updates tiered spatial minimums.
    pub fn update_ranges(&mut self, ranges: &[f32; TOTAL_3D_BEAMS], timestamp_ns: u64) {
        self.timestamp_ns = timestamp_ns;
        let mut min_all = f32::MAX;
        let mut min_ceil = f32::MAX;
        let mut min_plan = f32::MAX;
        let mut min_grnd = f32::MAX;

        for (i, &r) in ranges.iter().enumerate() {
            let clamped_r = if r < 0.05 { 0.05 } else if r > 40.0 { 40.0 } else { r };
            self.beams[i].distance_m = clamped_r;
            self.beams[i].is_valid = r > 0.05 && r < 38.0;

            if clamped_r < min_all {
                min_all = clamped_r;
            }

            match self.beams[i].channel {
                BeamChannel::Ceiling => {
                    if clamped_r < min_ceil { min_ceil = clamped_r; }
                }
                BeamChannel::Planar => {
                    if clamped_r < min_plan { min_plan = clamped_r; }
                }
                BeamChannel::Ground => {
                    if clamped_r < min_grnd { min_grnd = clamped_r; }
                }
            }
        }

        self.min_clearance_m = min_all;
        self.ceiling_min_m = min_ceil;
        self.planar_min_m = min_plan;
        self.ground_min_m = min_grnd;
    }

    /// Computes sector obstacle density for 4 horizontal quadrants: [Front, Right, Rear, Left].
    pub fn planar_quadrant_minimums(&self) -> [f32; 4] {
        let mut quads = [f32::MAX; 4];
        // 32 planar beams: 8 per quadrant
        for q in 0..4 {
            for b in 0..8 {
                let idx = CEILING_BEAMS + (q * 8) + b;
                let d = self.beams[idx].distance_m;
                if d < quads[q] {
                    quads[q] = d;
                }
            }
        }
        quads
    }
}

/// Standard 3D Cartesian vector with zero-overhead layout.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline(always)]
    pub fn norm_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    #[inline(always)]
    pub fn norm(&self) -> f32 {
        #[cfg(feature = "std")]
        { self.norm_squared().sqrt() }
        #[cfg(not(feature = "std"))]
        { libm::sqrtf(self.norm_squared()) }
    }
}

/// Unit Quaternion representation for 3D body orientation.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }
}

/// 6-DOF Complete Vehicle Kinematics State.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KinematicState3D {
    /// Cartesian position in world frame (East-North-Up in meters).
    pub position: Vector3,
    /// Linear velocity in world frame (m/s).
    pub velocity: Vector3,
    /// Euler attitude angles: roll, pitch, yaw in radians.
    pub attitude_euler: Vector3,
    /// Body-frame angular velocity (rad/s).
    pub angular_velocity: Vector3,
    /// Vehicle total mass in kg.
    pub mass_kg: f32,
    /// Ground altitude AGL (Above Ground Level) in meters.
    pub altitude_agl_m: f32,
}

/// Action identifier representing deterministic safety arbiter decisions.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReflexAction {
    /// Nominal passthrough: Autonomous or pilot trajectory is safe.
    Nominal = 0,
    /// Urgent vertical escape: Full climb thrust to clear obstacles.
    VerticalEscape = 1,
    /// Smooth collective climb: Coordinated altitude augmentation.
    CollectiveClimb = 2,
    /// Immediate emergency brake: Maximum deceleration vector clamp.
    EmergencyBrake = 3,
    /// Lateral slalom: High-g horizontal diversion around thin obstacles.
    LateralSlalom = 4,
}

/// Current active flight mode state.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightMode3D {
    /// Normal flight tracking nominal trajectory commands.
    Nominal = 0,
    /// Active vertical avoidance override engaged.
    VerticalEscape = 1,
    /// Emergency failsafe clamp engaged.
    VerticalFailsafe = 2,
    /// Kinetic braking and speed decay engaged.
    Braking = 3,
}

/// Output decision packet returned by the safety arbiter on every cycle.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArbiterDecision {
    /// High-level reflex action.
    pub action: ReflexAction,
    /// Active flight mode state.
    pub flight_mode: FlightMode3D,
    /// Sanitized 3D linear velocity command dispatched to flight controller.
    pub commanded_velocity: Vector3,
    /// Normalized motor thrust command [0.0 to 1.0].
    pub thrust_norm: f32,
    /// Minimum clearance observed across all 48 spatial beams.
    pub min_clearance_m: f32,
    /// End-to-end reflex decision latency in microseconds.
    pub latency_us: f32,
    /// True if the safety arbiter intervened and overrode pilot/autopilot command.
    pub is_intervening: bool,
    /// True if an anomaly, sensor freeze, or hardware degradation was flagged.
    pub anomaly_alert: bool,
    /// Timestamp of intervention dispatch (monotonic nanoseconds).
    pub timestamp_ns: u64,
}

impl Default for ArbiterDecision {
    fn default() -> Self {
        Self {
            action: ReflexAction::Nominal,
            flight_mode: FlightMode3D::Nominal,
            commanded_velocity: Vector3::ZERO,
            thrust_norm: 0.5,
            min_clearance_m: 25.0,
            latency_us: 4.8,
            is_intervening: false,
            anomaly_alert: false,
            timestamp_ns: 0,
        }
    }
}
