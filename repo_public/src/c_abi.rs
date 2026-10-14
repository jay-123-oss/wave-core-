//! Zero-Copy C-ABI & FFI Integration Layer.
//!
//! Exposes `#![no_std]` C-compatible functions and structs for integration into:
//! - ROS2 nodes (`rclcpp`, `rclpy`, micro-ROS)
//! - PX4 / ArduPilot middleware
//! - C/C++ avionics controllers

use crate::arbiter::{ArbiterConfig, ReferenceSafetyArbiter};
use crate::traits::{ReflexController, SensorIngestion};
use crate::types::{ArbiterDecision, Vector3, TOTAL_3D_BEAMS};

/// C-compatible representation of arbiter configuration.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct WaveApiConfig {
    pub mass_kg: f32,
    pub max_tilt_deg: f32,
    pub max_climb_rate_mps: f32,
    pub trigger_base_m: f32,
    pub ground_safety_margin_m: f32,
    pub ceiling_safety_margin_m: f32,
    pub vertical_escape_thrust_factor: f32,
}

impl Default for WaveApiConfig {
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

/// C-compatible output packet returned across the FFI boundary.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WaveApiOutput {
    pub action_id: u32,
    pub flight_mode: u32,
    pub cmd_vx: f32,
    pub cmd_vy: f32,
    pub cmd_vz: f32,
    pub thrust_norm: f32,
    pub min_clearance_m: f32,
    pub latency_us: f32,
    pub is_intervening: u8,
    pub anomaly_alert: u8,
    pub _pad: [u8; 2],
}

impl From<ArbiterDecision> for WaveApiOutput {
    fn from(d: ArbiterDecision) -> Self {
        Self {
            action_id: d.action as u32,
            flight_mode: d.flight_mode as u32,
            cmd_vx: d.commanded_velocity.x,
            cmd_vy: d.commanded_velocity.y,
            cmd_vz: d.commanded_velocity.z,
            thrust_norm: d.thrust_norm,
            min_clearance_m: d.min_clearance_m,
            latency_us: d.latency_us,
            is_intervening: if d.is_intervening { 1 } else { 0 },
            anomaly_alert: if d.anomaly_alert { 1 } else { 0 },
            _pad: [0; 2],
        }
    }
}

/// Initializes a new safety arbiter context on the heap or returns null.
#[no_mangle]
pub extern "C" fn wave_core_api_init(config: *const WaveApiConfig) -> *mut ReferenceSafetyArbiter {
    let cfg = if config.is_null() {
        ArbiterConfig::default()
    } else {
        let c = unsafe { &*config };
        ArbiterConfig {
            mass_kg: c.mass_kg,
            max_tilt_deg: c.max_tilt_deg,
            max_climb_rate_mps: c.max_climb_rate_mps,
            trigger_base_m: c.trigger_base_m,
            ground_safety_margin_m: c.ground_safety_margin_m,
            ceiling_safety_margin_m: c.ceiling_safety_margin_m,
            vertical_escape_thrust_factor: c.vertical_escape_thrust_factor,
        }
    };

    #[cfg(feature = "std")]
    {
        Box::into_raw(Box::new(ReferenceSafetyArbiter::new(cfg)))
    }
    #[cfg(not(feature = "std"))]
    {
        core::ptr::null_mut()
    }
}

/// Processes a 48-beam range scan and executes safety arbitration in sub-millisecond time.
#[no_mangle]
pub extern "C" fn wave_core_api_step(
    ctx: *mut ReferenceSafetyArbiter,
    ranges_ptr: *const f32,
    ranges_len: usize,
    nominal_vx: f32,
    nominal_vy: f32,
    nominal_vz: f32,
    dt_sec: f32,
) -> WaveApiOutput {
    if ctx.is_null() || ranges_ptr.is_null() || ranges_len != TOTAL_3D_BEAMS {
        return WaveApiOutput::default();
    }

    let arbiter = unsafe { &mut *ctx };
    let ranges_slice = unsafe { core::slice::from_raw_parts(ranges_ptr, TOTAL_3D_BEAMS) };
    let mut ranges_array = [0.0f32; TOTAL_3D_BEAMS];
    ranges_array.copy_from_slice(ranges_slice);

    let _ = arbiter.ingest_raw_ranges(&ranges_array, 0);
    let nominal_cmd = Vector3::new(nominal_vx, nominal_vy, nominal_vz);
    let decision = arbiter.arbitrate(&nominal_cmd, dt_sec);

    decision.into()
}

/// Frees an arbiter context previously allocated with `wave_core_api_init`.
#[no_mangle]
pub extern "C" fn wave_core_api_free(ctx: *mut ReferenceSafetyArbiter) {
    if !ctx.is_null() {
        #[cfg(feature = "std")]
        unsafe {
            drop(Box::from_raw(ctx));
        }
    }
}
