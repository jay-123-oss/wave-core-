//! Integration & Robust Type System Test Suite for Wave-Core Public SDK.

use wave_core_api::arbiter::ReferenceSafetyArbiter;
use wave_core_api::c_abi::{wave_core_api_free, wave_core_api_init, wave_core_api_step, WaveApiConfig};
use wave_core_api::traits::{ReflexController, SensorIngestion};
use wave_core_api::types::{
    BeamChannel, KinematicState3D, ReflexAction, SphericalLidarMesh, Vector3,
    CEILING_BEAMS, GROUND_BEAMS, PLANAR_BEAMS, TOTAL_3D_BEAMS,
};

#[test]
fn test_48_beam_spherical_geometry_and_alignment() {
    assert_eq!(CEILING_BEAMS, 8);
    assert_eq!(PLANAR_BEAMS, 32);
    assert_eq!(GROUND_BEAMS, 8);
    assert_eq!(TOTAL_3D_BEAMS, 48);

    // Verify 64-byte CPU cache-line alignment
    assert_eq!(core::mem::align_of::<SphericalLidarMesh>(), 64);

    let mesh = SphericalLidarMesh::new_empty();
    assert_eq!(mesh.beams.len(), 48);

    // Check ceiling beams (+45 deg elevation)
    for i in 0..8 {
        assert_eq!(mesh.beams[i].channel, BeamChannel::Ceiling);
        assert!((mesh.beams[i].elevation_rad - core::f32::consts::FRAC_PI_4).abs() < 1e-4);
    }

    // Check planar beams (0 deg elevation)
    for i in 8..40 {
        assert_eq!(mesh.beams[i].channel, BeamChannel::Planar);
        assert_eq!(mesh.beams[i].elevation_rad, 0.0);
    }

    // Check ground beams (-45 deg elevation)
    for i in 40..48 {
        assert_eq!(mesh.beams[i].channel, BeamChannel::Ground);
        assert!((mesh.beams[i].elevation_rad - (-core::f32::consts::FRAC_PI_4)).abs() < 1e-4);
    }
}

#[test]
fn test_nominal_flight_when_clear() {
    let mut arbiter = ReferenceSafetyArbiter::default();
    let nominal_cmd = Vector3::new(10.0, 0.0, 0.0);

    // Clear airspace: all ranges 25 meters
    let ranges = [25.0f32; 48];
    assert!(arbiter.ingest_raw_ranges(&ranges, 1000).is_ok());

    let decision = arbiter.arbitrate(&nominal_cmd, 0.001);

    assert_eq!(decision.action, ReflexAction::Nominal);
    assert!(!decision.is_intervening);
    assert_eq!(decision.commanded_velocity.x, 10.0);
    assert_eq!(decision.commanded_velocity.z, 0.0);
}

#[test]
fn test_vertical_escape_on_wall_approach() {
    let mut arbiter = ReferenceSafetyArbiter::default();
    let nominal_cmd = Vector3::new(12.0, 0.0, 0.0);

    // Obstacle 1.2 meters directly ahead in horizontal plane, ceiling clear (25m)
    let mut ranges = [25.0f32; 48];
    for b in 8..16 {
        ranges[b] = 1.2; // Front planar beams blocked
    }

    assert!(arbiter.ingest_raw_ranges(&ranges, 2000).is_ok());
    let decision = arbiter.arbitrate(&nominal_cmd, 0.001);

    assert_eq!(decision.action, ReflexAction::VerticalEscape);
    assert!(decision.is_intervening);
    // Vertical climb velocity must be actively engaged
    assert!(decision.commanded_velocity.z > 0.0);
    assert!(decision.thrust_norm > 0.7);
}

#[test]
fn test_ground_dive_pull_up_failsafe() {
    let mut arbiter = ReferenceSafetyArbiter::default();
    let nominal_cmd = Vector3::new(8.0, 0.0, -2.0); // Commanded descent

    // Altitude below ground safety margin (0.5m)
    let mut kin = KinematicState3D::default();
    kin.altitude_agl_m = 0.5;
    assert!(arbiter.update_kinematics(&kin).is_ok());

    let ranges = [25.0f32; 48];
    assert!(arbiter.ingest_raw_ranges(&ranges, 3000).is_ok());

    let decision = arbiter.arbitrate(&nominal_cmd, 0.001);

    assert_eq!(decision.action, ReflexAction::VerticalEscape);
    assert!(decision.is_intervening);
    assert!(decision.commanded_velocity.z > 4.0); // Full climb thrust
}

#[test]
fn test_c_abi_lifecycle() {
    let cfg = WaveApiConfig::default();
    let ctx = wave_core_api_init(&cfg);
    assert!(!ctx.is_null());

    let ranges = [25.0f32; 48];
    let output = wave_core_api_step(ctx, ranges.as_ptr(), 48, 5.0, 0.0, 0.0, 0.001);

    assert_eq!(output.action_id, 0); // Nominal
    assert_eq!(output.is_intervening, 0);
    assert_eq!(output.cmd_vx, 5.0);

    wave_core_api_free(ctx);
}
