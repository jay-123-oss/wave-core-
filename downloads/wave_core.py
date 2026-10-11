"""
Wave-Core Neuromorphic Robotics Python & ROS2 Binding.

Allows any Python ROS2 node (`rclpy`) or autonomy script to load and execute
Wave-Core with sub-10 microsecond latency and zero memory copy overhead.
"""

import ctypes
import os
import sys
from typing import Tuple, Optional

# C-compatible structs matching wave_core_robotics.h
class WaveCoreTwist(ctypes.Structure):
    _fields_ = [
        ("linear_x", ctypes.c_float),
        ("linear_y", ctypes.c_float),
        ("linear_z", ctypes.c_float),
        ("angular_x", ctypes.c_float),
        ("angular_y", ctypes.c_float),
        ("angular_z", ctypes.c_float),
    ]

class WaveCoreKinematicState(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("vx", ctypes.c_float),
        ("vy", ctypes.c_float),
        ("heading", ctypes.c_float),
        ("omega", ctypes.c_float),
    ]

class WaveCoreDecision(ctypes.Structure):
    _fields_ = [
        ("action", ctypes.c_uint32),
        ("steering", ctypes.c_uint32),
        ("is_coherent", ctypes.c_bool),
        ("dist_t", ctypes.c_uint32),
        ("tau", ctypes.c_uint32),
        ("thin_obstacle_detected", ctypes.c_bool),
        ("anomaly_detected", ctypes.c_bool),
        ("sensor_frozen", ctypes.c_bool),
        ("front_min_distance", ctypes.c_float),
        ("front_avg_distance", ctypes.c_float),
        ("right_min_distance", ctypes.c_float),
        ("left_min_distance", ctypes.c_float),
        ("back_min_distance", ctypes.c_float),
    ]

class WaveCoreKinematicState3D(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("vx", ctypes.c_float),
        ("vy", ctypes.c_float),
        ("vz", ctypes.c_float),
        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float),
        ("wx", ctypes.c_float),
        ("wy", ctypes.c_float),
        ("wz", ctypes.c_float),
    ]

class WaveCoreDecision3D(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_uint32),
        ("vertical_reflex", ctypes.c_uint32),
        ("emergency_active", ctypes.c_bool),
        ("anomaly_detected", ctypes.c_bool),
        ("sensor_frozen", ctypes.c_bool),
        ("ground_min_distance", ctypes.c_float),
        ("ceiling_min_distance", ctypes.c_float),
        ("front_min_distance", ctypes.c_float),
        ("left_avg_distance", ctypes.c_float),
        ("right_avg_distance", ctypes.c_float),
    ]

class WaveCoreNode:
    """High-level Python wrapper for Wave-Core Robotics Reflex Engine."""

    def __init__(self, lib_path: Optional[str] = None):
        if lib_path is None:
            # Look in target directories
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            candidates = [
                os.path.join(base_dir, "target", "release", "libwave_core_robotics.so"),
                os.path.join(base_dir, "target", "debug", "libwave_core_robotics.so"),
            ]
            for c in candidates:
                if os.path.exists(c):
                    lib_path = c
                    break

        if lib_path is None or not os.path.exists(lib_path):
            raise FileNotFoundError(f"Wave-Core shared library not found: {lib_path}")

        self._lib = ctypes.CDLL(lib_path)
        self._setup_ffi()

        self._controller = self._lib.wave_core_controller_create()
        self._engine = self._lib.wave_core_engine_create()

    def _setup_ffi(self):
        # Setup function argument and return types
        self._lib.wave_core_resample_laserscan.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_size_t,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_float),
        ]
        self._lib.wave_core_resample_laserscan.restype = ctypes.c_int32

        self._lib.wave_core_process_rays.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_float,
            ctypes.POINTER(WaveCoreDecision),
        ]
        self._lib.wave_core_process_rays.restype = ctypes.c_int32

        self._lib.wave_core_step_controller.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(WaveCoreKinematicState),
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(WaveCoreTwist),
            ctypes.POINTER(WaveCoreDecision),
        ]
        self._lib.wave_core_step_controller.restype = ctypes.c_int32

        self._lib.wave_core_engine_create.restype = ctypes.c_void_p
        self._lib.wave_core_engine_free.argtypes = [ctypes.c_void_p]

        self._lib.wave_core_controller_create.restype = ctypes.c_void_p
        self._lib.wave_core_controller_free.argtypes = [ctypes.c_void_p]

    def resample_laserscan(self, ranges: list, angle_min: float, angle_increment: float) -> list:
        """Converts arbitrary ROS2 LaserScan array into 32 polar rays."""
        c_ranges = (ctypes.c_float * len(ranges))(*ranges)
        out_32 = (ctypes.c_float * 32)()

        res = self._lib.wave_core_resample_laserscan(
            c_ranges, len(ranges), angle_min, angle_increment, out_32
        )
        if res != 0:
            raise RuntimeError(f"Resampling failed with code {res}")

        return list(out_32)

    def process_frame(self, rays_32: list, speed_m_s: float) -> WaveCoreDecision:
        """Processes 32 rays and outputs reflex decision."""
        c_rays = (ctypes.c_float * 32)(*rays_32)
        decision = WaveCoreDecision()

        res = self._lib.wave_core_process_rays(
            self._engine, c_rays, speed_m_s, ctypes.byref(decision)
        )
        if res != 0:
            raise RuntimeError(f"Processing failed with code {res}")

        return decision

    def compute_cmd_vel(
        self,
        rays_32: list,
        x: float, y: float, vx: float, vy: float, heading: float, omega: float,
        goal_x: float, goal_y: float, desired_speed: float
    ) -> Tuple[WaveCoreTwist, WaveCoreDecision]:
        """Computes /cmd_vel Twist message from 32 rays, vehicle state, and goal."""
        c_rays = (ctypes.c_float * 32)(*rays_32)
        state = WaveCoreKinematicState(x, y, vx, vy, heading, omega)
        twist = WaveCoreTwist()
        decision = WaveCoreDecision()

        res = self._lib.wave_core_step_controller(
            self._controller,
            c_rays,
            ctypes.byref(state),
            goal_x,
            goal_y,
            desired_speed,
            ctypes.byref(twist),
            ctypes.byref(decision),
        )
        if res != 0:
            raise RuntimeError(f"Controller step failed with code {res}")

        return twist, decision

    def __del__(self):
        if hasattr(self, "_engine") and self._engine:
            self._lib.wave_core_engine_free(self._engine)
        if hasattr(self, "_controller") and self._controller:
            self._lib.wave_core_controller_free(self._controller)


class WaveCoreNode3D:
    """High-level Python wrapper for Wave-Core 3D Volumetric Aerial Engine."""

    def __init__(self, lib_path: Optional[str] = None):
        if lib_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            candidates = [
                os.path.join(base_dir, "target", "release", "libwave_core_robotics.so"),
                os.path.join(base_dir, "target", "debug", "libwave_core_robotics.so"),
            ]
            for cand in candidates:
                if os.path.exists(cand):
                    lib_path = cand
                    break

        if lib_path is None or not os.path.exists(lib_path):
            raise FileNotFoundError("Could not locate libwave_core_robotics.so. Build with `cargo build --release` first.")

        self._lib = ctypes.CDLL(lib_path)
        self._setup_ffi()

        self._engine_3d = self._lib.wave_core_engine_3d_create()
        if not self._engine_3d:
            raise RuntimeError("Failed to allocate Wave-Core 3D Engine")

    def _setup_ffi(self):
        self._lib.wave_core_resample_pointcloud_3d.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_float),
        ]
        self._lib.wave_core_resample_pointcloud_3d.restype = ctypes.c_int32

        self._lib.wave_core_step_controller_3d.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(WaveCoreKinematicState3D),
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(WaveCoreTwist),
            ctypes.POINTER(WaveCoreDecision3D),
        ]
        self._lib.wave_core_step_controller_3d.restype = ctypes.c_int32

        self._lib.wave_core_engine_3d_create.restype = ctypes.c_void_p
        self._lib.wave_core_engine_3d_free.argtypes = [ctypes.c_void_p]

    def resample_pointcloud(self, points_xyz: list) -> list:
        """Downsamples flat [x0, y0, z0, ...] pointcloud into 48 spherical rays."""
        num_points = len(points_xyz) // 3
        c_points = (ctypes.c_float * len(points_xyz))(*points_xyz)
        out_48 = (ctypes.c_float * 48)()

        res = self._lib.wave_core_resample_pointcloud_3d(c_points, num_points, out_48)
        if res != 0:
            raise RuntimeError(f"Pointcloud resampling failed with code {res}")

        return list(out_48)

    def step_controller_3d(
        self,
        rays_48: list,
        x: float, y: float, z: float,
        vx: float, vy: float, vz: float,
        yaw: float,
        goal_x: float, goal_y: float, goal_z: float,
        desired_speed: float
    ) -> Tuple[WaveCoreTwist, WaveCoreDecision3D]:
        """Computes 6-DOF /cmd_vel Twist message from 48 rays and 3D drone state."""
        c_rays = (ctypes.c_float * 48)(*rays_48)
        state = WaveCoreKinematicState3D(x, y, z, vx, vy, vz, 0.0, 0.0, yaw, 0.0, 0.0, 0.0)
        twist = WaveCoreTwist()
        decision = WaveCoreDecision3D()

        res = self._lib.wave_core_step_controller_3d(
            self._engine_3d,
            c_rays,
            ctypes.byref(state),
            goal_x, goal_y, goal_z,
            desired_speed,
            ctypes.byref(twist),
            ctypes.byref(decision),
        )
        if res != 0:
            raise RuntimeError(f"3D controller step failed with code {res}")

        return twist, decision

    def __del__(self):
        if hasattr(self, "_engine_3d") and self._engine_3d:
            self._lib.wave_core_engine_3d_free(self._engine_3d)


class WaveCoreSensorFusion(ctypes.Structure):
    _fields_ = [
        ("pos_ned", ctypes.c_float * 3),
        ("vel_ned", ctypes.c_float * 3),
        ("quat_w", ctypes.c_float),
        ("quat_x", ctypes.c_float),
        ("quat_y", ctypes.c_float),
        ("quat_z", ctypes.c_float),
        ("gyro", ctypes.c_float * 3),
        ("accel", ctypes.c_float * 3),
        ("wind_force_est", ctypes.c_float * 3),
        ("downwash_factor", ctypes.c_float),
        ("last_timestamp_us", ctypes.c_uint64),
    ]


class WaveCoreAutopilot:
    """High-level Python wrapper for Wave-Core Unified 3D Drone Autopilot (Options 1+2+3)."""

    def __init__(self, lib_path: Optional[str] = None):
        if lib_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            candidates = [
                os.path.join(base_dir, "target", "release", "libwave_core_robotics.so"),
                os.path.join(base_dir, "target", "debug", "libwave_core_robotics.so"),
            ]
            for cand in candidates:
                if os.path.exists(cand):
                    lib_path = cand
                    break

        if lib_path is None or not os.path.exists(lib_path):
            raise FileNotFoundError("Could not locate libwave_core_robotics.so.")

        self._lib = ctypes.CDLL(lib_path)
        self._setup_ffi()

        self._ap = self._lib.wave_core_autopilot_create()
        if not self._ap:
            raise RuntimeError("Failed to allocate Wave-Core Drone Autopilot")

    def _setup_ffi(self):
        self._lib.wave_core_autopilot_create.restype = ctypes.c_void_p
        self._lib.wave_core_autopilot_free.argtypes = [ctypes.c_void_p]

        self._lib.wave_core_autopilot_ingest_mavlink.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
        ]
        self._lib.wave_core_autopilot_ingest_mavlink.restype = ctypes.c_size_t

        self._lib.wave_core_autopilot_update_imu.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.wave_core_autopilot_update_imu.restype = ctypes.c_int32

        self._lib.wave_core_autopilot_step.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_size_t,
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(WaveCoreTwist),
            ctypes.POINTER(WaveCoreDecision3D),
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_size_t),
        ]
        self._lib.wave_core_autopilot_step.restype = ctypes.c_int32

        self._lib.wave_core_autopilot_get_fusion_state.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(WaveCoreSensorFusion),
        ]
        self._lib.wave_core_autopilot_get_fusion_state.restype = ctypes.c_int32

    def update_imu(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float, dt: float):
        res = self._lib.wave_core_autopilot_update_imu(self._ap, ax, ay, az, gx, gy, gz, dt)
        if res != 0:
            raise RuntimeError(f"IMU update failed with code {res}")

    def ingest_mavlink(self, raw_bytes: bytes) -> int:
        c_buf = (ctypes.c_uint8 * len(raw_bytes))(*raw_bytes)
        return self._lib.wave_core_autopilot_ingest_mavlink(self._ap, c_buf, len(raw_bytes))

    def step(
        self,
        points_xyz: list,
        goal_ned_x: float, goal_ned_y: float, goal_ned_z: float,
        desired_speed: float
    ) -> Tuple[WaveCoreTwist, WaveCoreDecision3D, bytes]:
        num_points = len(points_xyz) // 3
        c_pts = (ctypes.c_float * len(points_xyz))(*points_xyz)
        twist = WaveCoreTwist()
        decision = WaveCoreDecision3D()
        target_buf = (ctypes.c_uint8 * 280)()
        target_len = ctypes.c_size_t(0)

        res = self._lib.wave_core_autopilot_step(
            self._ap,
            c_pts,
            num_points,
            goal_ned_x, goal_ned_y, goal_ned_z,
            desired_speed,
            ctypes.byref(twist),
            ctypes.byref(decision),
            target_buf,
            ctypes.byref(target_len),
        )
        if res != 0:
            raise RuntimeError(f"Autopilot step failed with code {res}")

        mavlink_bytes = bytes(target_buf[:target_len.value])
        return twist, decision, mavlink_bytes

    def get_fusion_state(self) -> WaveCoreSensorFusion:
        fusion = WaveCoreSensorFusion()
        res = self._lib.wave_core_autopilot_get_fusion_state(self._ap, ctypes.byref(fusion))
        if res != 0:
            raise RuntimeError("Failed to read fusion state")
        return fusion

    def __del__(self):
        if hasattr(self, "_ap") and self._ap:
            self._lib.wave_core_autopilot_free(self._ap)


