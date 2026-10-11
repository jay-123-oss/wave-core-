/**
 * @file wave_core_robotics.h
 * @brief Standard C/C++ API for Wave-Core Neuromorphic Robotics Reflex Engine.
 *
 * Compatible with ROS2 (rclcpp / micro-ROS), C, and C++ autonomous robot stacks.
 * Zero-copy stack execution in < 3 microseconds per frame.
 */

#ifndef WAVE_CORE_ROBOTICS_H
#define WAVE_CORE_ROBOTICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WAVE_CORE_TOTAL_LIDAR_BEAMS 32

/**
 * @brief 6-DOF velocity command mirroring geometry_msgs/msg/Twist (/cmd_vel).
 */
typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
} WaveCoreTwist;

/**
 * @brief 2D vehicle kinematic state for closed-loop navigation.
 */
typedef struct {
    float x;
    float y;
    float vx;
    float vy;
    float heading;
    float omega;
} WaveCoreKinematicState;

/**
 * @brief Reflex decision packet mirroring wave_core_msgs/msg/ReflexStatus.
 */
typedef struct {
    uint32_t action;                 /**< 0=None, 1=EmergencyBrake, 2=SlowDown, 3=SteerLeft, 4=SteerRight, 5=ReverseEscape, 6=Rotate180 */
    uint32_t steering;               /**< 0=MaintainCourse, 1=SteerLeft, 2=SteerRight, 3=ReverseEscape, 4=Rotate180 */
    bool     is_coherent;            /**< True if temporal phase coherence is locked onto a real hazard */
    uint32_t dist_t;                 /**< Current Hamming distance from hazard prototype */
    uint32_t tau;                    /**< Speed-scaled dynamic Hamming threshold */
    bool     thin_obstacle_detected; /**< Isolated thin pole/wire flag */
    bool     anomaly_detected;       /**< Anomaly / sensor glitch flag */
    bool     sensor_frozen;          /**< Hardware watchdog sensor freeze flag */
    float    front_min_distance;     /**< Closest obstacle in front quadrant (m) */
    float    front_avg_distance;     /**< Average obstacle distance in front (m) */
    float    right_min_distance;     /**< Closest obstacle on right (m) */
    float    left_min_distance;      /**< Closest obstacle on left (m) */
    float    back_min_distance;      /**< Closest obstacle behind (m) */
} WaveCoreDecision;

/**
 * @brief Downsamples arbitrary ROS2 LaserScan (e.g. 360, 720, 1081 beams) into 32 polar rays.
 *
 * Employs conservative minimum pooling across angular sectors to ensure thin wires/poles are preserved.
 *
 * @param raw_ranges Pointer to raw float array from sensor_msgs/LaserScan.
 * @param count Number of range measurements.
 * @param angle_min Start angle in radians.
 * @param angle_increment Angular step in radians.
 * @param out_32_rays Destination buffer of size 32.
 * @return 0 on success, negative error code on failure.
 */
int32_t wave_core_resample_laserscan(
    const float* raw_ranges,
    size_t count,
    float angle_min,
    float angle_increment,
    float* out_32_rays
);

/**
 * @brief Processes 32 polar rays through Wave-Core ReflexEngine.
 *
 * @param engine_ptr Opaque handle to ReflexEngine.
 * @param rays_32 Array of exactly 32 float distance readings (0.05m to 10.0m).
 * @param speed_m_s Current vehicle forward speed in m/s.
 * @param out_decision Pointer to output decision struct.
 * @return 0 on success, negative error code on failure.
 */
int32_t wave_core_process_rays(
    void* engine_ptr,
    const float* rays_32,
    float speed_m_s,
    WaveCoreDecision* out_decision
);

/**
 * @brief Executes closed-loop navigation step to generate /cmd_vel Twist output.
 *
 * @param ctrl_ptr Opaque handle to ClosedLoopController.
 * @param rays_32 Array of 32 float distances.
 * @param state Vehicle current odometry state.
 * @param goal_x Target navigation goal X coordinate.
 * @param goal_y Target navigation goal Y coordinate.
 * @param desired_speed Target cruising speed in m/s.
 * @param out_twist Pointer to output geometry_msgs/Twist command.
 * @param out_decision Optional pointer to receive reflex decision (can be NULL).
 * @return 0 on success, negative error code on failure.
 */
int32_t wave_core_step_controller(
    void* ctrl_ptr,
    const float* rays_32,
    const WaveCoreKinematicState* state,
    float goal_x,
    float goal_y,
    float desired_speed,
    WaveCoreTwist* out_twist,
    WaveCoreDecision* out_decision
);

#define WAVE_CORE_TOTAL_3D_BEAMS 48

/**
 * @brief 3D Drone Kinematic State for 6-DOF aerial navigation.
 */
typedef struct {
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float roll;
    float pitch;
    float yaw;
    float wx;
    float wy;
    float wz;
} WaveCoreKinematicState3D;

/**
 * @brief 3D Volumetric Decision packet mirroring Arbiter3D.
 */
typedef struct {
    uint32_t mode;                   /**< 0=Nominal, 1=VerticalFailsafe, 2=VerticalEscape */
    uint32_t vertical_reflex;        /**< 0=None, 1=CollectiveClimb, 2=TerrainPullUp, 3=CeilingAvoidance, 4=VerticalClimbOverpass */
    bool     emergency_active;       /**< True if emergency evasive action is currently active */
    bool     anomaly_detected;       /**< True if NaN/Inf or sensor glitch was filtered */
    bool     sensor_frozen;          /**< True if sensor stream is frozen */
    float    ground_min_distance;    /**< Minimum downward distance (m) */
    float    ceiling_min_distance;   /**< Minimum upward distance (m) */
    float    front_min_distance;     /**< Minimum forward distance (m) */
    float    left_avg_distance;      /**< Left sector average clearance (m) */
    float    right_avg_distance;     /**< Right sector average clearance (m) */
} WaveCoreDecision3D;

/**
 * @brief Downsamples arbitrary 3D Pointcloud (x, y, z float buffer) into 48 spherical rays.
 *
 * Beams 0..7: Ceiling (+45 deg), Beams 8..39: Planar (0 deg), Beams 40..47: Ground (-45 deg).
 *
 * @param points_xyz Pointer to contiguous [x, y, z] float buffer.
 * @param num_points Number of 3D points.
 * @param out_48_rays Destination buffer of size 48.
 * @return 0 on success, negative error code on failure.
 */
int32_t wave_core_resample_pointcloud_3d(
    const float* points_xyz,
    size_t num_points,
    float* out_48_rays
);

/**
 * @brief Executes 3D volumetric navigation step to generate 6-DOF /cmd_vel Twist output.
 *
 * @param engine_ptr Opaque handle to ReflexEngine3D.
 * @param rays_48 Array of 48 float distances.
 * @param state 3D drone odometry state.
 * @param goal_x Target 3D goal X.
 * @param goal_y Target 3D goal Y.
 * @param goal_z Target 3D goal Z (altitude).
 * @param desired_speed Cruise speed in m/s.
 * @param out_twist Output 6-DOF velocity command (linear_x, linear_y, linear_z, angular_z).
 * @param out_decision Optional pointer to receive 3D decision telemetry.
 * @return 0 on success, negative error code on failure.
 */
int32_t wave_core_step_controller_3d(
    void* engine_ptr,
    const float* rays_48,
    const WaveCoreKinematicState3D* state,
    float goal_x,
    float goal_y,
    float goal_z,
    float desired_speed,
    WaveCoreTwist* out_twist,
    WaveCoreDecision3D* out_decision
);

/**
 * @brief Converts 6-DOF velocity command into MAVLink NED velocity frame.
 */
int32_t wave_core_decision_to_mavlink_ned(
    float command_vx,
    float command_vy,
    float command_vz,
    float command_yaw_rate,
    float* out_ned_vx,
    float* out_ned_vy,
    float* out_ned_vz,
    float* out_ned_yaw_rate
);

void* wave_core_engine_3d_create(void);
void  wave_core_engine_3d_free(void* ptr);

/**
 * @brief Multi-Sensor Fusion & Aerodynamic Disturbance Observer telemetry.
 */
typedef struct {
    float pos_ned[3];
    float vel_ned[3];
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float gyro[3];
    float accel[3];
    float wind_force_est[3];
    float downwash_factor;
    uint64_t last_timestamp_us;
} WaveCoreSensorFusion;

void* wave_core_autopilot_create(void);
void  wave_core_autopilot_free(void* ptr);

size_t wave_core_autopilot_ingest_mavlink(
    void* autopilot_ptr,
    const uint8_t* stream_ptr,
    size_t stream_len
);

int32_t wave_core_autopilot_update_imu(
    void* autopilot_ptr,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float dt
);

int32_t wave_core_autopilot_step(
    void* autopilot_ptr,
    const float* points_xyz,
    size_t num_points,
    float goal_ned_x,
    float goal_ned_y,
    float goal_ned_z,
    float desired_speed,
    WaveCoreTwist* out_twist,
    WaveCoreDecision3D* out_decision,
    uint8_t* out_target_mavlink,
    size_t* out_target_len
);

int32_t wave_core_autopilot_get_fusion_state(
    const void* autopilot_ptr,
    WaveCoreSensorFusion* out_fusion
);

#ifdef __cplusplus
}
#endif

#endif /* WAVE_CORE_ROBOTICS_H */
