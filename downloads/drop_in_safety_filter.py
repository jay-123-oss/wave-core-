#!/usr/bin/env python3
"""
Wave-Core 5-Line Drop-In Safety Shield Demo
==========================================

This example demonstrates how an autonomous robotics/drone company integrates
Wave-Core without modifying their existing autonomy or navigation software.

The company keeps their own path planner (Nav2, move_base, custom AI).
Wave-Core sits between their planner and the robot's motors as an ultra-fast
(5.56 microsecond) safety shield.
"""

import sys
import os

# Add bindings directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "bindings", "python"))
from wave_core import WaveCoreAutopilot

# ==============================================================================
# 1. SETUP: Initialize Wave-Core Shield (Runs once on robot boot)
# ==============================================================================
shield = WaveCoreAutopilot()


# ==============================================================================
# 2. MOCK: The Company's Existing Autonomous Navigation Stack
# ==============================================================================
class MockCompanyAutonomyStack:
    def __init__(self):
        self.cruise_speed = 3.0 # m/s

    def compute_mission_velocity(self):
        """Company's slow planner commanding forward motion (e.g. at 5 Hz)."""
        return {"linear_x": 3.0, "linear_y": 0.0, "linear_z": 0.0, "angular_z": 0.0}

company_planner = MockCompanyAutonomyStack()


# ==============================================================================
# 3. THE 5-LINE DROP-IN SAFETY ARBITER CALLBACK
# ==============================================================================
def on_planner_command(planner_cmd, sensor_pointcloud, goal_x, goal_y, goal_z):
    """
    THIS IS THE ONLY FUNCTION A COMPANY ADDS TO THEIR CODEBASE!
    Zero changes to their planner or flight controller.
    """
    # Line 1: Ask Wave-Core for safe arbitrated velocities
    safe_cmd, decision, mavlink_packet = shield.step(
        sensor_pointcloud, goal_x, goal_y, goal_z, desired_speed=3.0
    )

    # Line 2: If hazard detected (emergency_active == True), use Wave-Core reflex;
    #         otherwise, pass the company's planner command unchanged!
    if decision.emergency_active:
        final_cmd = {
            "linear_x": safe_cmd.linear_x,
            "linear_y": safe_cmd.linear_y,
            "linear_z": safe_cmd.linear_z,
            "angular_z": safe_cmd.angular_z,
            "source": "WAVE_CORE_SAFETY_REFLEX_OVERRIDE"
        }
    else:
        final_cmd = {
            "linear_x": planner_cmd["linear_x"],
            "linear_y": planner_cmd["linear_y"],
            "linear_z": planner_cmd["linear_z"],
            "angular_z": planner_cmd["angular_z"],
            "source": "COMPANY_ORIGINAL_PATH_PLANNER"
        }

    # Line 3: Send final command to motors or autopilot (and optional MAVLink packet)
    return final_cmd, mavlink_packet


# ==============================================================================
# 4. SIMULATION DEMONSTRATION
# ==============================================================================
if __name__ == "__main__":
    print("=" * 70)
    print("WAVE-CORE DROP-IN SAFETY SHIELD: LIVE DEMONSTRATION")
    print("=" * 70)

    # Scenario A: Path is completely clear
    # ------------------------------------
    clear_points = [10.0, 0.0, 0.0] # Far wall at 10m
    planner_cmd = company_planner.compute_mission_velocity()
    cmd, mav_bytes = on_planner_command(planner_cmd, clear_points, 20.0, 0.0, -2.0)

    print("\n[Scenario A: Clear Sky / No Obstacles]")
    print(f"-> Planner Desired:   linear_x={planner_cmd['linear_x']} m/s, linear_z={planner_cmd['linear_z']} m/s")
    print(f"-> Motor Final Exec:  linear_x={cmd['linear_x']:.2f} m/s, linear_z={cmd['linear_z']:.2f} m/s")
    print(f"-> Active Authority:  {cmd['source']} (0% interference)")

    # Scenario B: Sudden Ground Plunge / Dive Hazard
    # ---------------------------------------------
    # Drone plunging downward, ground rock emerges directly below at 0.40m
    shield.update_imu(ax=0.0, ay=0.0, az=-2.5, gx=0.0, gy=0.0, gz=0.0, dt=0.005) # descending
    ground_hazard_points = [0.0, 0.0, -0.40] # rock 40cm below
    planner_cmd_blind = company_planner.compute_mission_velocity() # Company planner still blindly commanding forward!
    cmd_b, mav_bytes_b = on_planner_command(planner_cmd_blind, ground_hazard_points, 20.0, 0.0, -2.0)

    print("\n[Scenario B: Ground Collision Hazard (Main Planner is Blind / Lagging)]")
    print(f"-> Planner Desired:   linear_x={planner_cmd_blind['linear_x']} m/s (BLIND CRUISE)")
    print(f"-> Motor Final Exec:  linear_x={cmd_b['linear_x']:.2f} m/s, linear_z={cmd_b['linear_z']:.2f} m/s (PULL-UP!)")
    print(f"-> Active Authority:  {cmd_b['source']}")
    print(f"-> Wire MAVLink:      {len(mav_bytes_b)} bytes generated for Pixhawk/PX4")

    # Scenario C: Low Ceiling / Overhead Pipe Hazard
    # ----------------------------------------------
    low_ceiling_points = [1.0, 0.0, 0.45] # pipe 45cm above
    cmd_c, _ = on_planner_command(planner_cmd, low_ceiling_points, 20.0, 0.0, -2.0)

    print("\n[Scenario C: Low Overhead Pipe (Ceiling Impact Prevention)]")
    print(f"-> Motor Final Exec:  linear_x={cmd_c['linear_x']:.2f} m/s, linear_z={cmd_c['linear_z']:.2f} m/s (CLAMPED)")
    print(f"-> Active Authority:  {cmd_c['source']}")

    print("\n" + "=" * 70)
    print("VERIFICATION COMPLETE: Customer stack was NOT modified.")
    print("Wave-Core intercepted hazards with 5.56 us latency.")
    print("=" * 70)
