---
title: 5. URDF & Robot Description
---

This section details the Unified Robot Description Format (URDF) setup for the Hydrakon vehicle, including how the robot is visualized in ROS 2, how it articulates (steers and rolls), and how it bridges with the CARLA simulator.

## Overview

The robot description is located in the `hydrakon_description` package. It provides:
1.  **Visual Model:** A high-fidelity 3D representation of the car and wheels.
2.  **Kinematics:** Definitions of joints allowing wheels to spin and steer.
3.  **Simulation Bridge:** A node that connects CARLA's physics to the URDF visualization.

## URDF Structure (`vehicle.urdf.xacro`)

The core description is an Xacro file: `src/hydrakon_description/urdf/vehicle.urdf.xacro`.

### 1. Base Link (Chassis)
The `base_link` represents the vehicle's body.
*   **Mesh:** Uses `ads_dv_body.stl`.
*   **Orientation Fix:** The visual mesh is rotated by **90 degrees (yaw)** to align the car body with the standard ROS Forward X-axis.
*   **Color:** Defined as White (`rgba="1.0 1.0 1.0 1.0"`).
*   **Collision:** A simplified box geometry is used for collision calculations.

### 2. Wheels
The wheels are defined using a reusable Xacro macro.
*   **Mesh:** Uses `ads_dv_wheel.stl`.
*   **Centering:** The wheel mesh was exported with its pivot point at (0,0,0) to ensure correct rotation. In the URDF, it is rotated 90 degrees to align its rolling axis with the joint's Y-axis.
*   **Steering Logic:**
    *   **Front Wheels:** Uses a `revolute` joint (`_steering_joint`) for steering (Z-axis rotation) connected to a `continuous` joint (`_wheel_joint`) for rolling.
    *   **Rear Wheels:** Uses only a `continuous` joint (`_wheel_joint`) for rolling.
*   **Placement:** Parameters `wheelbase_x` and `track_width_y` control the physical spacing.

### 3. Vertical Offset (`robot_z_offset`)
To position the car correctly relative to the ground (since the meshes might be centered differently):
*   A `base_footprint` link is defined as the root (Z=0).
*   The `base_link` is attached to `base_footprint` with a Z-offset (e.g., `5.5m` or adjusted value) to ensure the wheels sit on the ground surface.

## Launch System (`display.launch.py`)

The launch file handles the visualization pipeline:
1.  **`robot_state_publisher`**: Reads the URDF and publishes the static transforms (TF tree).
2.  **`rviz2`**: Launches the 3D visualization tool.
3.  **`carla_bridge`**: Starts the custom bridge node to animate the robot.

**Note:** The standard `joint_state_publisher_gui` is disabled in favor of the automated `carla_bridge`.

## CARLA-to-ROS Bridge

Because we are not using the full `carla-ros-bridge` stack, a custom lightweight bridge was created to animate the URDF based on CARLA physics.

**Node:** `src/hydrakon_manager/hydrakon_manager/carla_to_ros_bridge.py`

### Functionality
1.  **Connection:** Connects to the CARLA server (localhost:2000) via the Python API.
2.  **Vehicle Detection:** Automatically finds the actor tagged as `vehicle.*` (the ego vehicle).
3.  **Telemetry Loop (50Hz):**
    *   **Steering:** Reads `control.steer` (-1.0 to 1.0) and maps it to the URDF steering limit (approx ±0.6 rad). The value is inverted (`-1.0 * val`) to match the visual coordinate system.
    *   **Velocity:** Reads the vehicle's 3D velocity vector.
    *   **Wheel Spin:** Calculates the required angular velocity (`v / r`) and integrates it over time to update the wheel position.
4.  **Publishing:** Sends `sensor_msgs/JointState` messages to `/joint_states`.

## How to Run

1.  **Build the Workspace:**
    ```bash
    colcon build --packages-select hydrakon_description hydrakon_manager
    source install/setup.bash
    ```

2.  **Ensure CARLA is Running:**
    Start your CARLA simulator.

3.  **Spawn a Vehicle:**
    Run your manual control or autonomous agent script to create a vehicle in CARLA.
    ```bash
    ros2 run hydrakon_manager manual_control
    ```

4.  **Launch Visualization:**
    ```bash
    ros2 launch hydrakon_description display.launch.py
    ```

### Troubleshooting
*   **Car "floating" too high:** Check the `robot_z_offset` in `vehicle.urdf.xacro`.
*   **Wheels spinning wrong way:** Check the steering inversion logic in `carla_to_ros_bridge.py`.
*   **"Vehicle not found":** Ensure the bridge node is running *after* the vehicle has been spawned in CARLA.
