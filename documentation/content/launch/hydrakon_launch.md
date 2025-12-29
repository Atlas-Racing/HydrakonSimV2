---
title: 1. Hydrakon Launch System
---

**Source File:** `src/hydrakon_launch/launch/launch.py`

This launch file acts as the main entry point for the simulation system. It orchestrates the spawning of the vehicle, sensors (cameras), and perception nodes (cone detection and localization), as well as optional tools like manual control and system monitoring.

## Nodes

### 1. Vehicle Spawner
- **Node Name:** `carla_vehicle_spawner`
- **Package:** `hydrakon_manager`
- **Executable:** `vehicle_spawner`
- **Source:** `src/hydrakon_manager/hydrakon_manager/vehicle_node.py`
- **Description:** Initializes and spawns the ego vehicle within the CARLA simulator environment at a designated spawn point.

### 2. Manual Control
- **Node Name:** `manual_control`
- **Package:** `hydrakon_manager`
- **Executable:** `manual_control`
- **Source:** `src/hydrakon_manager/hydrakon_manager/manual_control.py`
- **Description:** Provides a Pygame-based window to manually drive the spawned vehicle.
- **Condition:** Enabled only if the `manual_control` launch argument is set to `True`.

### 3. Depth Camera Spawner
- **Node Name:** `carla_depth_camera_spawner`
- **Package:** `hydrakon_camera`
- **Executable:** `depth_camera_spawner`
- **Source:** `src/hydrakon_camera/hydrakon_camera/depth_camera.py`
- **Description:** Attaches a depth camera to the ego vehicle in CARLA and publishes depth images to ROS 2 topics.

### 4. RGB Camera Spawner
- **Node Name:** `carla_rgb_camera_spawner`
- **Package:** `hydrakon_camera`
- **Executable:** `rgb_camera_spawner`
- **Source:** `src/hydrakon_camera/hydrakon_camera/camera.py`
- **Description:** Attaches an RGB camera to the ego vehicle in CARLA and publishes standard color images.

### 5. Cone Detector
- **Node Name:** `cone_detections`
- **Package:** `hydrakon_camera`
- **Executable:** `cone_detector`
- **Source:** `src/hydrakon_camera/hydrakon_camera/YOLO.py`
- **Description:** Performs real-time object detection (specifically traffic cones) on the camera stream using a YOLO model (ONNX or PyTorch).

### 6. Cone Locator
- **Node Name:** `cone_locator`
- **Package:** `hydrakon_camera`
- **Executable:** `cone_locator`
- **Source:** `src/hydrakon_camera/hydrakon_camera/cone_locator.py`
- **Description:** Creates a local 3D position map (No mapping) of detected cones relative to the vehicle, combining 2D detections (from YOLO) with depth information (from depth camera).

### 7. R2S Gateway (TUI)
- **Node Name:** `r2s_gw`
- **Package:** `r2s_gw`
- **Executable:** `r2s_gw`
- **Source:** `src/r2s_gw/r2s_gw/main.py`
- **Description:** Launches the R2S Gateway, a Terminal User Interface (TUI) for monitoring and interacting with the ROS 2 system.

### 8. `base_link_to_cone_frame` Static Transform
- **Node Name:** `base_link_to_cone_frame`
- **Package:** `tf2_ros`
- **Executable:** `static_transform_publisher`
- **Arguments:** `0 0 0.7 0 0 0 base_link cone_frame`
- **Description:** Publishes a static transform between `base_link` and `cone_frame`. The `cone_frame` is offset by `0.5` meters along the Z-axis from `base_link` to lift the visualized cones off the ground.

### 9. RViz2
- **Node Name:** `rviz2`
- **Package:** `rviz2`
- **Executable:** `rviz2`
- **Description:** 3D visualization tool for ROS 2. Launches with a custom configuration file (`custom.rviz`) to visualize the vehicle, sensor data, and cone detections.
- **Condition:** Enabled unless the `rviz` launch argument is set to `False` (default is `True`).

## Launch Arguments

| Argument | Default Value | Description |
| :--- | :--- | :--- |
| `model_path` | `.../best.onnx` | Path to the YOLO object detection model file. |
| `benchmark` | `False` | Enable benchmarking mode for inference timing statistics. |
| `manual_control` | `False` | Set to `True` to launch the Pygame manual control window. |
| `gw` | `False` | Set to `True` to launch the Greenwave Monitor (TUI). |
| `rviz` | `True` | Set to `False` to disable RViz visualization. |
| `host` | `localhost` | IP address of the CARLA simulator (supports remote connection). |

---

# Additional Launch Files

## Mapping Lap

**Source File:** `src/hydrakon_launch/launch/mapping_lap.launch.py`

**Usage:**
```bash
ros2 launch hydrakon_launch mapping_lap.launch.py
```

This launch file is used for generating a map of the environment using SLAM (Simultaneous Localization and Mapping). It integrates LiDAR data from CARLA with the ROS 2 SLAM Toolbox.

### Key Nodes
*   **Carla Bridge:** Provides Odometry and TF from CARLA.
*   **Pointcloud to LaserScan:** Converts 3D LiDAR point clouds into 2D laser scans required by `slam_toolbox`.
*   **SLAM Toolbox:** Performs online asynchronous SLAM to build a map.
*   **Pure Pursuit:** A basic path tracking algorithm (starts after a delay).

## Navigation

**Source File:** `src/hydrakon_launch/launch/navigation.launch.py`

**Usage:**
```bash
ros2 launch hydrakon_launch navigation.launch.py
```

This launch file initializes the full Navigation 2 (Nav2) stack for autonomous navigation within a pre-built map.

### Key Nodes
*   **Carla Bridge:** Provides Odometry and TF.
*   **Pointcloud to LaserScan:** Converts LiDAR data for the local costmap (obstacle avoidance).
*   **Nav2 Bringup:** Launches the core Nav2 stack (planner, controller, behavior trees, etc.) with the specified map and configuration.
*   **Initial Pose:** Automatically publishes an initial pose estimate to localize the robot on the map.