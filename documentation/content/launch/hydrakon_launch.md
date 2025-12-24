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

## Launch Arguments

| Argument | Default Value | Description |
| :--- | :--- | :--- |
| `model_path` | `.../best.onnx` | Path to the YOLO object detection model file. |
| `benchmark` | `False` | Enable benchmarking mode for inference timing statistics. |
| `manual_control` | `False` | Set to `True` to launch the Pygame manual control window. |
| `gw` | `False` | Set to `True` to launch the Greenwave Monitor (TUI). |
