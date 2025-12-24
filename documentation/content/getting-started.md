---
title: 2. Getting Started
---

## Building the Workspace

Use `colcon` to build the ROS 2 workspace. We use `--symlink-install` to allow Python script changes to take effect without rebuilding (mostly).

```bash
# Ensure you are in the project root
colcon build --symlink-install
```

## Running the Simulation

To run the simulation, you must set up the environment variables correctly. The `PYTHONPATH` export is **critical** for ROS 2 to find the `carla` module installed in your virtual environment.

```bash
# 1. Activate Virtual Environment
source venv/bin/activate

# 2. Source the Workspace
source install/setup.bash

# 3. Export PYTHONPATH (Crucial for finding 'carla')
export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages

# 4. Launch (with optional parameters)
ros2 launch hydrakon_launch launch.py [model_path:=...] [benchmark:=True/False] [manual_control:=True/False]
```

### Launch Arguments

For a comprehensive list and detailed descriptions of all launch arguments, please refer to the [Launch System Overview](/launch/hydrakon_launch) documentation.

*   `model_path`: Path to the YOLO model (default: `src/.../models/best.onnx`).
*   `benchmark`: Enable inference timing logging (default: `False`).
*   `manual_control`: Enable Pygame window for driving the vehicle (default: `False`).
*   `gw`: Enable Greenwave Monitor TUI (default: `False`).

### Examples

**Run with Manual Control:**
```bash
ros2 launch hydrakon_launch launch.py manual_control:=True
```
Use **WASD** to drive, **Space** for handbrake, **Q** to toggle reverse.

**Run with PyTorch model (default) and no benchmarking:**
```bash
source venv/bin/activate && source install/setup.bash && export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages && ros2 launch hydrakon_launch launch.py
```

**Run with ONNX model and enable benchmarking:**
```bash
source venv/bin/activate && source install/setup.bash && export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages && ros2 launch hydrakon_launch launch.py model_path:=/home/aditya/HydrakonSimV2/src/hydrakon_camera/hydrakon_camera/best.onnx benchmark:=True
```

## Visualization in RViz

The system publishes 3D markers for detected cones, allowing you to visualize their positions in 3D space.

1.  **Start the Simulation:** Run one of the launch commands above.
2.  **Open RViz2:**
    ```bash
    ros2 run rviz2 rviz2
    ```
3.  **Configure RViz:**
    *   **Fixed Frame:** Set to `cone_frame` (Type this in manually).
    *   **Add Display:** Click **Add** -> Select **MarkerArray** -> Set Topic to `/camera/cone_markers`.
    *   **Add Display (Optional):** Click **Add** -> Select **Image** -> Set Topic to `/camera/cone_detections_image` to see the 2D detections.

The cones will appear as cylinders on the grid (XY plane), color-coded by class:
*   **Yellow:** Yellow Cone
*   **Blue:** Blue Cone
*   **Orange:** Orange Cone / Large Orange Cone
*   **White:** Unknown
