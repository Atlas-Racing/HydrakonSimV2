# HydrakonSimV2

Revamped HydrakonSim V2 that builds upon HydrakonSimV1. Being Developed for HydrakonV2 for FS-AI UK 2026.

## Prerequisites

*   **OS:** Ubuntu 24.04 (Noble Numbat)
*   **ROS 2:** Jazzy Jalisco
*   **Simulator:** Carla 0.10.0 (Unreal Engine 5 based)
*   **Python:** 3.12 (Default system Python)

## Installation Guide

### 1. Cloning the Repository
Clone the repository recursively to ensure submodules (like Depth-Anything-V2) are included.

```bash
git clone --recursive https://github.com/FS-AI-HWUD/HydrakonSimV2.git
cd HydrakonSimV2
```

If you already cloned without `--recursive`:
```bash
git submodule update --init --recursive
```

### 2. Virtual Environment Setup
It is highly recommended to use a virtual environment to manage Python dependencies and avoid conflicts with system packages (especially for the Carla "hack").

```bash
python3 -m venv venv
source venv/bin/activate
```

**Note:** Always ensure the virtual environment is activated (`source venv/bin/activate`) before installing packages or running the simulation.

### 3. Install Dependencies
Install the required Python libraries. Note that we strictly require **NumPy < 2.0** because `cv_bridge` in ROS 2 Jazzy is currently compiled against NumPy 1.x.

```bash
pip install "numpy<2" torch torchvision ultralytics opencv-python onnx onnxruntime-gpu
```
...
## Running the Simulation

To run the simulation, you must set up the environment variables correctly. The `PYTHONPATH` export is **critical** for ROS 2 to find the `carla` module installed in your virtual environment.

```bash
# 1. Activate Virtual Environment
source venv/bin/activate

# 2. Source the Workspace
source install/setup.bash

# 3. Export PYTHONPATH (Crucial for finding 'carla')
export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages

# 4. Launch (with optional parameters for cone_detector)
ros2 launch hydrakon_launch launch.py [model_path:=/path/to/model.pt_or_onnx] [benchmark:=True/False]
```

### Examples for `cone_detector`

**Run with PyTorch model (default) and no benchmarking:**
```bash
source venv/bin/activate && source install/setup.bash && export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages && ros2 launch hydrakon_launch launch.py
```

**Run with ONNX model and enable benchmarking:**
```bash
source venv/bin/activate && source install/setup.bash && export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages && ros2 launch hydrakon_launch launch.py model_path:=/home/aditya/HydrakonSimV2/src/hydrakon_camera/hydrakon_camera/best.onnx benchmark:=True
```

**Run with PyTorch model and enable benchmarking:**
```bash
source venv/bin/activate && source install/setup.bash && export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages && ros2 launch hydrakon_launch launch.py model_path:=/home/aditya/HydrakonSimV2/src/hydrakon_camera/hydrakon_camera/best.pt benchmark:=True
```

## Visualization in RViz

The system publishes 3D markers for detected cones, allowing you to visualize their positions in 3D space.

1.  **Start the Simulation:** Run one of the launch commands above.
2.  **Open RViz2:**
    ```bash
    rviz2
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

## Troubleshooting

### `ModuleNotFoundError: No module named 'carla'`
*   **Cause:** The `PYTHONPATH` is not set correctly, or the `.so` file in `site-packages` was not renamed.
*   **Fix:** Ensure you exported `PYTHONPATH` pointing to your `venv/lib/python3.12/site-packages` and that `carla.so` exists in that folder (see Installation Step 4).

### `ImportError: ... module compiled using NumPy 1.x cannot be run in NumPy 2.x`
*   **Cause:** You have `numpy` version 2.0 or greater installed, but ROS 2 Jazzy's `cv_bridge` requires NumPy 1.x.
*   **Fix:** Run `pip install "numpy<2"`.

### `NameError: name 'da_path' is not defined`
*   **Cause:** Older code version in `depth_anything.py`.
*   **Fix:** Pull the latest changes or ensure `da_path` is defined as `self.da_path` in the class `__init__` method.
