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
pip install "numpy<2" torch torchvision ultralytics opencv-python
```

### 4. Carla 0.10.0 Setup (Important Compatibility Fix)
Carla 0.10.0 binaries for Linux typically come with a Python API wheel built for Python 3.10 (`cp310`). Since Ubuntu 24.04 uses Python 3.12, we must apply a workaround to install the API.

1.  **Locate your Carla Python API wheel.** It is usually in `<CARLA_ROOT>/PythonAPI/carla/dist/`.
    *   *Example:* `~/Documents/Carla/PythonAPI/carla/dist/carla-0.10.0-cp310-cp310-linux_x86_64.whl`

2.  **Copy and Rename the Wheel:**
    Copy the wheel to a temporary location and rename it to match Python 3.12 tags (`cp312`).
    ```bash
    cp ~/Documents/Carla/PythonAPI/carla/dist/carla-0.10.0-cp310-cp310-linux_x86_64.whl /tmp/carla-0.10.0-cp312-cp312-linux_x86_64.whl
    ```

3.  **Install the Renamed Wheel:**
    ```bash
    pip install /tmp/carla-0.10.0-cp312-cp312-linux_x86_64.whl
    ```

4.  **Rename the Shared Object File (The Critical Fix):**
    The installed package still contains a C extension compiled with the "310" name. We must rename it so Python 3.12 can import it.
    ```bash
    # Navigate to your venv site-packages
    cd venv/lib/python3.12/site-packages/
    
    # Rename the file (remove the specific python version tag)
    mv carla.cpython-310-x86_64-linux-gnu.so carla.so
    
    # Return to project root
    cd -
    ```

### 5. Depth Anything V2 Setup
The `depth_anything_processor` node requires model weights to be downloaded manually.

```bash
# Create the checkpoints directory
mkdir -p src/hydrakon_camera/Depth-Anything-V2/checkpoints

# Download the VITS (Small) model weights
wget -O src/hydrakon_camera/Depth-Anything-V2/checkpoints/depth_anything_v2_vits.pth https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth
```

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

# 4. Launch
ros2 launch hydrakon_launch launch.py
```

### One-Liner for Launching
For convenience, you can run:
```bash
source venv/bin/activate && source install/setup.bash && export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages && ros2 launch hydrakon_launch launch.py
```

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
