# r2s_gw

**Note: This is a fork of [r2s](https://github.com/mjcarroll/r2s) designed to work with the Greenwave Monitor system. It has been renamed to r2s_gw to allow users to install it alongside the standard r2s package.**

r2s_gw is a _Text User Interface_ (TUI) for interacting with various aspects of a ROS 2 system, with enhanced integration for [Greenwave Monitor](https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor) diagnostics.

r2s_gw is written in Python and utilizes [Textual](https://github.com/textualize/textual/) for building the UI.

![Alt Text](doc/r2s.gif)

## Dependencies

r2s_gw depends on the [Greenwave Monitor](https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor) package. Make sure greenwave_monitor is installed first.

## Installation

This package is distributed separately from Greenwave Monitor to keep dependencies minimal.

### Install from source with colcon (recommended for ROS users):

```bash
cd ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/r2s_gw.git
cd ../..
colcon build --packages-up-to r2s_gw
source install/setup.bash
```

### Install from source with pip (for development):

```bash
cd r2s_gw
pip install -e .
```

## Usage

Launch the r2s_gw dashboard (automatically starts greenwave_monitor):

```bash
ros2 run r2s_gw r2s_gw_dashboard
```

Or launch with demo publishers:

```bash
ros2 run r2s_gw r2s_gw_dashboard -- --demo
```

For more usage details, see the [Greenwave Monitor README](https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor).

## Development

To run in development mode with Textual's dev tools:

```bash
# In one terminal, run the textual console
textual console

# In another terminal, run the app in dev mode
textual run --dev r2s_gw.main:main
```

You can also run the UI directly (without the dashboard wrapper script):

```bash
# Make sure greenwave_monitor is already running
ros2 run greenwave_monitor greenwave_monitor &

# Then run the r2s_gw UI
ros2 run r2s_gw r2s_gw
```

## Testing

Run tests with pytest:

```bash
# Install test dependencies
pip install pytest pytest-asyncio

# Run tests
pytest tests/
```

Or use colcon:

```bash
colcon test --packages-select r2s_gw
```

## Roadmap

Currently, there are 2 primary widgets for visualizing information in a grid or as a stream of text logs.

* ROS 2
  * Visualize stream of ROS messages in the text log view (ros2 topic echo)
  * Visualize stream of ROS logs from a node (rosconsole)
  * Visualize system load of individual nodes (requires instrumentation to associate PID with node)
  * Publish messages on a topic (ros2 topic pub)
  * Advance lifecycle node states (ros2 lifecycle)
  * Visualize message definitions (ros2 interface)
  * Call Services (ros2 service)
* Colcon
  * List packages in a workspace (colcon list/colcon graph)
  * Select and build multiple packages
  * View package build logs
  * View test logs
* Gazebo
  * Visualize topics/nodes
  * Port ROS functionality to gz-transport and gz-msgs
