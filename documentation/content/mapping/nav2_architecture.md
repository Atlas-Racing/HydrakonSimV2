---
title: Mapping and Navigation Architecture
---

## 1. Mapping (SLAM)

HydrakonSimV2 uses **SLAM Toolbox** for creating 2D occupancy grid maps of the track. This process typically runs during a "Mapping Lap" where the vehicle drives slowly around the track.

**Launch File:** `src/hydrakon_launch/launch/mapping_lap.launch.py`

### Architecture

1.  **Sensor Input:**
    *   **Source:** CARLA LiDAR sensor (`/carla/lidar`).
    *   **Processing:** The `pointcloud_to_laserscan` node flattens the 3D point cloud into a 2D `LaserScan`. This is critical because SLAM Toolbox operates on 2D scan data.
    *   **Filtering:** The scan is filtered by height (`min_height`/`max_height`) to remove ground strikes and high overhead objects.

2.  **Odometry:**
    *   **Source:** `carla_bridge` provides ground truth odometry from the simulator.
    *   **Transform:** Publishes the `odom` -> `base_footprint` TF.

3.  **SLAM Toolbox (Online Async):**
    *   Subscribes to `/scan` and `/tf`.
    *   Builds a pose graph and occupancy grid map.
    *   **Loop Closure:** Detects when the vehicle returns to a previously visited location to correct drift.
    *   **Output:** Publishes the `/map` topic and the `map` -> `odom` TF correction.

4.  **Map Saving:**
    *   The `pure_pursuit` controller is programmed to automatically trigger the `nav2_map_server` CLI to save the map to `~/HydrakonSimV2/my_track_map` upon completing the target number of laps.

---

## 2. Navigation (Nav2 Stack)

Once a map is generated, the system switches to the **Navigation 2 (Nav2)** stack for high-speed autonomous racing.

**Launch File:** `src/hydrakon_launch/launch/navigation.launch.py`

### System Breakdown

#### A. Localization (AMCL)
*   **Purpose:** Determines the robot's precise location on the map.
*   **Algorithm:** Adaptive Monte Carlo Localization (Particle Filter).
*   **Process:**
    1.  Takes the initial pose estimate (published automatically by the launch file).
    2.  Compares the real-time `LaserScan` with the static `Map`.
    3.  Converges particles to the most likely position.
    4.  Publishes the `map` -> `odom` transform to correct for odometry drift.

#### B. Global Planner (The "Brain")
*   **Role:** Calculates the optimal path from the robot's current position to the goal.
*   **Server:** `planner_server`
*   **Algorithm:** Configurable (typically A* or Dijkstra via `NavFnPlanner`).
*   **Input:** Global Costmap (Static Map + Inflation Layer).

#### C. Local Planner / Controller (The "Driver")
*   **Role:** Executes the global path while avoiding dynamic obstacles.
*   **Server:** `controller_server`
*   **Algorithm:** DWB (Dynamic Window Approach) or MPPI.
*   **Function:**
    *   Generates velocity commands (`/cmd_vel`).
    *   Uses "Critics" to score trajectories (e.g., Path Align, Goal Align, Obstacle Avoidance).
    *   **Tuning:** The controller is tuned for high-speed tracking (up to 4.0 m/s) with minimal oscillation.

#### D. Costmaps
*   **Global Costmap:** Used by the planner. Represents the static track layout.
*   **Local Costmap:** Used by the controller. Represents the immediate environment (rolling window) and updates dynamically with live Lidar data.

### Autonomous Racing Workflow

1.  **Launch:** The `navigation.launch.py` brings up the Bridge, Perception, and Nav2 stack.
2.  **Lifecycle Management:** Nav2 nodes start in an "Unconfigured" state and transition to "Active".
3.  **Initialization:** The launch file posts a fake `/initialpose` to jumpstart AMCL.
4.  **Race Control:**
    *   *(Planned Feature)* A Lap Manager node loads a pre-recorded racing line (`my_track_path.csv`).
    *   It waits for AMCL to localize.
    *   It sends the racing line as a series of waypoints to the Nav2 `FollowWaypoints` action server.
    *   The vehicle executes the path.
