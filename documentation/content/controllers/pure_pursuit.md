---
title: Pure Pursuit Controller
---

## Overview

The Pure Pursuit controller is a geometric path tracking algorithm used for the `mapping_lap.launch.py` workflow. It calculates the necessary steering angle to move the vehicle from its current position to a target point on the path.

**Source File:** `src/hydrakon_manager/hydrakon_manager/pure_pursuit.py`

## How it Works

The implementation in HydrakonSimV2 is customized for cone-based track following (specifically for Formula Student scenarios).

1.  **Perception Input:** It subscribes to `/camera/cone_markers` (published by `cone_locator`) to receive the 3D positions of detected blue and yellow cones.
2.  **Target Selection:**
    *   The controller looks for the furthest visible gate (pair of blue and yellow cones).
    *   **Both Sides Visible:** The target is the midpoint between the blue and yellow cone.
    *   **One Side Visible:** The target is offset from the single cone by `track_width_offset` (multiplied by a dynamic `single_side_offset_multiplier`).
    *   **No Cones:** The vehicle stops for safety.
3.  **Steering Calculation:**
    *   Calculates the curvature required to drive an arc to the target point.
    *   `Steering Angle = atan( (2 * Wheelbase * Target_Y) / Lookahead_Distance^2 )`
    *   **Adaptive Lookahead:** The lookahead distance (`L_d`) adapts based on the distance to the target, clamped by `min_lookahead`.
4.  **Speed Control:**
    *   The speed is adjusted based on the sharpness of the turn.
    *   **Straight:** High speed (`constant_speed`).
    *   **Sharp Turn:** Low speed (`min_speed`).
    *   Interpolates linearly based on the steering ratio.

## State Machine

The node implements a simple state machine to manage the lap:

1.  **START_ZONE (0):** Waits until the vehicle leaves the start area (orange cones disappear).
2.  **RACING (1):** actively follows the track. Counts laps when crossing the finish line (orange cones).
3.  **FINISHED (2):** Stops the vehicle after the target number of laps and automatically saves the map using `nav2_map_server`.
4.  **LAP_COOLDOWN (3):** A brief pause after crossing the line to prevent double-counting laps.

## Parameters

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `min_lookahead` | `2.0` | Minimum lookahead distance (m). |
| `max_lookahead` | `5.0` | Maximum lookahead distance (m). |
| `max_steering_angle` | `0.7` | Maximum steering angle (rad) ~40 deg. |
| `constant_speed` | `0.1` | Max target speed (m/s). |
| `min_speed` | `0.05` | Minimum speed during sharp turns (m/s). |
| `track_width_offset` | `1.8` | Base offset from a single cone boundary (m). |
| `steering_gain` | `2.0` | Gain multiplier for steering aggressiveness. |
| `vision_horizon` | `15.0` | Maximum distance to consider cones (m). |
