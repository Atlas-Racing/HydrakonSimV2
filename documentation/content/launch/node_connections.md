---
title: 2. Node Connections and Visuals
---

This diagram illustrates the architecture of the Hydrakon simulation system, showing the data flow between different ROS 2 nodes and packages.

```mermaid
graph TD
    subgraph Manager_Package ["hydrakon_manager"]
        VS[vehicle_spawner]
        MC[manual_control]
        CB[carla_bridge]
    end

    subgraph External ["External Simulator"]
        Carla(CARLA Simulator)
    end

    subgraph Camera_Package ["hydrakon_camera"]
        RGB[rgb_camera_spawner]
        Depth[depth_camera_spawner]
        YOLO[cone_detector]
        Locator[cone_locator]
    end

    subgraph Desc_Package ["hydrakon_description"]
        RSP[robot_state_publisher]
    end

    subgraph TF_Package ["Package: tf2_ros"]
        TF[static_transform_publisher]
    end

    subgraph Vis_Package ["Visualization"]
        Rviz[rviz2]
        GW[r2s_gw]
    end

    VS -.->|Spawns Vehicle| Carla
    MC -.->|Controls Vehicle| Carla
    Carla ==>|Vehicle State| CB
    
    Carla ==>|RGB Data| RGB
    Carla ==>|Depth Data| Depth
    
    RGB -->|/camera/raw| YOLO
    YOLO -->|/camera/cone_detections| Locator
    Depth -->|/camera/depth| Locator
    Locator -->|/camera/cone_markers| Rviz
    YOLO -.->|/camera/cone_detections_image| Rviz

    CB -->|/joint_states| RSP
    RSP -->|/tf, /tf_static| Rviz
    TF -->|/tf_static| Rviz
    RSP -.->|URDF Model| Rviz

    GW -.-|Monitors System| CB
    GW -.-|Monitors System| YOLO
```

## Description of Data Flow

1.  **Vehicle Control & State:**
    *   The `vehicle_spawner` initializes the car in CARLA.
    *   The `manual_control` node sends control commands to CARLA.
    *   The `carla_bridge` reads the car's state (velocity, steering) from CARLA and calculates wheel positions, publishing them to `/joint_states`.

2.  **Robot Visualization:**
    *   `robot_state_publisher` reads the URDF and the `/joint_states` to publish the Transform Tree (`/tf`).
    *   `rviz2` uses this TF tree to render the moving robot model.

3.  **Perception Pipeline:**
    *   `rgb_camera_spawner` and `depth_camera_spawner` bridge CARLA sensor data to ROS topics.
    *   `cone_detector` runs YOLO inference on the RGB image to find bounding boxes.
    *   `cone_locator` combines these bounding boxes with the depth map to calculate the 3D position of cones relative to the car.
    *   These 3D positions are published as markers for Rviz.