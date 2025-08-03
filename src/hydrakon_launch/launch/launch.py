#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hydrakon_manager',
            executable='vehicle_spawner',
            name='carla_vehicle_spawner',
            output='screen',
            parameters=[
                {'carla_host': 'localhost'},
                {'carla_port': 2000}
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='hydrakon_camera',
                    executable='depth_camera_spawner',
                    name='carla_depth_camera_spawner',
                    output='screen',
                    parameters=[
                        {'carla_host': 'localhost'},
                        {'carla_port': 2000},
                        {'camera_width': 800},
                        {'camera_height': 600},
                        {'camera_fov': 90.0}
                    ]
                )
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='hydrakon_camera',
                    executable='rgb_camera_spawner',
                    name='carla_rgb_camera_spawner',
                    output='screen',
                )
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='hydrakon_camera',
                    executable='depth_anything_processor',
                    name='depth_anything_processor',
                    output='screen',
                    cwd='/home/aditya/HydrakonSimV2/src/hydrakon_camera/Depth-Anything-V2',
                )
            ]
        ),
    ])