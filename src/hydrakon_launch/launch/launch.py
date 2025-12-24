#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/aditya/HydrakonSimV2/src/hydrakon_camera/hydrakon_camera/models/best.onnx',
        description='Path to the YOLO model (.pt or .onnx)'
    )

    benchmark_arg = DeclareLaunchArgument(
        'benchmark',
        default_value='False',
        description='Enable benchmarking mode for inference timing'
    )

    manual_control_arg = DeclareLaunchArgument(
        'manual_control',
        default_value='False',
        description='Enable manual control window (Pygame)'
    )

    gw_arg = DeclareLaunchArgument(
        'gw',
        default_value='False',
        description='Enable Greenwave Monitor (TUI)'
    )
    
    from launch.conditions import IfCondition
    
    hydrakon_description_dir = get_package_share_directory('hydrakon_description')
    hydrakon_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hydrakon_description_dir, 'launch', 'display.launch.py')
        )
    )

    return LaunchDescription([
        model_path_arg,
        benchmark_arg,
        manual_control_arg,
        gw_arg,
        
        hydrakon_description_launch,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_cone_frame',
            arguments=['0', '0', '0.7', '0', '0', '0', 'base_link', 'cone_frame']
        ),

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

        Node(
            package='hydrakon_manager',
            executable='manual_control',
            name='manual_control',
            output='screen',
            condition=IfCondition(LaunchConfiguration('manual_control')),
            parameters=[
                {'carla_host': 'localhost'},
                {'carla_port': 2000}
            ]
        ),

        TimerAction(
            period=1.0,
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
            period=1.0,
            actions=[
                Node(
                    package='hydrakon_camera',
                    executable='rgb_camera_spawner',
                    name='carla_rgb_camera_spawner',
                    output='screen',
                )
            ]
        ),

        # TimerAction(
        #     period=1.0,
        #     actions=[
        #         Node(
        #             package='hydrakon_camera',
        #             executable='depth_anything_processor',
        #             name='depth_anything_processor',
        #             output='screen',
        #             cwd='/home/aditya/HydrakonSimV2/src/hydrakon_camera/Depth-Anything-V2',
        #         )
        #     ]
        # ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='hydrakon_camera',
                    executable='cone_detector',
                    name='cone_detections',
                    output='screen',
                    parameters=[
                        {'model_path': LaunchConfiguration('model_path')},
                        {'benchmark': LaunchConfiguration('benchmark')}
                    ]
                )
            ]
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='hydrakon_camera',
                    executable='cone_locator',
                    name='cone_locator',
                    output='screen',
                )
            ]
        ),

        # TimerAction(
        #     period=3.0,
        #     actions=[
        #         Node(
        #             package='greenwave_monitor',
        #             executable='greenwave_monitor',
        #             name='greenwave_monitor',
        #             output='log',
        #         )
        #     ]
        # ),
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='r2s_gw',
                    executable='r2s_gw',
                    name='r2s_gw',
                    output='screen',
                    prefix='xterm -e',
                    condition=IfCondition(LaunchConfiguration('gw')),
                )
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('hydrakon_launch'), 'rviz', 'default.rviz')]
        )
        
    ])