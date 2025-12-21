#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/aditya/HydrakonSimV2/src/hydrakon_camera/hydrakon_camera/best.pt',
        description='Path to the YOLO model (.pt or .onnx)'
    )

    benchmark_arg = DeclareLaunchArgument(
        'benchmark',
        default_value='False',
        description='Enable benchmarking mode for inference timing'
    )

    return LaunchDescription([
        model_path_arg,
        benchmark_arg,

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
        #             package='rosapi',
        #             executable='rosapi_node',
        #             name='rosapi',
        #             output='screen'
        #         ),
        #         Node(
        #             package='rosbridge_server',
        #             executable='rosbridge_websocket',
        #             name='rosbridge_websocket',
        #             output='screen',
        #             parameters=[
        #                 {'port': 9090},
        #                 {'address': '0.0.0.0'},
        #                 {'authenticate': False},
        #                 {'fragment_timeout': 600},
        #                 {'delay_between_messages': 0}
        #             ]
        #         )
        #     ]
        # ),
        
    ])