import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'hydrakon_description'
    file_subpath = 'urdf/vehicle.urdf.xacro'

    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # 1. Publish the Robot State (TF Tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # 2. GUI to play with joints
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui'
        # ),

        # 2. Carla Bridge (Joint States)
        Node(
            package='hydrakon_manager',
            executable='carla_bridge',
            name='carla_bridge',
            output='screen'
        )
    ])