import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hydrakon_launch_dir = get_package_share_directory('hydrakon_launch')
    slam_config_file = os.path.join(hydrakon_launch_dir, 'config', 'mapper_params_online_async.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='localhost',
        description='Carla Host IP'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='localhost',
        description='Carla Host IP'
    )

    return LaunchDescription([
        
        host_arg,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        # 1. Carla to ROS Bridge (Provides Odom and TF)
        Node(
            package='hydrakon_manager',
            executable='carla_bridge',
            name='carla_bridge_slam',
            output='screen',
            parameters=[
                {'carla_host': LaunchConfiguration('host')},
                {'carla_host': LaunchConfiguration('host')},
                {'carla_port': 2000}
            ]
        ),

        # 2. PointCloud to LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'target_frame': 'lidar_link',
                'transform_tolerance': 0.01,
                'min_height': -1.3, 
                'max_height': 0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.05,
                'range_min': 3.0,
                'range_max': 12.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            remappings=[
                ('cloud_in', '/carla/lidar'),
                ('scan', '/scan')
            ]
        ),

        # 3. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_file, {'use_sim_time': use_sim_time}]
        ),

        # ========================================================================================
        # ROS 2 VERSION HANDLING
        # ----------------------------------------------------------------------------------------
        # OPTION 1: ROS 2 HUMBLE (Ubuntu 22.04) - "The Old Way"
        # On Humble, SLAM Toolbox often auto-starts. The Lifecycle Manager is optional or handled differently.
        # TO USE: Comment out the "OPTION 2" block below.
        # ========================================================================================

        # ========================================================================================
        # OPTION 2: ROS 2 JAZZY (Ubuntu 24.04) - "The New Way"
        # On Jazzy, SLAM Toolbox is a strict Lifecycle Node and stays 'Unconfigured' without this manager.
        # TO USE: Keep this block uncommented.
        # ========================================================================================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['slam_toolbox']}
            ]
        ),
        # ========================================================================================

        # ========================================================================================
        # ROS 2 VERSION HANDLING
        # ----------------------------------------------------------------------------------------
        # OPTION 1: ROS 2 HUMBLE (Ubuntu 22.04) - "The Old Way"
        # On Humble, SLAM Toolbox often auto-starts. The Lifecycle Manager is optional or handled differently.
        # TO USE: Comment out the "OPTION 2" block below.
        # ========================================================================================

        # ========================================================================================
        # OPTION 2: ROS 2 JAZZY (Ubuntu 24.04) - "The New Way"
        # On Jazzy, SLAM Toolbox is a strict Lifecycle Node and stays 'Unconfigured' without this manager.
        # TO USE: Keep this block uncommented.
        # ========================================================================================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['slam_toolbox']},
                {'bond_timeout': 0.0}
            ]
        ),
        # ========================================================================================

        # 4. Pure Pursuit (Delayed to ensure system is stable)
        TimerAction(
            period=3.0, # Brief delay for SLAM to initialize
            actions=[
                Node(
                    package='hydrakon_manager',
                    executable='pure_pursuit',
                    name='pure_pursuit_node',
                    output='screen'
                )
            ]
        )
    ])