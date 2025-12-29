import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hydrakon_launch_dir = get_package_share_directory('hydrakon_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    home_dir = os.path.expanduser('~')
    map_dir = os.path.join(home_dir, 'HydrakonSimV2', 'my_track_map.yaml') 
    params_file = os.path.join(hydrakon_launch_dir, 'config', 'nav2_params.yaml')

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
            name='carla_bridge_nav',
            output='screen',
            parameters=[
                {'carla_host': LaunchConfiguration('host')},
                {'carla_port': 2000}
            ]
        ),

        # 2. PointCloud to LaserScan (For obstacle avoidance)
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

        # 3. Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
                'initial_pose_x': '0.0',
                'initial_pose_y': '0.0',
                'initial_pose_yaw': '0.0'
            }.items()
        ),

        # 4. Force Initial Pose (Manual Publish)
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '-1', '/initialpose', 
                        'geometry_msgs/msg/PoseWithCovarianceStamped',
                        '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}'
                    ],
                    output='screen'
                )
            ]
        ),

        # 5. Lap Manager (Waypoints for Nav2)
        Node(
            package='hydrakon_manager',
            executable='lap_manager',
            name='lap_manager_node',
            output='screen',
            parameters=[
                {'path_file': '/home/abdul/Documents/CARLA_2025/HydrakonSimV2/my_track_path.csv'},
                {'waypoint_spacing': 5.0},
                {'laps': 3}
            ]
        )
    ])
