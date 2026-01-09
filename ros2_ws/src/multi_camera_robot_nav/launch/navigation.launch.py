#!/usr/bin/env python3
"""
Launch file for full robot navigation with external marker tracking.

Launches:
    - Marker tracker and TF broadcaster (or simulation bridge)
    - Point cloud stitcher and occupancy grid mapper
    - Nav2 navigation stack
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('multi_camera_robot_nav')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # Paths
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Use simulation bridge instead of real tracker'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Name of the robot model'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

    # Marker tracker (for real hardware)
    marker_tracker = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('simulation')),
        actions=[
            Node(
                package='multi_camera_robot_nav',
                executable='marker_tracker_node',
                name='marker_tracker_node',
                output='screen',
                parameters=[{
                    'marker_id': 0,
                    'world_frame': 'world',
                    'robot_frame': 'base_link',
                    'publish_rate': 30.0,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }]
            ),
        ]
    )

    # Simulation bridge (for Gazebo)
    simulation_bridge = GroupAction(
        condition=IfCondition(LaunchConfiguration('simulation')),
        actions=[
            Node(
                package='multi_camera_robot_nav',
                executable='simulation_bridge_node',
                name='simulation_bridge_node',
                output='screen',
                parameters=[{
                    'robot_model_name': LaunchConfiguration('robot_name'),
                    'world_frame': 'world',
                    'robot_frame': 'base_link',
                    'add_noise': True,
                    'position_noise_std': 0.01,
                    'orientation_noise_std': 0.02,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }]
            ),
        ]
    )

    # TF broadcaster (always needed)
    tf_broadcaster = Node(
        package='multi_camera_robot_nav',
        executable='tf_broadcaster_node',
        name='tf_broadcaster_node',
        output='screen',
        parameters=[{
            'world_frame': 'world',
            'odom_frame': 'odom',
            'robot_frame': 'base_link',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Point cloud stitcher
    point_cloud_stitcher = Node(
        package='multi_camera_robot_nav',
        executable='point_cloud_stitcher_node',
        name='point_cloud_stitcher_node',
        output='screen',
        parameters=[{
            'num_cameras': 4,
            'world_frame': 'world',
            'publish_rate': 10.0,
            'min_height': 0.05,
            'max_height': 2.0,
            'max_range': 5.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Occupancy grid mapper
    occupancy_grid_mapper = Node(
        package='multi_camera_robot_nav',
        executable='occupancy_grid_mapper_node',
        name='occupancy_grid_mapper_node',
        output='screen',
        parameters=[{
            'map_frame': 'world',
            'resolution': 0.05,
            'map_width': 8.0,
            'map_height': 8.0,
            'origin_x': -4.0,
            'origin_y': -4.0,
            'obstacle_height_min': 0.1,
            'obstacle_height_max': 1.5,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Static transform: world -> map (identity for this setup)
    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    # Nav2 bringup with extended timeout for simulation
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file,
            'autostart': LaunchConfiguration('autostart'),
            'bond_timeout': '10.0',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        simulation_arg,
        robot_name_arg,
        autostart_arg,
        marker_tracker,
        simulation_bridge,
        tf_broadcaster,
        point_cloud_stitcher,
        occupancy_grid_mapper,
        static_tf_world_map,
        nav2_bringup,
    ])
