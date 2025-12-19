#!/usr/bin/env python3
"""
Launch file for point cloud mapping nodes.

Launches:
    - point_cloud_stitcher_node: Combines point clouds from all cameras
    - occupancy_grid_mapper_node: Creates 2D occupancy grid from point cloud
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    num_cameras_arg = DeclareLaunchArgument(
        'num_cameras',
        default_value='4',
        description='Number of depth cameras'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Map resolution in meters'
    )

    # Point cloud stitcher
    point_cloud_stitcher = Node(
        package='multi_camera_robot_nav',
        executable='point_cloud_stitcher_node',
        name='point_cloud_stitcher_node',
        output='screen',
        parameters=[{
            'num_cameras': LaunchConfiguration('num_cameras'),
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
            'resolution': LaunchConfiguration('resolution'),
            'map_width': 8.0,
            'map_height': 8.0,
            'origin_x': -4.0,
            'origin_y': -4.0,
            'obstacle_height_min': 0.1,
            'obstacle_height_max': 1.5,
            'free_threshold': 3,
            'occupied_threshold': 5,
            'publish_rate': 5.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        num_cameras_arg,
        resolution_arg,
        point_cloud_stitcher,
        occupancy_grid_mapper,
    ])
