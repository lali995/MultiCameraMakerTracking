#!/usr/bin/env python3
"""
Launch file for RViz2 visualization with navigation configuration.

Launches RViz2 with pre-configured displays for:
    - Robot model and TF tree
    - Occupancy map from point cloud stitching
    - Combined point cloud visualization
    - Nav2 costmaps and paths
    - Navigation goal tools
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share directory
    pkg_share = get_package_share_directory('multi_camera_robot_nav')

    # RViz config file
    rviz_config = os.path.join(pkg_share, 'config', 'navigation.rviz')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Path to RViz configuration file'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        rviz_node,
    ])
