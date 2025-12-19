#!/usr/bin/env python3
"""
Launch file for marker tracking nodes.

Launches:
    - marker_tracker_node: Bridges internal tracker to ROS2
    - tf_broadcaster_node: Publishes TF transforms
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='0',
        description='ID of the marker to track (mounted on robot)'
    )

    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='world',
        description='World/map frame ID'
    )

    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_link',
        description='Robot base frame ID'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Publish rate in Hz'
    )

    # Marker tracker node
    marker_tracker_node = Node(
        package='multi_camera_robot_nav',
        executable='marker_tracker_node',
        name='marker_tracker_node',
        output='screen',
        parameters=[{
            'marker_id': LaunchConfiguration('marker_id'),
            'world_frame': LaunchConfiguration('world_frame'),
            'robot_frame': LaunchConfiguration('robot_frame'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    # TF broadcaster node
    tf_broadcaster_node = Node(
        package='multi_camera_robot_nav',
        executable='tf_broadcaster_node',
        name='tf_broadcaster_node',
        output='screen',
        parameters=[{
            'world_frame': LaunchConfiguration('world_frame'),
            'odom_frame': 'odom',
            'robot_frame': LaunchConfiguration('robot_frame'),
        }]
    )

    return LaunchDescription([
        marker_id_arg,
        world_frame_arg,
        robot_frame_arg,
        publish_rate_arg,
        marker_tracker_node,
        tf_broadcaster_node,
    ])
