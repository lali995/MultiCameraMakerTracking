#!/usr/bin/env python3
"""
Launch file for simulation mode.

Launches:
    - simulation_bridge_node: Bridges Gazebo to ROS2 topics
    - tf_broadcaster_node: Publishes TF transforms
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model_name',
        default_value='robot',
        description='Name of the robot model in Gazebo'
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

    add_noise_arg = DeclareLaunchArgument(
        'add_noise',
        default_value='true',
        description='Add simulated sensor noise'
    )

    position_noise_arg = DeclareLaunchArgument(
        'position_noise_std',
        default_value='0.01',
        description='Position noise standard deviation (m)'
    )

    orientation_noise_arg = DeclareLaunchArgument(
        'orientation_noise_std',
        default_value='0.02',
        description='Orientation noise standard deviation (rad)'
    )

    # Simulation bridge node
    simulation_bridge_node = Node(
        package='multi_camera_robot_nav',
        executable='simulation_bridge_node',
        name='simulation_bridge_node',
        output='screen',
        parameters=[{
            'robot_model_name': LaunchConfiguration('robot_model_name'),
            'world_frame': LaunchConfiguration('world_frame'),
            'robot_frame': LaunchConfiguration('robot_frame'),
            'add_noise': LaunchConfiguration('add_noise'),
            'position_noise_std': LaunchConfiguration('position_noise_std'),
            'orientation_noise_std': LaunchConfiguration('orientation_noise_std'),
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
        robot_model_arg,
        world_frame_arg,
        robot_frame_arg,
        add_noise_arg,
        position_noise_arg,
        orientation_noise_arg,
        simulation_bridge_node,
        tf_broadcaster_node,
    ])
