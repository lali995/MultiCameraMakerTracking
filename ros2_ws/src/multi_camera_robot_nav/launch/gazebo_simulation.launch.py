#!/usr/bin/env python3
"""
Launch file for Gazebo simulation with ceiling cameras and robot.

Launches:
    - Gazebo with tracking_room world
    - Robot spawner
    - Simulation bridge node
    - TF broadcaster node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share directory
    pkg_share = get_package_share_directory('multi_camera_robot_nav')

    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'tracking_room.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Name of the robot in Gazebo'
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial X position'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial Y position'
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # Gazebo client (GUI) - optional, can be disabled for headless
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.1',
        ]
    )

    # Simulation bridge node
    simulation_bridge = Node(
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
    )

    # TF broadcaster
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

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        x_pose_arg,
        y_pose_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        simulation_bridge,
        tf_broadcaster,
    ])
