#!/usr/bin/env python3
"""
Full System Launch File - Starts everything for robot navigation with marker tracking.

Launches:
    - Ignition Gazebo with tracking_room world and robot
    - ROS-Gazebo bridge for cameras, odom, cmd_vel
    - ArUco marker tracker (real or simulation bridge)
    - Navigation stack (Nav2)
    - RViz visualization
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('multi_camera_robot_nav')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'tracking_room.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    model_path = os.path.join(pkg_share, 'models')
    robot_sdf_file = os.path.join(pkg_share, 'models', 'marker_robot', 'model.sdf')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'navigation.rviz')

    # Read URDF for robot_state_publisher
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Set Ignition resource path for models
    existing_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    new_path = f'{model_path}:{existing_path}' if existing_path else model_path
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=new_path
    )

    # ==================== Launch Arguments ====================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Use simulation bridge instead of real ArUco tracker'
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

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    # ==================== Gazebo Simulation ====================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items()
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
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-file', robot_sdf_file,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.05',
        ]
    )

    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Camera 1
            '/cameras/camera_1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_1/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_1/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cameras/camera_1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Camera 2
            '/cameras/camera_2/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_2/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_2/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cameras/camera_2/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Camera 3
            '/cameras/camera_3/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_3/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_3/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cameras/camera_3/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Camera 4
            '/cameras/camera_4/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_4/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_4/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cameras/camera_4/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}]
    )

    # ==================== Marker Tracking ====================
    # Real marker tracker (for real hardware)
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

    # Simulation bridge (for Gazebo - gets pose directly from simulation)
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

    # Static transform: world -> map
    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    # ==================== Mapping ====================
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

    # ==================== Navigation (delayed start) ====================
    # Delay Nav2 launch to allow Gazebo to stabilize
    nav2_bringup = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to start
        actions=[
            IncludeLaunchDescription(
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
        ]
    )

    # ==================== Visualization ====================
    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            TimerAction(
                period=3.0,  # Wait 3 seconds for topics to be available
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        output='screen',
                        arguments=['-d', rviz_config_file],
                        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
                    )
                ]
            )
        ]
    )

    return LaunchDescription([
        # Environment
        set_ign_resource_path,
        # Arguments
        use_sim_time_arg,
        simulation_arg,
        robot_name_arg,
        x_pose_arg,
        y_pose_arg,
        autostart_arg,
        rviz_arg,
        # Gazebo simulation
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        # Marker tracking / localization
        marker_tracker,
        simulation_bridge,
        tf_broadcaster,
        static_tf_world_map,
        # Mapping
        point_cloud_stitcher,
        occupancy_grid_mapper,
        # Navigation (delayed)
        nav2_bringup,
        # Visualization (delayed)
        rviz,
    ])
