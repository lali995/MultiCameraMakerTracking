#!/usr/bin/env python3
"""
Launch file for Ignition Gazebo (Fortress) simulation with robot.

Uses ros_gz for Gazebo Fortress/Ignition instead of Gazebo Classic.

Launches:
    - Ignition Gazebo with tracking_room world
    - Robot spawner with ArUco marker
    - ROS-Gazebo bridge
    - TF broadcaster node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share directory
    pkg_share = get_package_share_directory('multi_camera_robot_nav')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'tracking_room.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    model_path = os.path.join(pkg_share, 'models')
    robot_sdf_file = os.path.join(pkg_share, 'models', 'marker_robot', 'model.sdf')

    # Read URDF for robot_state_publisher
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Set environment variable for Ignition to find our models
    # Append to existing IGN_GAZEBO_RESOURCE_PATH if it exists
    existing_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    new_path = f'{model_path}:{existing_path}' if existing_path else model_path
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=new_path
    )

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

    # Ignition Gazebo - use gz sim (Fortress)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items()
    )

    # Robot state publisher (for TF tree)
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

    # Spawn robot SDF model in Ignition Gazebo (with ArUco marker texture)
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

    # ROS-Gazebo bridge for clock, robot control, and ceiling RGBD cameras
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Robot control (cmd_vel from ROS to Ignition)
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Robot odometry from diff_drive plugin (publishes odom and TF)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # TF from diff_drive (odom -> base_link)
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Ceiling camera 1 (NE corner) - RGB + Depth
            '/cameras/camera_1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_1/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_1/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cameras/camera_1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Ceiling camera 2 (NW corner) - RGB + Depth
            '/cameras/camera_2/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_2/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_2/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cameras/camera_2/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Ceiling camera 3 (SW corner) - RGB + Depth
            '/cameras/camera_3/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_3/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_3/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cameras/camera_3/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Ceiling camera 4 (SE corner) - RGB + Depth
            '/cameras/camera_4/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_4/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cameras/camera_4/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/cameras/camera_4/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}]
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

    # Static transform for world->map
    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    return LaunchDescription([
        set_ign_resource_path,
        use_sim_time_arg,
        robot_name_arg,
        x_pose_arg,
        y_pose_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        tf_broadcaster,
        static_tf_world_map,
    ])
