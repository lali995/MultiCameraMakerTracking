#!/usr/bin/env python3
"""
Marker Tracker ROS2 Node

Bridges the multi-camera marker tracking system to ROS2.
Publishes robot pose and odometry from external camera observations.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from typing import Optional, Tuple
import time
import sys
import os

# Add parent project to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))

try:
    from src.core.data_types import FusedMarkerPose, CameraDetection
    from src.core.message_bus import get_message_bus
    from validation.velocity_estimator import VelocityEstimator
    TRACKER_AVAILABLE = True
except ImportError:
    TRACKER_AVAILABLE = False


class MarkerTrackerNode(Node):
    """
    ROS2 node that publishes robot pose from external marker tracking.

    Topics Published:
        /robot/pose (PoseStamped): Current robot pose in world frame
        /robot/pose_cov (PoseWithCovarianceStamped): Pose with covariance
        /robot/odom (Odometry): Full odometry including velocity
        /robot/velocity (TwistStamped): Robot velocity

    Parameters:
        marker_id (int): ID of the marker to track (default: 0)
        world_frame (str): World frame ID (default: 'world')
        robot_frame (str): Robot base frame ID (default: 'base_link')
        publish_rate (float): Publish rate in Hz (default: 30.0)
    """

    def __init__(self):
        super().__init__('marker_tracker_node')

        # Declare parameters
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('publish_rate', 30.0)

        # Get parameters
        self.marker_id = self.get_parameter('marker_id').value
        self.world_frame = self.get_parameter('world_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # QoS profile for reliable pose publishing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped, '/robot/pose', qos_profile)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot/pose_cov', qos_profile)
        self.odom_pub = self.create_publisher(
            Odometry, '/robot/odom', qos_profile)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/robot/velocity', qos_profile)

        # Internal state
        self.last_pose: Optional[FusedMarkerPose] = None
        self.velocity_estimator = VelocityEstimator() if TRACKER_AVAILABLE else None

        # Subscribe to internal message bus if available
        if TRACKER_AVAILABLE:
            self.message_bus = get_message_bus()
            self.message_bus.subscribe('/markers/fused_poses', self._on_fused_pose)
            self.get_logger().info('Connected to internal message bus')
        else:
            self.get_logger().warn('Tracker not available, running in standalone mode')

        # Timer for periodic publishing and message bus spinning
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Marker Tracker Node started. Tracking marker ID {self.marker_id}'
        )

    def _on_fused_pose(self, pose: 'FusedMarkerPose'):
        """Callback when new fused pose is received from tracker."""
        if pose.marker_id == self.marker_id:
            self.last_pose = pose
            if self.velocity_estimator:
                self.velocity_estimator.add_pose(
                    position=pose.to_array(),
                    orientation=pose.orientation,
                    timestamp=pose.timestamp
                )

    def timer_callback(self):
        """Periodic callback to publish pose and spin message bus."""
        # Spin internal message bus to process callbacks
        if TRACKER_AVAILABLE:
            self.message_bus.spin_once()

        # Publish pose if available
        if self.last_pose is not None:
            self.publish_pose(self.last_pose)

    def publish_pose(self, pose: 'FusedMarkerPose'):
        """Publish pose to all topics."""
        now = self.get_clock().now()

        # Create header
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.world_frame

        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position.x = float(pose.x)
        pose_msg.pose.position.y = float(pose.y)
        pose_msg.pose.position.z = float(pose.z)

        if pose.orientation:
            pose_msg.pose.orientation.x = float(pose.orientation[0])
            pose_msg.pose.orientation.y = float(pose.orientation[1])
            pose_msg.pose.orientation.z = float(pose.orientation[2])
            pose_msg.pose.orientation.w = float(pose.orientation[3])
        else:
            pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)

        # PoseWithCovarianceStamped
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header = header
        pose_cov_msg.pose.pose = pose_msg.pose

        # Covariance based on number of cameras and confidence
        position_var = 0.01 / max(pose.num_cameras(), 1)  # Lower with more cameras
        orientation_var = 0.05
        pose_cov_msg.pose.covariance = [
            position_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, position_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, position_var, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, orientation_var, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, orientation_var, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, orientation_var,
        ]

        self.pose_cov_pub.publish(pose_cov_msg)

        # Odometry
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.child_frame_id = self.robot_frame
        odom_msg.pose = pose_cov_msg.pose

        # Add velocity if available
        if self.velocity_estimator:
            linear_vel = self.velocity_estimator.get_linear_velocity()
            angular_vel = self.velocity_estimator.get_angular_velocity()

            if linear_vel is not None:
                odom_msg.twist.twist.linear.x = float(linear_vel[0])
                odom_msg.twist.twist.linear.y = float(linear_vel[1])
                odom_msg.twist.twist.linear.z = float(linear_vel[2])

            if angular_vel is not None:
                odom_msg.twist.twist.angular.x = float(angular_vel[0])
                odom_msg.twist.twist.angular.y = float(angular_vel[1])
                odom_msg.twist.twist.angular.z = float(angular_vel[2])

        self.odom_pub.publish(odom_msg)

        # TwistStamped
        velocity_msg = TwistStamped()
        velocity_msg.header = header
        velocity_msg.twist = odom_msg.twist.twist

        self.velocity_pub.publish(velocity_msg)

    def set_pose_from_simulation(self, x: float, y: float, z: float,
                                  qx: float = 0.0, qy: float = 0.0,
                                  qz: float = 0.0, qw: float = 1.0,
                                  num_cameras: int = 4):
        """
        Set pose directly for simulation/testing.

        Args:
            x, y, z: Position
            qx, qy, qz, qw: Quaternion orientation
            num_cameras: Simulated number of detecting cameras
        """
        if not TRACKER_AVAILABLE:
            # Create a minimal pose object for simulation
            from types import SimpleNamespace
            pose = SimpleNamespace()
            pose.marker_id = self.marker_id
            pose.x = x
            pose.y = y
            pose.z = z
            pose.orientation = (qx, qy, qz, qw)
            pose.confidence = 1.0
            pose.detecting_cameras = [f'cam_{i}' for i in range(num_cameras)]
            pose.to_array = lambda: np.array([x, y, z])
            pose.num_cameras = lambda: num_cameras
            pose.timestamp = time.time()
            self.last_pose = pose


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
