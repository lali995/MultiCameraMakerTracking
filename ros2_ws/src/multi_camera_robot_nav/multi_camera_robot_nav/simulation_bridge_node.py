#!/usr/bin/env python3
"""
Simulation Bridge Node

Bridges Gazebo simulation marker detections to the tracking system.
Used for testing and validation in simulation before deploying to real hardware.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from typing import Optional

# Optional import for Gazebo simulation
try:
    from gazebo_msgs.msg import ModelStates
    GAZEBO_AVAILABLE = True
except ImportError:
    GAZEBO_AVAILABLE = False
    ModelStates = None  # Placeholder


class SimulationBridgeNode(Node):
    """
    Bridge node that extracts robot pose from Gazebo ground truth.

    In simulation, this node:
    1. Subscribes to Gazebo model states for ground truth
    2. Optionally adds noise to simulate real sensor behavior
    3. Publishes to the same topics as the real marker tracker

    Topics Subscribed:
        /gazebo/model_states (ModelStates): Gazebo ground truth

    Topics Published:
        /robot/pose (PoseStamped): Robot pose
        /robot/pose_cov (PoseWithCovarianceStamped): Pose with covariance
        /robot/odom (Odometry): Full odometry
        /robot/velocity (TwistStamped): Robot velocity
    """

    def __init__(self):
        super().__init__('simulation_bridge_node')

        # Declare parameters
        self.declare_parameter('robot_model_name', 'robot')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('add_noise', True)
        self.declare_parameter('position_noise_std', 0.01)  # 1cm std
        self.declare_parameter('orientation_noise_std', 0.02)  # ~1 degree std
        self.declare_parameter('publish_rate', 30.0)

        # Get parameters
        self.robot_model_name = self.get_parameter('robot_model_name').value
        self.world_frame = self.get_parameter('world_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.add_noise = self.get_parameter('add_noise').value
        self.position_noise_std = self.get_parameter('position_noise_std').value
        self.orientation_noise_std = self.get_parameter('orientation_noise_std').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot/pose_cov', 10)
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/robot/velocity', 10)

        # Subscriber to Gazebo
        if GAZEBO_AVAILABLE:
            self.model_states_sub = self.create_subscription(
                ModelStates,
                '/gazebo/model_states',
                self.model_states_callback,
                10
            )
            self.get_logger().info(
                f'Simulation Bridge started. Tracking model: {self.robot_model_name}'
            )
        else:
            self.model_states_sub = None
            self.get_logger().warn(
                'gazebo_msgs not available. Install ros-humble-gazebo-ros-pkgs for simulation.'
            )

        # Velocity estimation from pose history
        self.last_pose = None
        self.last_time = None

    def model_states_callback(self, msg):
        """Process Gazebo model states to extract robot pose."""
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            self.get_logger().warn(
                f'Robot model "{self.robot_model_name}" not found in Gazebo',
                throttle_duration_sec=5.0
            )
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        now = self.get_clock().now()
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.world_frame

        # Add noise if enabled
        if self.add_noise:
            pose = self._add_pose_noise(pose)
            twist = self._add_twist_noise(twist)

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose = pose
        self.pose_pub.publish(pose_msg)

        # Publish PoseWithCovarianceStamped
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header = header
        pose_cov_msg.pose.pose = pose

        # Covariance based on simulated noise
        pos_var = self.position_noise_std ** 2
        ori_var = self.orientation_noise_std ** 2
        pose_cov_msg.pose.covariance = [
            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, pos_var, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, ori_var, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, ori_var, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, ori_var,
        ]
        self.pose_cov_pub.publish(pose_cov_msg)

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.child_frame_id = self.robot_frame
        odom_msg.pose = pose_cov_msg.pose
        odom_msg.twist.twist = twist
        self.odom_pub.publish(odom_msg)

        # Publish TwistStamped
        velocity_msg = TwistStamped()
        velocity_msg.header = header
        velocity_msg.twist = twist
        self.velocity_pub.publish(velocity_msg)

    def _add_pose_noise(self, pose):
        """Add Gaussian noise to pose to simulate real sensor noise."""
        from geometry_msgs.msg import Pose
        noisy = Pose()

        # Position noise
        noisy.position.x = pose.position.x + np.random.normal(0, self.position_noise_std)
        noisy.position.y = pose.position.y + np.random.normal(0, self.position_noise_std)
        noisy.position.z = pose.position.z + np.random.normal(0, self.position_noise_std)

        # Orientation noise (add small perturbation to quaternion)
        q = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        noise = np.random.normal(0, self.orientation_noise_std, 4)
        q_noisy = q + noise
        q_noisy = q_noisy / np.linalg.norm(q_noisy)  # Re-normalize

        noisy.orientation.x = q_noisy[0]
        noisy.orientation.y = q_noisy[1]
        noisy.orientation.z = q_noisy[2]
        noisy.orientation.w = q_noisy[3]

        return noisy

    def _add_twist_noise(self, twist):
        """Add noise to twist/velocity."""
        from geometry_msgs.msg import Twist
        noisy = Twist()

        velocity_noise_std = 0.01  # m/s

        noisy.linear.x = twist.linear.x + np.random.normal(0, velocity_noise_std)
        noisy.linear.y = twist.linear.y + np.random.normal(0, velocity_noise_std)
        noisy.linear.z = twist.linear.z + np.random.normal(0, velocity_noise_std)

        angular_noise_std = 0.02  # rad/s
        noisy.angular.x = twist.angular.x + np.random.normal(0, angular_noise_std)
        noisy.angular.y = twist.angular.y + np.random.normal(0, angular_noise_std)
        noisy.angular.z = twist.angular.z + np.random.normal(0, angular_noise_std)

        return noisy


def main(args=None):
    rclpy.init(args=args)
    node = SimulationBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
