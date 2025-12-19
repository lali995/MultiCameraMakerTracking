#!/usr/bin/env python3
"""
TF Broadcaster Node

Broadcasts robot transforms from marker tracking.
Publishes transforms: world -> odom -> base_link
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry


class TFBroadcasterNode(Node):
    """
    ROS2 node that broadcasts TF transforms for the robot.

    Subscribes:
        /robot/pose (PoseStamped): Robot pose in world frame
        /robot/odom (Odometry): Full odometry

    Broadcasts:
        world -> odom: Static identity transform
        odom -> base_link: Dynamic robot pose transform
    """

    def __init__(self):
        super().__init__('tf_broadcaster_node')

        # Declare parameters
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')

        # Get parameters
        self.world_frame = self.get_parameter('world_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Broadcast static world -> odom transform (identity)
        self.broadcast_static_transforms()

        # Subscribe to pose and odometry
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'TF Broadcaster started. Publishing {self.world_frame} -> '
            f'{self.odom_frame} -> {self.robot_frame}'
        )

    def broadcast_static_transforms(self):
        """Broadcast static transforms."""
        # world -> odom (identity transform - they are the same in external tracking)
        world_to_odom = TransformStamped()
        world_to_odom.header.stamp = self.get_clock().now().to_msg()
        world_to_odom.header.frame_id = self.world_frame
        world_to_odom.child_frame_id = self.odom_frame
        world_to_odom.transform.translation.x = 0.0
        world_to_odom.transform.translation.y = 0.0
        world_to_odom.transform.translation.z = 0.0
        world_to_odom.transform.rotation.x = 0.0
        world_to_odom.transform.rotation.y = 0.0
        world_to_odom.transform.rotation.z = 0.0
        world_to_odom.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(world_to_odom)
        self.get_logger().info('Published static transform: world -> odom')

    def pose_callback(self, msg: PoseStamped):
        """Handle incoming pose and broadcast TF."""
        self.broadcast_robot_transform(msg)

    def odom_callback(self, msg: Odometry):
        """Handle incoming odometry and broadcast TF."""
        # Create PoseStamped from Odometry
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.broadcast_robot_transform(pose_msg)

    def broadcast_robot_transform(self, pose_msg: PoseStamped):
        """Broadcast odom -> base_link transform."""
        t = TransformStamped()
        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.robot_frame

        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z

        t.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
