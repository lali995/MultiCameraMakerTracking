#!/usr/bin/env python3
"""
Point Cloud Stitcher Node

Combines point clouds from multiple ceiling-mounted depth cameras
into a single unified point cloud for obstacle mapping.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
import numpy as np
from typing import Dict, Optional, List
import struct
import threading

try:
    import tf2_ros
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class PointCloudStitcherNode(Node):
    """
    Combines point clouds from multiple cameras into world frame.

    Subscribes:
        /cameras/camera_N/points (PointCloud2): Point clouds from each camera

    Publishes:
        /combined_cloud (PointCloud2): Stitched point cloud in world frame
        /filtered_cloud (PointCloud2): Filtered for floor plane and outliers
    """

    def __init__(self):
        super().__init__('point_cloud_stitcher_node')

        # Declare parameters
        self.declare_parameter('num_cameras', 4)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('min_height', 0.05)  # Filter floor
        self.declare_parameter('max_height', 2.0)   # Filter ceiling
        self.declare_parameter('max_range', 5.0)    # Max point distance

        # Get parameters
        self.num_cameras = self.get_parameter('num_cameras').value
        self.world_frame = self.get_parameter('world_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.max_range = self.get_parameter('max_range').value

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Point cloud buffers (one per camera)
        self.cloud_buffers: Dict[str, Optional[PointCloud2]] = {}
        self.buffer_lock = threading.Lock()

        # Subscribers for each camera
        self.cloud_subs = []
        for i in range(1, self.num_cameras + 1):
            topic = f'/cameras/camera_{i}/points'
            sub = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, cam_id=f'camera_{i}': self.cloud_callback(msg, cam_id),
                qos
            )
            self.cloud_subs.append(sub)
            self.cloud_buffers[f'camera_{i}'] = None
            self.get_logger().info(f'Subscribed to {topic}')

        # Publishers
        self.combined_pub = self.create_publisher(PointCloud2, '/combined_cloud', 10)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_cloud', 10)

        # TF buffer for transforms
        if TF2_AVAILABLE:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.get_logger().warn('tf2_ros not available, using static transforms')

        # Camera transforms (static, from world file)
        # Format: position (x, y, z), rotation (roll, pitch, yaw) in radians
        self.camera_poses = {
            'camera_1': {'pos': [1.8, 1.8, 2.4], 'rot': [0, 0.785, -2.356]},   # NE
            'camera_2': {'pos': [-1.8, 1.8, 2.4], 'rot': [0, 0.785, -0.785]},  # NW
            'camera_3': {'pos': [-1.8, -1.8, 2.4], 'rot': [0, 0.785, 0.785]},  # SW
            'camera_4': {'pos': [1.8, -1.8, 2.4], 'rot': [0, 0.785, 2.356]},   # SE
        }

        # Timer for periodic stitching
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.stitch_clouds)

        self.get_logger().info(
            f'Point Cloud Stitcher started with {self.num_cameras} cameras'
        )

    def cloud_callback(self, msg: PointCloud2, camera_id: str):
        """Store incoming point cloud in buffer."""
        with self.buffer_lock:
            self.cloud_buffers[camera_id] = msg

    def stitch_clouds(self):
        """Combine all point clouds into world frame."""
        with self.buffer_lock:
            # Get available clouds
            available_clouds = {
                k: v for k, v in self.cloud_buffers.items() if v is not None
            }

        if not available_clouds:
            return

        combined_points = []

        for camera_id, cloud_msg in available_clouds.items():
            # Get camera transform
            transform = self.get_camera_transform(camera_id)
            if transform is None:
                continue

            # Extract points from cloud
            points = self.pointcloud2_to_xyz(cloud_msg)
            if points is None or len(points) == 0:
                continue

            # Transform points to world frame
            transformed = self.transform_points(points, transform)
            combined_points.append(transformed)

        if not combined_points:
            return

        # Combine all points
        all_points = np.vstack(combined_points)

        # Create combined cloud message
        combined_msg = self.xyz_to_pointcloud2(all_points, self.world_frame)
        self.combined_pub.publish(combined_msg)

        # Filter and publish
        filtered_points = self.filter_points(all_points)
        if len(filtered_points) > 0:
            filtered_msg = self.xyz_to_pointcloud2(filtered_points, self.world_frame)
            self.filtered_pub.publish(filtered_msg)

    def get_camera_transform(self, camera_id: str) -> Optional[np.ndarray]:
        """Get 4x4 transform matrix for camera."""
        # Try TF first
        if self.tf_buffer is not None:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    f'{camera_id}_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                return self.transform_to_matrix(trans)
            except Exception:
                pass

        # Fall back to static transforms
        if camera_id in self.camera_poses:
            pose = self.camera_poses[camera_id]
            return self.pose_to_matrix(pose['pos'], pose['rot'])

        return None

    def pose_to_matrix(self, pos: List[float], rot: List[float]) -> np.ndarray:
        """Convert position and euler angles to 4x4 transform matrix."""
        roll, pitch, yaw = rot

        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R = Rz @ Ry @ Rx

        # Build 4x4 matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos

        return T

    def transform_to_matrix(self, trans: TransformStamped) -> np.ndarray:
        """Convert ROS transform to 4x4 matrix."""
        t = trans.transform.translation
        q = trans.transform.rotation

        # Quaternion to rotation matrix
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]

        return T

    def transform_points(self, points: np.ndarray, T: np.ndarray) -> np.ndarray:
        """Apply 4x4 transform to points."""
        # Add homogeneous coordinate
        ones = np.ones((len(points), 1))
        points_h = np.hstack([points, ones])

        # Transform
        transformed = (T @ points_h.T).T

        return transformed[:, :3]

    def filter_points(self, points: np.ndarray) -> np.ndarray:
        """Filter points by height and range."""
        if len(points) == 0:
            return points

        # Height filter (Z axis in world frame)
        height_mask = (points[:, 2] >= self.min_height) & (points[:, 2] <= self.max_height)

        # Range filter (distance from origin in XY plane)
        xy_distance = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        range_mask = xy_distance <= self.max_range

        # Combine masks
        mask = height_mask & range_mask

        return points[mask]

    def pointcloud2_to_xyz(self, cloud_msg: PointCloud2) -> Optional[np.ndarray]:
        """Extract XYZ points from PointCloud2 message."""
        if cloud_msg.width * cloud_msg.height == 0:
            return None

        # Find x, y, z field offsets
        field_names = {f.name: f for f in cloud_msg.fields}
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            return None

        x_offset = field_names['x'].offset
        y_offset = field_names['y'].offset
        z_offset = field_names['z'].offset

        # Parse points
        points = []
        data = cloud_msg.data
        point_step = cloud_msg.point_step

        for i in range(cloud_msg.width * cloud_msg.height):
            start = i * point_step
            x = struct.unpack_from('f', data, start + x_offset)[0]
            y = struct.unpack_from('f', data, start + y_offset)[0]
            z = struct.unpack_from('f', data, start + z_offset)[0]

            # Skip NaN/inf points
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])

        return np.array(points) if points else None

    def xyz_to_pointcloud2(self, points: np.ndarray, frame_id: str) -> PointCloud2:
        """Create PointCloud2 message from XYZ points."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = len(points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # Pack points into bytes
        buffer = bytearray(msg.row_step * msg.height)
        for i, pt in enumerate(points):
            struct.pack_into('fff', buffer, i * 12, pt[0], pt[1], pt[2])

        msg.data = bytes(buffer)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudStitcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
