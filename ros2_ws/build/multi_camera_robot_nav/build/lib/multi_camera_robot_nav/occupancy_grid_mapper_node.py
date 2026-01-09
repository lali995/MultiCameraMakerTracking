#!/usr/bin/env python3
"""
Occupancy Grid Mapper Node

Converts the stitched point cloud from ceiling cameras
into a 2D occupancy grid for robot navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import numpy as np
from typing import Optional
import struct


class OccupancyGridMapperNode(Node):
    """
    Converts point cloud to 2D occupancy grid for Nav2.

    Subscribes:
        /filtered_cloud (PointCloud2): Filtered point cloud in world frame

    Publishes:
        /map (OccupancyGrid): 2D occupancy grid for navigation
    """

    def __init__(self):
        super().__init__('occupancy_grid_mapper_node')

        # Declare parameters
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('resolution', 0.05)       # 5cm per cell
        self.declare_parameter('map_width', 8.0)         # 8m width
        self.declare_parameter('map_height', 8.0)        # 8m height
        self.declare_parameter('origin_x', -4.0)         # Origin X
        self.declare_parameter('origin_y', -4.0)         # Origin Y
        self.declare_parameter('obstacle_height_min', 0.1)   # Min obstacle height
        self.declare_parameter('obstacle_height_max', 1.5)   # Max obstacle height
        self.declare_parameter('free_threshold', 3)      # Points needed for free
        self.declare_parameter('occupied_threshold', 5)  # Points needed for occupied
        self.declare_parameter('publish_rate', 5.0)      # Hz
        self.declare_parameter('static_map', True)        # Generate map once at startup
        self.declare_parameter('initialization_time', 3.0)  # Seconds to collect data before finalizing

        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.resolution = self.get_parameter('resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.obstacle_height_min = self.get_parameter('obstacle_height_min').value
        self.obstacle_height_max = self.get_parameter('obstacle_height_max').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.static_map = self.get_parameter('static_map').value
        self.initialization_time = self.get_parameter('initialization_time').value

        # Calculate grid dimensions
        self.grid_width = int(self.map_width / self.resolution)
        self.grid_height = int(self.map_height / self.resolution)

        # Persistent occupancy counts
        self.obstacle_counts = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        self.observation_counts = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)

        # Static map state tracking
        self.map_finalized = False
        self.final_grid = None
        self.start_time = None  # Will be set on first cloud received

        # QoS profile - must match publisher (RELIABLE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscriber
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/filtered_cloud',
            self.cloud_callback,
            qos
        )

        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )

        # Latest cloud
        self.latest_cloud: Optional[PointCloud2] = None

        # Timer for periodic map publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_map)

        mode = "static (one-time)" if self.static_map else "dynamic (continuous)"
        self.get_logger().info(
            f'Occupancy Grid Mapper started: {self.grid_width}x{self.grid_height} grid, '
            f'{self.resolution}m resolution, mode: {mode}'
        )
        if self.static_map:
            self.get_logger().info(
                f'Map will be finalized after {self.initialization_time}s of data collection'
            )

    def cloud_callback(self, msg: PointCloud2):
        """Store latest point cloud."""
        self.latest_cloud = msg

    def world_to_grid(self, x: float, y: float) -> tuple:
        """Convert world coordinates to grid cell indices."""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def is_in_grid(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are valid."""
        return 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height

    def update_from_cloud(self, cloud_msg: PointCloud2):
        """Update occupancy grid from point cloud."""
        points = self.pointcloud2_to_xyz(cloud_msg)
        if points is None or len(points) == 0:
            return

        # Filter points by obstacle height
        height_mask = (
            (points[:, 2] >= self.obstacle_height_min) &
            (points[:, 2] <= self.obstacle_height_max)
        )
        obstacle_points = points[height_mask]

        # Clear temporary counts for this update
        temp_counts = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)

        # Project points to grid
        for pt in obstacle_points:
            gx, gy = self.world_to_grid(pt[0], pt[1])
            if self.is_in_grid(gx, gy):
                temp_counts[gy, gx] += 1

        # Update persistent counts (decay only in dynamic mode)
        if not self.static_map:
            self.obstacle_counts = (self.obstacle_counts * 0.9).astype(np.int32)
        self.obstacle_counts += temp_counts

        # Update observation counts
        observed_cells = temp_counts > 0
        self.observation_counts[observed_cells] += 1

    def generate_occupancy_grid(self) -> np.ndarray:
        """Generate occupancy grid from counts."""
        grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)

        # Mark observed cells
        observed = self.observation_counts > 0

        # Free cells: observed but few obstacles
        free_mask = observed & (self.obstacle_counts < self.free_threshold)
        grid[free_mask] = 0

        # Occupied cells: many obstacles
        occupied_mask = self.obstacle_counts >= self.occupied_threshold
        grid[occupied_mask] = 100

        # Cells with some obstacles: partially occupied
        partial_mask = observed & (self.obstacle_counts >= self.free_threshold) & ~occupied_mask
        grid[partial_mask] = 50

        return grid

    def publish_map(self):
        """Update and publish occupancy grid."""
        # If map is finalized (static mode), just publish the cached grid
        if self.map_finalized and self.final_grid is not None:
            grid = self.final_grid
        else:
            # Process new cloud data
            if self.latest_cloud is not None:
                # Start timing on first cloud received
                if self.start_time is None:
                    self.start_time = self.get_clock().now()
                    self.get_logger().info('First point cloud received, starting map initialization...')

                self.update_from_cloud(self.latest_cloud)
                self.latest_cloud = None  # Clear to avoid re-processing

            # Check if we should finalize the map (static mode only)
            if self.static_map and not self.map_finalized and self.start_time is not None:
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                if elapsed >= self.initialization_time:
                    # Finalize the map
                    self.final_grid = self.generate_occupancy_grid()
                    self.map_finalized = True
                    # Unsubscribe from point cloud to save resources
                    self.destroy_subscription(self.cloud_sub)
                    self.get_logger().info(
                        f'Map finalized after {elapsed:.1f}s. Static map will be used from now on.'
                    )

            # Generate grid (dynamic mode or still initializing)
            grid = self.final_grid if self.map_finalized else self.generate_occupancy_grid()

        # Create message
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info = MapMetaData()
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height

        # Map origin
        msg.info.origin = Pose()
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flatten grid to 1D array (row-major order)
        msg.data = grid.flatten().tolist()

        self.map_pub.publish(msg)

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


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
