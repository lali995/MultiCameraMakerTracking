"""
Factory for creating Open3D geometry primitives.
"""
import numpy as np
from typing import List, Tuple, Optional, TYPE_CHECKING

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: Open3D not installed. Install with: pip install open3d")

from ..core.data_types import CameraPose


# Color palette for markers
MARKER_COLORS = [
    [1.0, 0.0, 0.0],    # Red
    [0.0, 1.0, 0.0],    # Green
    [0.0, 0.0, 1.0],    # Blue
    [1.0, 1.0, 0.0],    # Yellow
    [1.0, 0.0, 1.0],    # Magenta
    [0.0, 1.0, 1.0],    # Cyan
    [1.0, 0.5, 0.0],    # Orange
    [0.5, 0.0, 1.0],    # Purple
]


class GeometryFactory:
    """Factory for creating Open3D geometries."""

    @staticmethod
    def create_marker_sphere(
        marker_id: int,
        position: np.ndarray = None,
        radius: float = 0.05
    ) -> 'o3d.geometry.TriangleMesh':
        """
        Create a colored sphere for a marker.

        Args:
            marker_id: Marker ID (used to select color)
            position: Initial position [x, y, z]
            radius: Sphere radius

        Returns:
            Open3D TriangleMesh sphere
        """
        if not HAS_OPEN3D:
            return None

        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
        color = MARKER_COLORS[marker_id % len(MARKER_COLORS)]
        sphere.paint_uniform_color(color)
        sphere.compute_vertex_normals()

        if position is not None:
            sphere.translate(position)

        return sphere

    @staticmethod
    def create_camera_frustum(
        pose: CameraPose,
        scale: float = 0.2,
        color: List[float] = None
    ) -> 'o3d.geometry.LineSet':
        """
        Create a camera frustum visualization.

        Args:
            pose: Camera pose
            scale: Size of the frustum
            color: RGB color [0-1]

        Returns:
            Open3D LineSet representing the frustum
        """
        if not HAS_OPEN3D:
            return None

        if color is None:
            color = [0.8, 0.2, 0.2]  # Default red

        # Frustum points in camera space
        # Camera looks along +Z (OpenCV convention)
        near = scale
        far = scale * 2
        fov_scale = 0.6

        points_camera = np.array([
            [0, 0, 0],  # Camera center
            [-fov_scale * near, -fov_scale * near, near],  # Near plane corners
            [fov_scale * near, -fov_scale * near, near],
            [fov_scale * near, fov_scale * near, near],
            [-fov_scale * near, fov_scale * near, near],
        ])

        # Build rotation matrix from direction
        z_axis = pose.direction / np.linalg.norm(pose.direction)

        # Choose an up vector (prefer Y-up)
        up = np.array([0, 1, 0])
        if abs(np.dot(z_axis, up)) > 0.99:
            up = np.array([1, 0, 0])

        x_axis = np.cross(up, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        rotation = np.column_stack([x_axis, y_axis, z_axis])

        # Transform points to world space
        points_world = (rotation @ points_camera.T).T + pose.position

        # Create line set
        lines = [
            [0, 1], [0, 2], [0, 3], [0, 4],  # From center to corners
            [1, 2], [2, 3], [3, 4], [4, 1],  # Near plane edges
        ]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points_world)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector([color] * len(lines))

        return line_set

    @staticmethod
    def create_coordinate_frame(
        size: float = 0.5,
        origin: np.ndarray = None
    ) -> 'o3d.geometry.TriangleMesh':
        """
        Create XYZ coordinate axes at origin.

        Args:
            size: Length of axes
            origin: Position of origin

        Returns:
            Open3D TriangleMesh coordinate frame
        """
        if not HAS_OPEN3D:
            return None

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)

        if origin is not None:
            frame.translate(origin)

        return frame

    @staticmethod
    def create_ground_plane(
        size: float = 4.0,
        color: List[float] = None,
        y_level: float = 0.0
    ) -> 'o3d.geometry.TriangleMesh':
        """
        Create a ground plane at Y=0.

        Args:
            size: Half-size of the plane
            color: RGB color
            y_level: Y coordinate of the plane

        Returns:
            Open3D TriangleMesh plane
        """
        if not HAS_OPEN3D:
            return None

        if color is None:
            color = [0.8, 0.8, 0.8]  # Light gray

        # Create a thin box as the ground plane
        plane = o3d.geometry.TriangleMesh.create_box(
            width=size * 2,
            height=0.01,
            depth=size * 2
        )
        plane.translate([-size, y_level - 0.01, -size])
        plane.paint_uniform_color(color)
        plane.compute_vertex_normals()

        return plane

    @staticmethod
    def create_grid(
        size: float = 4.0,
        divisions: int = 10,
        color: List[float] = None,
        y_level: float = 0.0
    ) -> 'o3d.geometry.LineSet':
        """
        Create a grid on the ground plane.

        Args:
            size: Half-size of the grid
            divisions: Number of divisions per side
            color: RGB color
            y_level: Y coordinate

        Returns:
            Open3D LineSet grid
        """
        if not HAS_OPEN3D:
            return None

        if color is None:
            color = [0.5, 0.5, 0.5]  # Gray

        points = []
        lines = []
        step = (size * 2) / divisions

        # Lines parallel to X axis
        for i in range(divisions + 1):
            z = -size + i * step
            points.append([-size, y_level, z])
            points.append([size, y_level, z])
            lines.append([len(points) - 2, len(points) - 1])

        # Lines parallel to Z axis
        for i in range(divisions + 1):
            x = -size + i * step
            points.append([x, y_level, -size])
            points.append([x, y_level, size])
            lines.append([len(points) - 2, len(points) - 1])

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector([color] * len(lines))

        return line_set

    @staticmethod
    def create_trajectory_line(
        points: List[np.ndarray],
        color: List[float] = None
    ) -> 'o3d.geometry.LineSet':
        """
        Create a line showing marker trajectory.

        Args:
            points: List of 3D points
            color: RGB color

        Returns:
            Open3D LineSet trajectory
        """
        if not HAS_OPEN3D or len(points) < 2:
            return None

        if color is None:
            color = [0.0, 0.5, 1.0]  # Light blue

        lines = [[i, i + 1] for i in range(len(points) - 1)]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector([color] * len(lines))

        return line_set

    @staticmethod
    def create_marker_axes(
        size: float = 0.1,
        position: np.ndarray = None,
        orientation: Tuple[float, float, float, float] = None
    ) -> 'o3d.geometry.LineSet':
        """
        Create XYZ axes for a marker showing its orientation.

        Args:
            size: Length of each axis
            position: Position [x, y, z]
            orientation: Quaternion (x, y, z, w)

        Returns:
            Open3D LineSet with colored axes (X=red, Y=green, Z=blue)
        """
        if not HAS_OPEN3D:
            return None

        # Axis endpoints in local frame
        points = np.array([
            [0, 0, 0],      # Origin
            [size, 0, 0],   # X axis end
            [0, size, 0],   # Y axis end
            [0, 0, size],   # Z axis end
        ])

        # Apply rotation if provided
        if orientation is not None:
            rotation_matrix = GeometryFactory.quaternion_to_rotation_matrix(orientation)
            points[1:] = (rotation_matrix @ points[1:].T).T

        # Apply translation if provided
        if position is not None:
            points = points + position

        # Create line set
        lines = [
            [0, 1],  # X axis
            [0, 2],  # Y axis
            [0, 3],  # Z axis
        ]

        colors = [
            [1.0, 0.0, 0.0],  # X = Red
            [0.0, 1.0, 0.0],  # Y = Green
            [0.0, 0.0, 1.0],  # Z = Blue
        ]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)

        return line_set

    @staticmethod
    def quaternion_to_rotation_matrix(q: Tuple[float, float, float, float]) -> np.ndarray:
        """
        Convert quaternion (x, y, z, w) to 3x3 rotation matrix.
        """
        x, y, z, w = q

        # Normalize quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm

        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])

        return R

    @staticmethod
    def update_marker_axes(
        axes: 'o3d.geometry.LineSet',
        size: float,
        position: np.ndarray,
        orientation: Tuple[float, float, float, float]
    ) -> None:
        """
        Update existing marker axes geometry with new position/orientation.

        Args:
            axes: Existing LineSet to update
            size: Length of each axis
            position: New position [x, y, z]
            orientation: New quaternion (x, y, z, w)
        """
        if not HAS_OPEN3D or axes is None:
            return

        # Axis endpoints in local frame
        points = np.array([
            [0, 0, 0],      # Origin
            [size, 0, 0],   # X axis end
            [0, size, 0],   # Y axis end
            [0, 0, size],   # Z axis end
        ])

        # Apply rotation
        if orientation is not None:
            rotation_matrix = GeometryFactory.quaternion_to_rotation_matrix(orientation)
            points[1:] = (rotation_matrix @ points[1:].T).T

        # Apply translation
        if position is not None:
            points = points + position

        # Update the line set points
        axes.points = o3d.utility.Vector3dVector(points)
