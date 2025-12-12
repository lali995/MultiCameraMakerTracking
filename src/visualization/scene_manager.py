"""
Manages all objects in the 3D scene.
"""
import numpy as np
from typing import Dict, List, Optional
from collections import deque

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

from ..core.data_types import CameraPose, MarkerPose, FusedMarkerPose
from .geometry_factory import GeometryFactory, MARKER_COLORS

# Camera colors for detection lines
CAMERA_COLORS = [
    [1.0, 0.3, 0.3],  # Red
    [0.3, 1.0, 0.3],  # Green
    [0.3, 0.3, 1.0],  # Blue
    [1.0, 1.0, 0.3],  # Yellow
    [1.0, 0.3, 1.0],  # Magenta
    [0.3, 1.0, 1.0],  # Cyan
]


class SceneManager:
    """
    Manages cameras, markers, and other scene objects.
    Handles creation, updates, and tracking of all geometries.
    """

    def __init__(self, show_trajectory: bool = True, trajectory_length: int = 100, show_axes: bool = True):
        """
        Initialize the scene manager.

        Args:
            show_trajectory: Whether to show marker movement trails
            trajectory_length: Number of points to keep in trajectory
            show_axes: Whether to show orientation axes on markers
        """
        self.show_trajectory = show_trajectory
        self.trajectory_length = trajectory_length
        self.show_axes = show_axes
        self.axes_size = 0.1  # Size of marker axes

        # Geometry storage
        self.camera_geometries: Dict[str, o3d.geometry.LineSet] = {}
        self.marker_geometries: Dict[int, o3d.geometry.TriangleMesh] = {}
        self.marker_axes: Dict[int, o3d.geometry.LineSet] = {}
        self.trajectory_points: Dict[int, deque] = {}
        self.trajectory_geometries: Dict[int, o3d.geometry.LineSet] = {}

        # Static scene elements
        self.coordinate_frame = None

        # Factory for creating geometries
        self.factory = GeometryFactory()

        # Multi-camera detection visualization
        # Key: (marker_id, camera_id), Value: LineSet connecting marker to camera
        self.detection_lines: Dict[tuple, 'o3d.geometry.LineSet'] = {}
        # Camera colors for detection lines
        self.camera_colors: Dict[str, List[float]] = {}
        # Camera positions cache
        self.camera_positions: Dict[str, np.ndarray] = {}

    def setup_static_scene(self) -> List:
        """
        Create static scene elements (coordinate frame, grid).

        Returns:
            List of static geometries to add to visualizer
        """
        geometries = []

        # Coordinate frame at origin
        self.coordinate_frame = self.factory.create_coordinate_frame(size=0.3)
        if self.coordinate_frame:
            geometries.append(self.coordinate_frame)

        return geometries

    def add_cameras(self, camera_poses: List[CameraPose]) -> List:
        """
        Add camera frustum visualizations.

        Args:
            camera_poses: List of camera poses

        Returns:
            List of camera geometries to add to visualizer
        """
        geometries = []

        for pose in camera_poses:
            frustum = self.factory.create_camera_frustum(pose, scale=0.15)
            if frustum:
                self.camera_geometries[pose.camera_id] = frustum
                geometries.append(frustum)

        return geometries

    def get_or_create_marker(self, marker_id: int) -> Optional['o3d.geometry.TriangleMesh']:
        """
        Get existing marker geometry or create a new one.

        Args:
            marker_id: Marker ID

        Returns:
            Marker sphere geometry (None if newly created, geometry if exists)
        """
        if marker_id in self.marker_geometries:
            return self.marker_geometries[marker_id]

        # Create new marker
        sphere = self.factory.create_marker_sphere(marker_id)
        if sphere:
            self.marker_geometries[marker_id] = sphere

            # Initialize trajectory
            if self.show_trajectory:
                self.trajectory_points[marker_id] = deque(maxlen=self.trajectory_length)

        return None  # Return None to indicate new geometry needs to be added

    def update_marker_position(
        self,
        marker_id: int,
        position: np.ndarray,
        orientation: tuple = None
    ) -> Optional['o3d.geometry.LineSet']:
        """
        Update a marker's position and orientation.

        Args:
            marker_id: Marker ID
            position: New position [x, y, z]
            orientation: Quaternion (x, y, z, w) or None

        Returns:
            New trajectory geometry if updated, None otherwise
        """
        if marker_id not in self.marker_geometries:
            return None

        sphere = self.marker_geometries[marker_id]

        # Reset to origin then translate to new position
        current_center = sphere.get_center()
        sphere.translate(-current_center)
        sphere.translate(position)

        # Update axes if they exist and orientation is provided
        if self.show_axes and marker_id in self.marker_axes and orientation is not None:
            self.factory.update_marker_axes(
                self.marker_axes[marker_id],
                self.axes_size,
                position,
                orientation
            )

        # Update trajectory
        new_trajectory = None
        if self.show_trajectory:
            self.trajectory_points[marker_id].append(position.copy())

            # Create/update trajectory line
            if len(self.trajectory_points[marker_id]) >= 2:
                points = list(self.trajectory_points[marker_id])
                color = MARKER_COLORS[marker_id % len(MARKER_COLORS)]
                # Fade color for trajectory
                faded_color = [c * 0.5 for c in color]

                new_trajectory = self.factory.create_trajectory_line(points, faded_color)

                # Store for later removal
                old_trajectory = self.trajectory_geometries.get(marker_id)
                self.trajectory_geometries[marker_id] = new_trajectory

                return new_trajectory

        return None

    def get_or_create_marker_axes(self, marker_id: int, position: np.ndarray = None, orientation: tuple = None) -> Optional['o3d.geometry.LineSet']:
        """
        Get existing marker axes or create new ones.

        Args:
            marker_id: Marker ID
            position: Initial position
            orientation: Initial orientation quaternion

        Returns:
            None if axes already exist, new axes geometry if created
        """
        if not self.show_axes:
            return None

        if marker_id in self.marker_axes:
            return None  # Already exists

        # Create new axes
        axes = self.factory.create_marker_axes(
            size=self.axes_size,
            position=position,
            orientation=orientation
        )
        if axes:
            self.marker_axes[marker_id] = axes

        return axes

    def get_marker_geometries(self) -> List:
        """Return all marker geometries."""
        return list(self.marker_geometries.values())

    def get_trajectory_geometries(self) -> List:
        """Return all trajectory geometries."""
        return list(self.trajectory_geometries.values())

    def get_all_geometries(self) -> List:
        """Return all geometries in the scene."""
        geometries = []

        if self.coordinate_frame:
            geometries.append(self.coordinate_frame)

        geometries.extend(self.camera_geometries.values())
        geometries.extend(self.marker_geometries.values())
        geometries.extend(self.trajectory_geometries.values())

        return geometries

    def clear_markers(self) -> None:
        """Clear all marker geometries."""
        self.marker_geometries.clear()
        self.trajectory_points.clear()
        self.trajectory_geometries.clear()

    def clear_trajectories(self) -> None:
        """Clear just the trajectories."""
        for points in self.trajectory_points.values():
            points.clear()
        self.trajectory_geometries.clear()

    # ===== Multi-Camera Detection Visualization =====

    def setup_camera_colors(self, camera_poses: List[CameraPose]) -> None:
        """
        Assign distinct colors to each camera and cache positions.

        Args:
            camera_poses: List of camera poses
        """
        for i, pose in enumerate(camera_poses):
            self.camera_colors[pose.camera_id] = CAMERA_COLORS[i % len(CAMERA_COLORS)]
            self.camera_positions[pose.camera_id] = pose.position.copy()

    def get_camera_color(self, camera_id: str) -> List[float]:
        """Get color assigned to a camera."""
        # Try exact match first
        if camera_id in self.camera_colors:
            return self.camera_colors[camera_id]

        # Try partial match
        for cid, color in self.camera_colors.items():
            if cid in camera_id or camera_id in cid:
                return color

        return [0.5, 0.5, 0.5]  # Default gray

    def get_camera_position(self, camera_id: str) -> Optional[np.ndarray]:
        """Get cached position for a camera."""
        # Try exact match first
        if camera_id in self.camera_positions:
            return self.camera_positions[camera_id]

        # Try partial match
        for cid, pos in self.camera_positions.items():
            if cid in camera_id or camera_id in cid:
                return pos

        return None

    def update_detection_lines(
        self,
        marker_id: int,
        marker_position: np.ndarray,
        detecting_cameras: List[str]
    ) -> tuple:
        """
        Update detection lines from marker to detecting cameras.

        Args:
            marker_id: Marker ID
            marker_position: Current marker position
            detecting_cameras: List of camera IDs currently detecting the marker

        Returns:
            Tuple of (new_lines, updated_lines, stale_lines) for visualizer updates
        """
        if not HAS_OPEN3D:
            return [], [], []

        new_lines = []
        updated_lines = []
        stale_lines = []
        active_keys = set()

        # Create/update lines for detecting cameras
        for camera_id in detecting_cameras:
            cam_pos = self.get_camera_position(camera_id)
            if cam_pos is None:
                continue

            key = (marker_id, camera_id)
            active_keys.add(key)
            color = self.get_camera_color(camera_id)

            if key in self.detection_lines:
                # Update existing line
                line = self.detection_lines[key]
                line.points = o3d.utility.Vector3dVector([marker_position, cam_pos])
                updated_lines.append(line)
            else:
                # Create new line
                line = o3d.geometry.LineSet()
                line.points = o3d.utility.Vector3dVector([marker_position, cam_pos])
                line.lines = o3d.utility.Vector2iVector([[0, 1]])
                line.colors = o3d.utility.Vector3dVector([color])
                self.detection_lines[key] = line
                new_lines.append(line)

        # Find stale lines (marker no longer detected by these cameras)
        for key in list(self.detection_lines.keys()):
            mid, cid = key
            if mid == marker_id and key not in active_keys:
                stale_lines.append(self.detection_lines[key])
                del self.detection_lines[key]

        return new_lines, updated_lines, stale_lines

    def clear_detection_lines_for_marker(self, marker_id: int) -> List:
        """
        Clear all detection lines for a specific marker.

        Args:
            marker_id: Marker ID

        Returns:
            List of removed line geometries
        """
        removed = []
        for key in list(self.detection_lines.keys()):
            mid, cid = key
            if mid == marker_id:
                removed.append(self.detection_lines[key])
                del self.detection_lines[key]
        return removed

    def clear_all_detection_lines(self) -> List:
        """
        Clear all detection lines.

        Returns:
            List of removed line geometries
        """
        removed = list(self.detection_lines.values())
        self.detection_lines.clear()
        return removed

    def get_detection_line_geometries(self) -> List:
        """Return all detection line geometries."""
        return list(self.detection_lines.values())
