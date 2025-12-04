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

from ..core.data_types import CameraPose, MarkerPose
from .geometry_factory import GeometryFactory, MARKER_COLORS


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
        self.ground_grid = None

        # Factory for creating geometries
        self.factory = GeometryFactory()

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

        # Ground grid
        self.ground_grid = self.factory.create_grid(size=2.0, divisions=8, y_level=0.0)
        if self.ground_grid:
            geometries.append(self.ground_grid)

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
        if self.ground_grid:
            geometries.append(self.ground_grid)

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
