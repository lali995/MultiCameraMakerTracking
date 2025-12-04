"""
Real-time Open3D viewer for marker tracking visualization.
"""
import numpy as np
from typing import List, Dict, Optional, Callable
import threading

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: Open3D not installed. Install with: pip install open3d")

from ..core.message_bus import MessageBus
from ..core.data_types import CameraPose, MarkerPose, TrackerStatus
from ..core.timing import RateController, FPSCounter
from .scene_manager import SceneManager
from .geometry_factory import GeometryFactory


class MarkerViewer:
    """
    Real-time 3D viewer for ArUco marker tracking.

    Uses Open3D for visualization with a main loop that:
    1. Processes incoming marker poses from message bus
    2. Updates marker positions in the scene
    3. Renders the updated scene

    Keyboard controls:
        Q/ESC: Quit
        P: Pause/Resume
        R: Reset view
        C: Clear trajectories
        T: Toggle trajectories
    """

    def __init__(
        self,
        message_bus: MessageBus,
        window_title: str = "ArUco Marker Tracking",
        window_width: int = 1280,
        window_height: int = 720,
        show_trajectory: bool = True
    ):
        """
        Initialize the viewer.

        Args:
            message_bus: Message bus for receiving marker poses
            window_title: Window title
            window_width: Window width in pixels
            window_height: Window height in pixels
            show_trajectory: Whether to show marker movement trails
        """
        if not HAS_OPEN3D:
            raise ImportError("Open3D is required. Install with: pip install open3d")

        self.bus = message_bus
        self.window_title = window_title
        self.window_width = window_width
        self.window_height = window_height

        # Visualization state
        self._running = False
        self._paused = False
        self._show_trajectory = show_trajectory

        # Open3D visualizer
        self.vis: o3d.visualization.VisualizerWithKeyCallback = None

        # Scene management
        self.scene = SceneManager(show_trajectory=show_trajectory)

        # Pending updates from message bus (thread-safe)
        self._pending_markers: Dict[int, MarkerPose] = {}
        self._pending_lock = threading.Lock()

        # Track which geometries have been added
        self._added_marker_ids: set = set()
        self._added_axes_ids: set = set()
        self._current_trajectory_geometries: Dict[int, o3d.geometry.LineSet] = {}

        # FPS tracking
        self._fps_counter = FPSCounter()
        self._tracker_fps = 0.0

    def setup(self, camera_poses: List[CameraPose]) -> None:
        """
        Initialize the visualization window and add static geometry.

        Args:
            camera_poses: List of camera poses to display
        """
        # Create visualizer
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(
            window_name=self.window_title,
            width=self.window_width,
            height=self.window_height
        )

        # Register keyboard callbacks
        self._register_key_callbacks()

        # Add static scene elements
        static_geoms = self.scene.setup_static_scene()
        for geom in static_geoms:
            self.vis.add_geometry(geom)

        # Add camera frustums
        camera_geoms = self.scene.add_cameras(camera_poses)
        for geom in camera_geoms:
            self.vis.add_geometry(geom)

        # Subscribe to marker poses
        self.bus.subscribe('/markers/poses', self._on_marker_pose)
        self.bus.subscribe('/tracker/status', self._on_tracker_status)

        # Set up camera view
        self._setup_camera_view()

    def _register_key_callbacks(self) -> None:
        """Register keyboard controls."""

        def quit_callback(vis):
            self._running = False
            return False

        def pause_callback(vis):
            self._paused = not self._paused
            status = "PAUSED" if self._paused else "RUNNING"
            print(f"Viewer: {status}")
            return False

        def reset_view_callback(vis):
            self._setup_camera_view()
            return False

        def clear_trajectories_callback(vis):
            # Remove trajectory geometries
            for geom in self._current_trajectory_geometries.values():
                self.vis.remove_geometry(geom, reset_bounding_box=False)
            self._current_trajectory_geometries.clear()
            self.scene.clear_trajectories()
            print("Trajectories cleared")
            return False

        def toggle_trajectory_callback(vis):
            self._show_trajectory = not self._show_trajectory
            self.scene.show_trajectory = self._show_trajectory
            if not self._show_trajectory:
                # Clear existing trajectories
                for geom in self._current_trajectory_geometries.values():
                    self.vis.remove_geometry(geom, reset_bounding_box=False)
                self._current_trajectory_geometries.clear()
            status = "ON" if self._show_trajectory else "OFF"
            print(f"Trajectories: {status}")
            return False

        # Register callbacks
        self.vis.register_key_callback(ord('Q'), quit_callback)
        self.vis.register_key_callback(256, quit_callback)  # ESC
        self.vis.register_key_callback(ord('P'), pause_callback)
        self.vis.register_key_callback(ord('R'), reset_view_callback)
        self.vis.register_key_callback(ord('C'), clear_trajectories_callback)
        self.vis.register_key_callback(ord('T'), toggle_trajectory_callback)

    def _setup_camera_view(self) -> None:
        """Set up the initial camera view."""
        ctr = self.vis.get_view_control()

        # Set view to look at origin from an angle
        ctr.set_lookat([0, 0.5, 0])  # Look at center of scene
        ctr.set_up([0, 1, 0])  # Y is up
        ctr.set_front([1, 0.5, 1])  # View from front-right
        ctr.set_zoom(0.5)

    def _on_marker_pose(self, pose: MarkerPose) -> None:
        """Callback when a marker pose is received."""
        with self._pending_lock:
            self._pending_markers[pose.marker_id] = pose

    def _on_tracker_status(self, status: TrackerStatus) -> None:
        """Callback when tracker status is received."""
        self._tracker_fps = status.fps

    def _process_marker_updates(self) -> None:
        """Process pending marker updates and update scene."""
        with self._pending_lock:
            markers_to_process = dict(self._pending_markers)
            self._pending_markers.clear()

        for marker_id, pose in markers_to_process.items():
            position = np.array([pose.x, pose.y, pose.z])
            orientation = pose.orientation

            # Check if marker geometry exists
            if marker_id not in self._added_marker_ids:
                # Create and add new marker sphere
                self.scene.get_or_create_marker(marker_id)
                sphere = self.scene.marker_geometries.get(marker_id)
                if sphere:
                    sphere.translate(position)
                    self.vis.add_geometry(sphere, reset_bounding_box=False)
                    self._added_marker_ids.add(marker_id)

            # Check if marker axes exist
            if marker_id not in self._added_axes_ids:
                # Create and add marker axes
                axes = self.scene.get_or_create_marker_axes(marker_id, position, orientation)
                if axes:
                    self.vis.add_geometry(axes, reset_bounding_box=False)
                    self._added_axes_ids.add(marker_id)

            # Update position and orientation
            new_trajectory = self.scene.update_marker_position(marker_id, position, orientation)

            # Update trajectory geometry
            if self._show_trajectory and new_trajectory is not None:
                # Remove old trajectory
                old_traj = self._current_trajectory_geometries.get(marker_id)
                if old_traj:
                    self.vis.remove_geometry(old_traj, reset_bounding_box=False)

                # Add new trajectory
                self.vis.add_geometry(new_trajectory, reset_bounding_box=False)
                self._current_trajectory_geometries[marker_id] = new_trajectory

    def run(self) -> None:
        """
        Main visualization loop.
        Must be called from the main thread (Open3D requirement).
        """
        self._running = True
        render_rate = RateController(60.0)  # 60 FPS render target

        print("Viewer started. Controls: Q=quit, P=pause, R=reset, C=clear, T=toggle trails")

        while self._running:
            # Process message bus callbacks
            self.bus.spin_once()

            # Update marker positions (unless paused)
            if not self._paused:
                self._process_marker_updates()

            # Update Open3D - update each marker geometry individually
            for marker_id in self._added_marker_ids:
                geom = self.scene.marker_geometries.get(marker_id)
                if geom:
                    self.vis.update_geometry(geom)

            # Update axes geometries
            for marker_id in self._added_axes_ids:
                axes = self.scene.marker_axes.get(marker_id)
                if axes:
                    self.vis.update_geometry(axes)

            self.vis.poll_events()
            self.vis.update_renderer()

            # Update FPS
            self._fps_counter.tick()

            # Rate limit
            render_rate.sleep()

        # Cleanup
        self.vis.destroy_window()

    def get_render_fps(self) -> float:
        """Return current render FPS."""
        return self._fps_counter.get_fps()

    def get_tracker_fps(self) -> float:
        """Return tracker FPS from status messages."""
        return self._tracker_fps

    def stop(self) -> None:
        """Stop the viewer."""
        self._running = False
