"""
Integrated Open3D viewer with embedded camera feeds.

Uses Open3D's GUI module to show both 3D visualization and camera feeds
in a single window, avoiding OpenCV/Open3D conflicts.
"""

import numpy as np
import threading
import time
import sys
import cv2
from typing import List, Dict, Optional, Callable
from collections import defaultdict

try:
    import open3d as o3d
    import open3d.visualization.gui as gui
    import open3d.visualization.rendering as rendering
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

from ..core.message_bus import MessageBus
from ..core.data_types import CameraPose, FusedMarkerPose
from .scene_manager import SceneManager
from .geometry_factory import GeometryFactory


# Camera colors
CAMERA_COLORS = [
    [0.0, 1.0, 0.0],    # Green
    [1.0, 0.5, 0.0],    # Orange
    [1.0, 0.0, 1.0],    # Magenta
    [0.0, 1.0, 1.0],    # Cyan
    [1.0, 1.0, 0.0],    # Yellow
    [0.0, 0.0, 1.0],    # Blue
]


class IntegratedViewer:
    """
    Open3D GUI viewer with integrated camera preview panels.

    Shows 3D marker visualization with camera feed thumbnails
    in a side panel.
    """

    MENU_QUIT = 1
    MENU_RESET_VIEW = 2
    MENU_TOGGLE_CAMERAS = 3

    def __init__(
        self,
        message_bus: MessageBus,
        tracker=None,
        window_title: str = "Multi-Camera Tracking",
        window_width: int = 1600,
        window_height: int = 900
    ):
        if not HAS_OPEN3D:
            raise ImportError("Open3D required")

        self.bus = message_bus
        self.tracker = tracker
        self.window_title = window_title
        self.window_width = window_width
        self.window_height = window_height

        # State
        self._running = False
        self._camera_poses: List[CameraPose] = []
        self._camera_colors: Dict[str, List[float]] = {}

        # GUI components
        self.window = None
        self.scene_widget = None
        self.camera_panel = None
        self.image_widgets: Dict[str, gui.ImageWidget] = {}

        # Scene
        self.scene = SceneManager(show_trajectory=True)
        self._marker_materials: Dict[int, rendering.MaterialRecord] = {}

        # Pending updates
        self._pending_fused: Dict[int, FusedMarkerPose] = {}
        self._pending_lock = threading.Lock()

        # Frame update
        self._frame_images: Dict[str, o3d.geometry.Image] = {}

    def setup(self, camera_poses: List[CameraPose]) -> None:
        """Set up the viewer with camera poses."""
        self._camera_poses = camera_poses

        # Assign colors
        for i, pose in enumerate(camera_poses):
            self._camera_colors[pose.camera_id] = CAMERA_COLORS[i % len(CAMERA_COLORS)]

        # Subscribe to fused poses
        self.bus.subscribe('/markers/fused_poses', self._on_fused_pose)

    def _on_fused_pose(self, pose: FusedMarkerPose) -> None:
        """Handle incoming fused pose."""
        with self._pending_lock:
            self._pending_fused[pose.marker_id] = pose

    def _create_window(self):
        """Create the GUI window."""
        self.window = gui.Application.instance.create_window(
            self.window_title, self.window_width, self.window_height
        )

        # Create layout
        em = self.window.theme.font_size

        # 3D scene widget
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene_widget.scene.set_background([0.1, 0.1, 0.15, 1.0])

        # Camera panel (scrollable vertical layout for camera feeds)
        self.camera_panel = gui.ScrollableVert(0, gui.Margins(em * 0.5, em * 0.5, em * 0.5, em * 0.5))

        # Title
        title = gui.Label("Camera Feeds")
        title.text_color = gui.Color(1.0, 1.0, 1.0)
        self.camera_panel.add_child(title)

        # Add camera entries with status labels
        self._status_labels: Dict[str, gui.Label] = {}
        self._detection_labels: Dict[str, gui.Label] = {}

        for pose in self._camera_poses:
            short_id = pose.camera_id.split('-')[-1] if '-' in pose.camera_id else pose.camera_id[-8:]
            color = self._camera_colors.get(pose.camera_id, [1, 1, 1])

            # Camera label with colored indicator
            label = gui.Label(f"● Camera: {short_id}")
            label.text_color = gui.Color(*color)
            self.camera_panel.add_child(label)

            # Status label
            status_label = gui.Label("  Status: Waiting...")
            status_label.text_color = gui.Color(0.7, 0.7, 0.7)
            self._status_labels[pose.camera_id] = status_label
            self.camera_panel.add_child(status_label)

            # Detection label
            det_label = gui.Label("  Detections: --")
            det_label.text_color = gui.Color(0.5, 0.5, 0.5)
            self._detection_labels[pose.camera_id] = det_label
            self.camera_panel.add_child(det_label)

            # Placeholder image (RGBA format for compatibility)
            placeholder = np.zeros((90, 160, 4), dtype=np.uint8)
            placeholder[:, :, :3] = [40, 40, 40]  # BGR
            placeholder[:, :, 3] = 255  # Alpha
            # Draw camera name on placeholder
            cv2.putText(placeholder, short_id, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
            rgba_placeholder = cv2.cvtColor(placeholder[:, :, :3], cv2.COLOR_BGR2RGB)
            rgba_placeholder = np.dstack([rgba_placeholder, placeholder[:, :, 3]])
            img = o3d.geometry.Image(np.ascontiguousarray(rgba_placeholder))
            img_widget = gui.ImageWidget(img)
            self.image_widgets[pose.camera_id] = img_widget
            self.camera_panel.add_child(img_widget)

            # Spacer
            self.camera_panel.add_fixed(em * 0.5)

        # Use layout callback to position widgets
        self._panel_width = 260  # Fixed width for camera panel
        self.window.set_on_layout(self._on_layout)
        self.window.add_child(self.scene_widget)
        self.window.add_child(self.camera_panel)

        # Set up 3D scene
        self._setup_3d_scene()

        # Menu
        self._setup_menu()

        # Set close callback
        self.window.set_on_close(self._on_close)

    def _on_layout(self, layout_context):
        """Layout callback to position 3D scene and camera panel."""
        r = self.window.content_rect

        # Camera panel on the right with fixed width
        panel_width = self._panel_width
        panel_height = r.height

        # 3D scene takes remaining space on the left
        scene_width = r.width - panel_width
        scene_height = r.height

        # Position scene widget (left side)
        self.scene_widget.frame = gui.Rect(r.x, r.y, scene_width, scene_height)

        # Position camera panel (right side)
        self.camera_panel.frame = gui.Rect(r.x + scene_width, r.y, panel_width, panel_height)

    def _setup_3d_scene(self):
        """Set up the 3D scene with cameras."""
        scene = self.scene_widget.scene

        # Add coordinate frame at origin
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        scene.add_geometry("axes", axes, mat)

        # Add cameras
        for pose in self._camera_poses:
            self._add_camera_to_scene(pose)

        # Set up camera view - look at scene from above/side
        bounds = scene.bounding_box
        center = bounds.get_center()
        self.scene_widget.setup_camera(60.0, bounds, center)

        # Position viewer to see cameras and floor area
        self.scene_widget.look_at(
            center,  # Look at center
            [center[0] + 3, center[1] + 5, center[2] + 3],  # Eye position
            [0, 1, 0]  # Up vector
        )

    def _create_grid(self, size=5.0, divisions=20):
        """Create a ground plane grid."""
        lines = []
        points = []
        step = size / divisions
        half = size / 2

        idx = 0
        for i in range(divisions + 1):
            x = -half + i * step
            # Line along Z
            points.append([x, 0, -half])
            points.append([x, 0, half])
            lines.append([idx, idx + 1])
            idx += 2

            # Line along X
            points.append([-half, 0, x])
            points.append([half, 0, x])
            lines.append([idx, idx + 1])
            idx += 2

        grid = o3d.geometry.LineSet()
        grid.points = o3d.utility.Vector3dVector(points)
        grid.lines = o3d.utility.Vector2iVector(lines)
        grid.colors = o3d.utility.Vector3dVector([[0.3, 0.3, 0.3]] * len(lines))
        return grid

    def _add_camera_to_scene(self, pose: CameraPose):
        """Add a camera frustum to the scene."""
        color = self._camera_colors.get(pose.camera_id, [1, 1, 1])

        # Create camera frustum
        frustum = GeometryFactory.create_camera_frustum(
            pose=pose,
            color=color,
            scale=0.15
        )

        mat = rendering.MaterialRecord()
        mat.shader = "unlitLine"
        mat.line_width = 2.0

        self.scene_widget.scene.add_geometry(f"cam_{pose.camera_id}", frustum, mat)

    def _setup_menu(self):
        """Set up the application menu."""
        if gui.Application.instance.menubar is None:
            menu = gui.Menu()

            file_menu = gui.Menu()
            file_menu.add_item("Quit", self.MENU_QUIT)
            menu.add_menu("File", file_menu)

            view_menu = gui.Menu()
            view_menu.add_item("Reset View", self.MENU_RESET_VIEW)
            view_menu.add_item("Toggle Camera Panel", self.MENU_TOGGLE_CAMERAS)
            menu.add_menu("View", view_menu)

            gui.Application.instance.menubar = menu

        self.window.set_on_menu_item_activated(self.MENU_QUIT, self._on_menu_quit)
        self.window.set_on_menu_item_activated(self.MENU_RESET_VIEW, self._on_menu_reset)

    def _on_menu_quit(self):
        self._running = False
        self.window.close()

    def _on_menu_reset(self):
        bounds = self.scene_widget.scene.bounding_box
        self.scene_widget.setup_camera(60.0, bounds, bounds.get_center())

    def _on_close(self):
        self._running = False
        return True

    def _update_camera_images(self):
        """Update camera feed images from tracker."""
        if not self.tracker:
            return

        try:
            frames = self.tracker.get_latest_frames()
            detections = self.tracker.get_latest_detections()
        except Exception:
            return

        for pose in self._camera_poses:
            cam_id = pose.camera_id

            # Find matching frame - try both directions for ID matching
            frame = None
            dets = []
            matched_fid = None
            for fid, f in frames.items():
                # Check if either ID contains the other (partial match)
                if cam_id in fid or fid in cam_id:
                    frame = f
                    matched_fid = fid
                    dets = detections.get(fid, [])
                    break

            # Update status label
            if cam_id in self._status_labels:
                if frame is not None:
                    self._status_labels[cam_id].text = "  Status: Connected ✓"
                    self._status_labels[cam_id].text_color = gui.Color(0.3, 1.0, 0.3)
                else:
                    self._status_labels[cam_id].text = "  Status: No signal"
                    self._status_labels[cam_id].text_color = gui.Color(1.0, 0.3, 0.3)

            # Update detection label
            if cam_id in self._detection_labels:
                if dets:
                    self._detection_labels[cam_id].text = f"  Detections: {len(dets)} markers"
                    self._detection_labels[cam_id].text_color = gui.Color(0.3, 1.0, 0.3)
                else:
                    self._detection_labels[cam_id].text = "  Detections: 0"
                    self._detection_labels[cam_id].text_color = gui.Color(0.5, 0.5, 0.5)

            if frame is not None and cam_id in self.image_widgets:
                # Resize frame for thumbnail
                h, w = frame.shape[:2]
                target_w, target_h = 160, 90
                scale = min(target_w / w, target_h / h)
                new_w, new_h = int(w * scale), int(h * scale)
                resized = cv2.resize(frame, (new_w, new_h))

                # Pad to exact size
                padded = np.zeros((target_h, target_w, 3), dtype=np.uint8)
                padded[:] = [30, 30, 30]
                y_off = (target_h - new_h) // 2
                x_off = (target_w - new_w) // 2
                padded[y_off:y_off+new_h, x_off:x_off+new_w] = resized

                # Draw detection indicator
                if dets:
                    color = self._camera_colors.get(cam_id, [0, 1, 0])
                    bgr_color = [int(c * 255) for c in color]
                    cv2.rectangle(padded, (0, 0), (target_w-1, target_h-1), bgr_color, 2)

                # Convert BGR to RGBA (some Open3D versions require RGBA)
                rgba = cv2.cvtColor(padded, cv2.COLOR_BGR2RGBA)

                # Update image widget - store for GUI thread update
                rgba_contiguous = np.ascontiguousarray(rgba).astype(np.uint8)
                img = o3d.geometry.Image(rgba_contiguous)

                # Store the pending image update
                if not hasattr(self, '_pending_images'):
                    self._pending_images = {}
                self._pending_images[cam_id] = img

        # Apply pending image updates directly (we're in the main thread)
        if hasattr(self, '_pending_images') and self._pending_images and self.window:
            for cam_id, img in list(self._pending_images.items()):
                if cam_id in self.image_widgets:
                    try:
                        self.image_widgets[cam_id].update_image(img)
                    except Exception:
                        pass
            self._pending_images.clear()

            # Force redraw
            try:
                self.window.post_redraw()
            except Exception:
                pass

    def _update_markers(self):
        """Update marker positions in 3D scene."""
        with self._pending_lock:
            poses = dict(self._pending_fused)

        if not poses:
            return

        scene = self.scene_widget.scene

        for marker_id, pose in poses.items():
            geom_name = f"marker_{marker_id}"

            # Create or update marker sphere - larger size for visibility
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
            sphere.translate([pose.x, pose.y, pose.z])
            sphere.paint_uniform_color([1.0, 0.2, 0.2])  # Bright red
            sphere.compute_vertex_normals()

            mat = rendering.MaterialRecord()
            mat.shader = "defaultLit"
            mat.base_color = [1.0, 0.2, 0.2, 1.0]

            if scene.has_geometry(geom_name):
                scene.remove_geometry(geom_name)
            scene.add_geometry(geom_name, sphere, mat)

            # Also add orientation axes at marker position
            axes_name = f"marker_axes_{marker_id}"
            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            axes.translate([pose.x, pose.y, pose.z])

            axes_mat = rendering.MaterialRecord()
            axes_mat.shader = "defaultUnlit"

            if scene.has_geometry(axes_name):
                scene.remove_geometry(axes_name)
            scene.add_geometry(axes_name, axes, axes_mat)

            # Draw detection lines to cameras
            for cam_id in pose.detecting_cameras:
                line_name = f"line_{marker_id}_{cam_id}"

                # Find camera position
                cam_pos = None
                for cp in self._camera_poses:
                    if cp.camera_id in cam_id or cam_id in cp.camera_id:
                        cam_pos = cp.extrinsic[:3, 3]
                        break

                if cam_pos is not None:
                    color = self._camera_colors.get(cam_id, [1, 1, 1])
                    for cid, c in self._camera_colors.items():
                        if cid in cam_id or cam_id in cid:
                            color = c
                            break

                    line = o3d.geometry.LineSet()
                    line.points = o3d.utility.Vector3dVector([
                        [pose.x, pose.y, pose.z],
                        cam_pos.tolist()
                    ])
                    line.lines = o3d.utility.Vector2iVector([[0, 1]])
                    line.colors = o3d.utility.Vector3dVector([color])

                    line_mat = rendering.MaterialRecord()
                    line_mat.shader = "unlitLine"
                    line_mat.line_width = 2.0

                    if scene.has_geometry(line_name):
                        scene.remove_geometry(line_name)
                    scene.add_geometry(line_name, line, line_mat)

    def _on_tick(self):
        """Called on each frame update."""
        if not self._running:
            return False

        try:
            self.bus.spin_once()
            self._update_camera_images()
            self._update_markers()
        except Exception:
            pass

        return True

    def run(self):
        """Run the viewer."""
        self._running = True

        app = gui.Application.instance
        app.initialize()

        self._create_window()

        # Manual event loop with rate-limited updates
        last_update = time.time()
        update_interval = 0.033  # ~30 FPS

        while self._running:
            if not app.run_one_tick():
                break

            # Process message bus to receive fused poses
            self.bus.spin_once()

            now = time.time()
            if now - last_update >= update_interval:
                try:
                    self._update_camera_images()
                    self._update_markers()
                except Exception:
                    pass
                last_update = now

            time.sleep(0.001)

        self._running = False
