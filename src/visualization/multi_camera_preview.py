"""
Multi-Camera Preview Window with Detection Highlighting.

Shows all camera streams in a grid with clear visual indicators
for which cameras are currently detecting markers.
"""

import cv2
import numpy as np
import threading
import time
from typing import Dict, List, Optional, Tuple
from collections import defaultdict


# Camera colors for consistent identification
CAMERA_COLORS = [
    (0, 255, 0),    # Green
    (255, 165, 0),  # Orange (BGR: 0, 165, 255)
    (255, 0, 255),  # Magenta
    (255, 255, 0),  # Cyan
    (0, 255, 255),  # Yellow
    (255, 0, 0),    # Blue
]


class MultiCameraPreview:
    """
    Multi-camera preview window showing all camera streams
    with detection highlighting.
    """

    def __init__(
        self,
        camera_ids: List[str],
        window_name: str = "Multi-Camera Detection Preview",
        cell_size: Tuple[int, int] = (480, 270),
        cols: int = 2
    ):
        """
        Initialize multi-camera preview.

        Args:
            camera_ids: List of camera IDs to display
            window_name: OpenCV window name
            cell_size: Size of each camera cell (width, height)
            cols: Number of columns in grid
        """
        self.camera_ids = camera_ids
        self.window_name = window_name
        self.cell_size = cell_size
        self.cols = cols
        self.rows = (len(camera_ids) + cols - 1) // cols

        # Assign colors to cameras
        self.camera_colors = {}
        for i, cam_id in enumerate(camera_ids):
            self.camera_colors[cam_id] = CAMERA_COLORS[i % len(CAMERA_COLORS)]

        # Detection state
        self._frames: Dict[str, np.ndarray] = {}
        self._detections: Dict[str, List[dict]] = {}
        self._fused_info: Dict[int, List[str]] = {}  # marker_id -> list of detecting camera_ids
        self._lock = threading.Lock()

        # Statistics
        self._fps_counters: Dict[str, float] = defaultdict(float)
        self._last_frame_times: Dict[str, float] = {}

        # Running state
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def get_camera_color(self, camera_id: str) -> Tuple[int, int, int]:
        """Get the color assigned to a camera."""
        for cid, color in self.camera_colors.items():
            if cid in camera_id or camera_id in cid:
                return color
        return (128, 128, 128)  # Gray for unknown cameras

    def get_short_id(self, camera_id: str) -> str:
        """Get shortened camera ID for display."""
        # Extract last part after last dash (e.g., "E52215" from full ID)
        if '-' in camera_id:
            return camera_id.split('-')[-1]
        return camera_id[-8:] if len(camera_id) > 8 else camera_id

    def update_frame(self, camera_id: str, frame: np.ndarray,
                     detections: List[dict] = None) -> None:
        """
        Update frame and detections for a camera.

        Args:
            camera_id: Camera identifier
            frame: BGR image from camera
            detections: List of detection dicts with 'corners', 'id', 'position'
        """
        with self._lock:
            self._frames[camera_id] = frame.copy()
            self._detections[camera_id] = detections or []

            # Update FPS
            now = time.time()
            if camera_id in self._last_frame_times:
                dt = now - self._last_frame_times[camera_id]
                if dt > 0:
                    self._fps_counters[camera_id] = 0.9 * self._fps_counters[camera_id] + 0.1 * (1.0 / dt)
            self._last_frame_times[camera_id] = now

    def update_fused_detections(self, marker_id: int, detecting_cameras: List[str]) -> None:
        """
        Update which cameras are detecting a specific marker.

        Args:
            marker_id: The marker ID
            detecting_cameras: List of camera IDs detecting this marker
        """
        with self._lock:
            self._fused_info[marker_id] = detecting_cameras

    def clear_fused_detections(self) -> None:
        """Clear all fused detection info."""
        with self._lock:
            self._fused_info.clear()

    def _match_camera_id(self, full_id: str, target_id: str) -> bool:
        """Check if camera IDs match (partial match)."""
        return full_id in target_id or target_id in full_id

    def _find_frame(self, target_cam_id: str) -> Optional[np.ndarray]:
        """Find frame for camera ID (with partial matching)."""
        for cam_id, frame in self._frames.items():
            if self._match_camera_id(cam_id, target_cam_id):
                return frame
        return None

    def _find_detections(self, target_cam_id: str) -> List[dict]:
        """Find detections for camera ID (with partial matching)."""
        for cam_id, dets in self._detections.items():
            if self._match_camera_id(cam_id, target_cam_id):
                return dets
        return []

    def _is_camera_detecting(self, cam_id: str) -> bool:
        """Check if camera is currently detecting any marker."""
        dets = self._find_detections(cam_id)
        return len(dets) > 0

    def _get_markers_by_camera(self, cam_id: str) -> List[int]:
        """Get list of marker IDs detected by this camera from fused info."""
        markers = []
        for marker_id, cameras in self._fused_info.items():
            for c in cameras:
                if self._match_camera_id(c, cam_id):
                    markers.append(marker_id)
                    break
        return markers

    def _render_camera_cell(self, cam_id: str) -> np.ndarray:
        """
        Render a single camera cell with detection highlighting.

        Args:
            cam_id: Camera ID to render

        Returns:
            Rendered cell image
        """
        frame = self._find_frame(cam_id)
        detections = self._find_detections(cam_id)
        is_detecting = len(detections) > 0
        color = self.get_camera_color(cam_id)
        short_id = self.get_short_id(cam_id)

        # Create cell
        cell = np.zeros((self.cell_size[1], self.cell_size[0], 3), dtype=np.uint8)

        if frame is not None:
            # Resize frame to cell size
            h, w = frame.shape[:2]
            scale = min(self.cell_size[0] / w, self.cell_size[1] / h)
            new_w, new_h = int(w * scale), int(h * scale)
            resized = cv2.resize(frame, (new_w, new_h))

            # Center in cell
            x_offset = (self.cell_size[0] - new_w) // 2
            y_offset = (self.cell_size[1] - new_h) // 2
            cell[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized

            # Draw marker detections
            for det in detections:
                if det.get('corners') is not None:
                    corners = det['corners'].copy()
                    # Scale corners
                    corners[:, 0] = corners[:, 0] * scale + x_offset
                    corners[:, 1] = corners[:, 1] * scale + y_offset
                    corners = corners.reshape((4, 2)).astype(int)

                    # Draw marker polygon
                    cv2.polylines(cell, [corners], True, color, 3)

                    # Draw marker ID
                    center = corners.mean(axis=0).astype(int)
                    marker_id = det.get('id', '?')
                    cv2.putText(cell, f"ID:{marker_id}",
                               (center[0] - 25, center[1] - 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                    # Draw position if available
                    pos = det.get('position')
                    if pos is not None:
                        pos_text = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                        cv2.putText(cell, pos_text,
                                   (center[0] - 60, center[1] + 25),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        else:
            # No signal placeholder
            cv2.putText(cell, "NO SIGNAL",
                       (self.cell_size[0] // 2 - 60, self.cell_size[1] // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Draw border - thick colored border when detecting
        border_thickness = 8 if is_detecting else 2
        border_color = color if is_detecting else (64, 64, 64)
        cv2.rectangle(cell, (0, 0),
                     (self.cell_size[0]-1, self.cell_size[1]-1),
                     border_color, border_thickness)

        # Camera ID label with colored background
        label_bg_color = color if is_detecting else (40, 40, 40)
        label_height = 30
        cv2.rectangle(cell, (0, 0), (self.cell_size[0], label_height), label_bg_color, -1)

        # Camera name and status
        status = "DETECTING" if is_detecting else "idle"
        fps = self._fps_counters.get(cam_id, 0)
        for cid in self._fps_counters:
            if self._match_camera_id(cid, cam_id):
                fps = self._fps_counters[cid]
                break

        label_text = f"{short_id} | {status} | {fps:.1f} FPS"
        text_color = (0, 0, 0) if is_detecting else (200, 200, 200)
        cv2.putText(cell, label_text, (10, 22),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

        # Detection count badge
        if is_detecting:
            count_text = f"{len(detections)}"
            badge_x = self.cell_size[0] - 40
            cv2.circle(cell, (badge_x, 15), 15, (255, 255, 255), -1)
            cv2.putText(cell, count_text, (badge_x - 8, 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        return cell

    def _render_status_bar(self) -> np.ndarray:
        """Render bottom status bar showing detection summary."""
        bar_height = 60
        bar_width = self.cols * self.cell_size[0]
        bar = np.zeros((bar_height, bar_width, 3), dtype=np.uint8)
        bar[:] = (30, 30, 30)

        # Title
        cv2.putText(bar, "Detection Status:", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # Show which markers are being detected and by which cameras
        x_offset = 180
        with self._lock:
            for marker_id, cameras in self._fused_info.items():
                if cameras:
                    # Marker ID
                    cv2.putText(bar, f"M{marker_id}:", (x_offset, 25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    x_offset += 45

                    # Camera indicators
                    for cam in cameras:
                        color = self.get_camera_color(cam)
                        short_id = self.get_short_id(cam)
                        cv2.circle(bar, (x_offset + 10, 20), 8, color, -1)
                        cv2.putText(bar, short_id[:6], (x_offset + 22, 25),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                        x_offset += 70

                    x_offset += 20

        # Legend at bottom
        y = 50
        x = 10
        cv2.putText(bar, "Cameras:", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        x += 70
        for cam_id in self.camera_ids:
            color = self.get_camera_color(cam_id)
            short_id = self.get_short_id(cam_id)
            cv2.circle(bar, (x, y - 5), 6, color, -1)
            cv2.putText(bar, short_id, (x + 12, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            x += 90

        return bar

    def render(self) -> np.ndarray:
        """
        Render the complete preview grid.

        Returns:
            Complete grid image with all cameras and status bar
        """
        with self._lock:
            # Render camera cells
            cells = []
            for cam_id in self.camera_ids:
                cell = self._render_camera_cell(cam_id)
                cells.append(cell)

            # Pad to fill grid
            empty_cell = np.zeros((self.cell_size[1], self.cell_size[0], 3), dtype=np.uint8)
            while len(cells) < self.rows * self.cols:
                cells.append(empty_cell.copy())

            # Arrange cells in grid
            rows = []
            for r in range(self.rows):
                row_cells = cells[r * self.cols:(r + 1) * self.cols]
                rows.append(np.hstack(row_cells))

            grid = np.vstack(rows)

            # Add status bar
            status_bar = self._render_status_bar()

            return np.vstack([grid, status_bar])

    def start(self) -> None:
        """Start the preview window in a background thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the preview window."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run_loop(self) -> None:
        """Main display loop."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name,
                        self.cols * self.cell_size[0],
                        self.rows * self.cell_size[1] + 60)

        while self._running:
            image = self.render()
            cv2.imshow(self.window_name, image)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                self._running = False
                break

        cv2.destroyWindow(self.window_name)

    def is_running(self) -> bool:
        """Check if preview is still running."""
        return self._running


def create_preview_from_tracker(tracker, camera_ids: List[str]) -> MultiCameraPreview:
    """
    Create a MultiCameraPreview connected to a MultiCameraTracker.

    Args:
        tracker: MultiCameraTracker instance
        camera_ids: List of camera IDs

    Returns:
        Configured MultiCameraPreview
    """
    preview = MultiCameraPreview(camera_ids)
    return preview
