"""
Network Stream ArUco Marker Tracker

Detects ArUco markers from video streams received over the network.
Works with StreamServer to process frames from multiple remote cameras.
"""

import time
import threading
from typing import Optional, Dict, List, Tuple, Callable
import numpy as np

try:
    import cv2
    from cv2 import aruco
except ImportError:
    raise ImportError("OpenCV with ArUco support required. Install with: pip install opencv-contrib-python")

from ..core.data_types import MarkerPose
from ..core.message_bus import MessageBus
from ..core.timing import RateController, FPSCounter
from ..streaming.stream_server import StreamServer


# ArUco dictionary mapping
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
}


class NetworkStreamTracker:
    """
    ArUco marker tracker for network video streams.

    Receives frames from a StreamServer and detects ArUco markers.
    Supports multiple cameras with independent tracking.
    """

    def __init__(self,
                 message_bus: MessageBus,
                 stream_server: StreamServer,
                 marker_size_m: float = 0.05,
                 dictionary_type: str = "DICT_4X4_50",
                 rate_hz: float = 30.0,
                 camera_matrices: Optional[Dict[str, np.ndarray]] = None,
                 dist_coeffs: Optional[Dict[str, np.ndarray]] = None,
                 extrinsic_matrices: Optional[Dict[str, np.ndarray]] = None):
        """
        Initialize network stream tracker.

        Args:
            message_bus: Message bus for publishing marker poses
            stream_server: StreamServer instance to receive frames from
            marker_size_m: Physical size of markers in meters
            dictionary_type: ArUco dictionary to use
            rate_hz: Target tracking rate in Hz
            camera_matrices: Dict mapping camera_id to 3x3 intrinsic matrix
            dist_coeffs: Dict mapping camera_id to distortion coefficients
            extrinsic_matrices: Dict mapping camera_id to 4x4 extrinsic matrix
        """
        self.message_bus = message_bus
        self.stream_server = stream_server
        self.marker_size_m = marker_size_m
        self.rate_hz = rate_hz

        # Camera parameters per camera
        self.camera_matrices = camera_matrices or {}
        self.dist_coeffs = dist_coeffs or {}
        self.extrinsic_matrices = extrinsic_matrices or {}

        # ArUco detector setup
        if dictionary_type not in ARUCO_DICT:
            raise ValueError(f"Unknown dictionary: {dictionary_type}")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dictionary_type])

        # API differs between OpenCV versions
        if hasattr(cv2.aruco, 'ArucoDetector'):
            # OpenCV 4.7+
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self._use_new_api = True
        else:
            # OpenCV 4.5.x and earlier
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_detector = None
            self._use_new_api = False

        # Internal state
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._fps_counter = FPSCounter()

        # Latest detections per camera
        self._latest_detections: Dict[str, List[Dict]] = {}
        self._detections_lock = threading.Lock()

        # Frame callback
        self.frame_callback: Optional[Callable] = None

    def set_camera_params(self, camera_id: str,
                          camera_matrix: np.ndarray,
                          dist_coeffs: np.ndarray = None,
                          extrinsic_matrix: np.ndarray = None):
        """
        Set camera parameters for a specific camera.

        Args:
            camera_id: Camera identifier
            camera_matrix: 3x3 intrinsic matrix
            dist_coeffs: Distortion coefficients (optional)
            extrinsic_matrix: 4x4 camera to world transform (optional)
        """
        self.camera_matrices[camera_id] = camera_matrix
        if dist_coeffs is not None:
            self.dist_coeffs[camera_id] = dist_coeffs
        if extrinsic_matrix is not None:
            self.extrinsic_matrices[camera_id] = extrinsic_matrix

    def start(self):
        """Start the tracking loop in background thread."""
        if self._running:
            return

        # Start stream server if not already running
        if not self.stream_server.running:
            self.stream_server.start()

        self._running = True
        self._thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self._thread.start()
        print("Network stream tracker started")

    def stop(self):
        """Stop the tracking loop."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _tracking_loop(self):
        """Main tracking loop."""
        rate = RateController(self.rate_hz)

        while self._running:
            # Get frames from all connected cameras
            frames = self.stream_server.get_all_frames()

            for camera_id, frame in frames.items():
                # Detect markers in this frame
                detections = self._detect_markers(frame, camera_id)

                # Store latest detections
                with self._detections_lock:
                    self._latest_detections[camera_id] = detections

                # Publish detected markers
                for detection in detections:
                    marker_pose = MarkerPose(
                        marker_id=detection['id'],
                        x=detection['position'][0],
                        y=detection['position'][1],
                        z=detection['position'][2],
                        timestamp=time.time(),
                        orientation=detection.get('orientation'),
                        confidence=detection.get('confidence', 1.0)
                    )
                    self.message_bus.publish('/markers/poses', marker_pose)

                # Call frame callback if registered
                if self.frame_callback:
                    self.frame_callback(camera_id, frame, detections)

            self._fps_counter.tick()
            rate.sleep()

    def _detect_markers(self, frame: np.ndarray, camera_id: str) -> List[Dict]:
        """
        Detect ArUco markers in frame.

        Args:
            frame: BGR color image
            camera_id: ID of the camera that captured this frame

        Returns:
            List of detection dictionaries
        """
        detections = []

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if self._use_new_api:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if ids is None:
            return detections

        # Get camera parameters for this camera
        camera_matrix = self.camera_matrices.get(camera_id)
        dist_coeffs = self.dist_coeffs.get(camera_id, np.zeros(5))
        extrinsic = self.extrinsic_matrices.get(camera_id, np.eye(4))

        # If no camera matrix, create default based on frame size
        if camera_matrix is None:
            h, w = frame.shape[:2]
            camera_matrix = np.array([
                [w, 0, w / 2],
                [0, w, h / 2],
                [0, 0, 1]
            ], dtype=np.float64)

        # Process each detected marker
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]

            # Pose estimation using solvePnP
            obj_points = np.array([
                [-self.marker_size_m / 2, self.marker_size_m / 2, 0],
                [self.marker_size_m / 2, self.marker_size_m / 2, 0],
                [self.marker_size_m / 2, -self.marker_size_m / 2, 0],
                [-self.marker_size_m / 2, -self.marker_size_m / 2, 0]
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners,
                camera_matrix, dist_coeffs
            )

            if success:
                # Position in camera frame
                position_camera = tvec.flatten()

                # Transform to world frame
                pos_homogeneous = np.append(position_camera, 1)
                pos_world = extrinsic @ pos_homogeneous

                # Get orientation quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

                detections.append({
                    'id': int(marker_id),
                    'camera_id': camera_id,
                    'position': (pos_world[0], pos_world[1], pos_world[2]),
                    'orientation': quaternion,
                    'corners': marker_corners,
                    'rvec': rvec,
                    'tvec': tvec,
                    'confidence': 1.0
                })
            else:
                # Fallback: use center of marker
                center = marker_corners.mean(axis=0)
                h, w = frame.shape[:2]

                detections.append({
                    'id': int(marker_id),
                    'camera_id': camera_id,
                    'position': (center[0] / w, center[1] / h, 1.0),
                    'orientation': (0, 0, 0, 1),
                    'corners': marker_corners,
                    'confidence': 0.5
                })

        return detections

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
        """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return (x, y, z, w)

    def get_latest_detections(self, camera_id: str = None) -> Dict[str, List[Dict]]:
        """
        Get latest detections.

        Args:
            camera_id: Specific camera ID, or None for all cameras

        Returns:
            Dict mapping camera_id to list of detections
        """
        with self._detections_lock:
            if camera_id:
                return {camera_id: self._latest_detections.get(camera_id, [])}
            return dict(self._latest_detections)

    def get_fps(self) -> float:
        """Return current tracking FPS."""
        return self._fps_counter.get_fps()

    def set_frame_callback(self, callback: Callable):
        """
        Set callback for each processed frame.

        Args:
            callback: Function(camera_id, frame, detections)
        """
        self.frame_callback = callback

    def draw_detections(self, frame: np.ndarray,
                        detections: List[Dict],
                        camera_id: str = None) -> np.ndarray:
        """
        Draw detected markers on frame.

        Args:
            frame: BGR image
            detections: List of detection dictionaries
            camera_id: Camera ID for pose visualization

        Returns:
            Frame with detections drawn
        """
        result = frame.copy()

        camera_matrix = self.camera_matrices.get(camera_id) if camera_id else None
        dist_coeffs = self.dist_coeffs.get(camera_id, np.zeros(5)) if camera_id else np.zeros(5)

        for detection in detections:
            corners = detection['corners'].reshape((4, 2)).astype(int)

            # Draw marker boundary
            cv2.polylines(result, [corners], True, (0, 255, 0), 2)

            # Draw marker ID
            center = corners.mean(axis=0).astype(int)
            cv2.putText(result, f"ID: {detection['id']}",
                       (center[0] - 30, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Draw position
            pos = detection['position']
            cv2.putText(result, f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                       (center[0] - 60, center[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            # Draw coordinate axes if pose available
            if camera_matrix is not None and 'rvec' in detection and 'tvec' in detection:
                cv2.drawFrameAxes(result, camera_matrix, dist_coeffs,
                                 detection['rvec'], detection['tvec'],
                                 self.marker_size_m * 0.5)

        return result

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        return False
