"""
Multi-Camera ArUco Marker Tracker.

Processes frames from multiple cameras simultaneously,
detects markers, and fuses poses from multiple viewpoints.
"""

import threading
import time
from typing import Dict, List, Optional, Tuple
from collections import defaultdict
from dataclasses import dataclass

import cv2
import numpy as np

from ..core.message_bus import MessageBus
from ..core.data_types import CameraPose, CameraDetection, FusedMarkerPose
from ..streaming.stream_server import StreamServer
from .pose_fusion import PoseFusion


# ArUco dictionary types
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
}


@dataclass
class CameraConfig:
    """Configuration for a single camera."""
    camera_id: str
    extrinsic: np.ndarray  # 4x4 camera-to-world transform
    intrinsic: Optional[np.ndarray] = None  # 3x3 camera matrix
    dist_coeffs: Optional[np.ndarray] = None  # Distortion coefficients


class MultiCameraTracker:
    """
    Tracks ArUco markers across multiple cameras with pose fusion.

    Features:
    - Processes frames from all connected cameras
    - Detects markers independently per camera
    - Fuses multi-view observations for better accuracy
    - Publishes both raw detections and fused poses
    """

    def __init__(
        self,
        message_bus: MessageBus,
        server: StreamServer,
        camera_configs: Dict[str, CameraConfig],
        marker_size_m: float = 0.05,
        dictionary_type: str = "DICT_4X4_50",
        fusion_method: str = "weighted_average"
    ):
        """
        Initialize multi-camera tracker.

        Args:
            message_bus: Message bus for publishing poses
            server: Stream server receiving camera frames
            camera_configs: Dict mapping camera_id to CameraConfig
            marker_size_m: Physical marker size in meters
            dictionary_type: ArUco dictionary type
            fusion_method: Pose fusion method
        """
        self.message_bus = message_bus
        self.server = server
        self.camera_configs = camera_configs
        self.marker_size_m = marker_size_m

        # ArUco detector
        if dictionary_type not in ARUCO_DICT:
            raise ValueError(f"Unknown dictionary: {dictionary_type}")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dictionary_type])

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self._use_new_api = True
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_detector = None
            self._use_new_api = False

        # Pose fusion
        self.fusion = PoseFusion(fusion_method=fusion_method)

        # Publishers
        self.fused_publisher = message_bus.create_publisher('/markers/fused_poses', FusedMarkerPose)
        self.raw_publisher = message_bus.create_publisher('/markers/raw_detections', CameraDetection)

        # Threading
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Per-camera intrinsics cache (computed from first frame if not provided)
        self._camera_matrices: Dict[str, np.ndarray] = {}

        # Statistics
        self.stats = {
            'frames_processed': defaultdict(int),
            'detections': defaultdict(int),
            'fused_poses': 0,
            'start_time': None
        }

        # Latest frames for preview (thread-safe)
        self._latest_frames: Dict[str, np.ndarray] = {}
        self._latest_detections: Dict[str, List[dict]] = {}
        self._frame_lock = threading.Lock()

    def start(self) -> None:
        """Start processing in background thread."""
        if self._running:
            return

        self._running = True
        self.stats['start_time'] = time.time()
        self._thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._thread.start()
        print(f"Multi-camera tracker started with {len(self.camera_configs)} cameras")

    def stop(self) -> None:
        """Stop processing."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _get_camera_matrix(self, camera_id: str, frame_shape: Tuple[int, int]) -> np.ndarray:
        """Get or create camera intrinsic matrix."""
        if camera_id in self._camera_matrices:
            return self._camera_matrices[camera_id]

        # Check if provided in config
        config = self.camera_configs.get(camera_id)
        if config and config.intrinsic is not None:
            self._camera_matrices[camera_id] = config.intrinsic
            return config.intrinsic

        # Create default based on frame size
        h, w = frame_shape[:2]
        focal_length = w  # Approximate
        camera_matrix = np.array([
            [focal_length, 0, w / 2],
            [0, focal_length, h / 2],
            [0, 0, 1]
        ], dtype=np.float64)

        self._camera_matrices[camera_id] = camera_matrix
        return camera_matrix

    def _get_dist_coeffs(self, camera_id: str) -> np.ndarray:
        """Get distortion coefficients for camera."""
        config = self.camera_configs.get(camera_id)
        if config and config.dist_coeffs is not None:
            return config.dist_coeffs
        return np.zeros(5)

    def _get_extrinsic(self, camera_id: str) -> np.ndarray:
        """Get extrinsic matrix for camera."""
        config = self.camera_configs.get(camera_id)
        if config:
            return config.extrinsic
        return np.eye(4)

    def _camera_to_world(self, position_camera: np.ndarray, extrinsic: np.ndarray) -> np.ndarray:
        """Transform position from camera frame to world frame."""
        pos_homogeneous = np.append(position_camera, 1)
        pos_world = extrinsic @ pos_homogeneous
        return pos_world[:3]

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> Tuple[float, float, float, float]:
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

    def _detect_markers(self, frame: np.ndarray, camera_id: str) -> List[CameraDetection]:
        """
        Detect ArUco markers in a frame and compute 3D poses.

        Args:
            frame: BGR image
            camera_id: Camera identifier

        Returns:
            List of CameraDetection objects
        """
        detections = []

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

        # Get camera parameters
        camera_matrix = self._get_camera_matrix(camera_id, frame.shape)
        dist_coeffs = self._get_dist_coeffs(camera_id)
        extrinsic = self._get_extrinsic(camera_id)

        # Process each detected marker
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]

            # 3D object points for marker corners
            obj_points = np.array([
                [-self.marker_size_m / 2, self.marker_size_m / 2, 0],
                [self.marker_size_m / 2, self.marker_size_m / 2, 0],
                [self.marker_size_m / 2, -self.marker_size_m / 2, 0],
                [-self.marker_size_m / 2, -self.marker_size_m / 2, 0]
            ], dtype=np.float32)

            # Solve PnP for pose estimation
            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners,
                camera_matrix, dist_coeffs
            )

            if success:
                position_camera = tvec.flatten()
                position_world = self._camera_to_world(position_camera, extrinsic)

                # Get orientation
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

                detection = CameraDetection(
                    camera_id=camera_id,
                    marker_id=int(marker_id),
                    position_world=position_world,
                    position_camera=position_camera,
                    timestamp=time.time(),
                    orientation=quaternion,
                    confidence=1.0,
                    corners=marker_corners,
                    rvec=rvec,
                    tvec=tvec
                )
                detections.append(detection)

        return detections

    def _processing_loop(self) -> None:
        """Main processing loop - processes all cameras."""
        last_stats_time = time.time()

        while self._running:
            current_time = time.time()
            all_detections: List[CameraDetection] = []

            # Get all connected cameras
            camera_ids = self.server.get_camera_ids()

            # Process each camera
            for camera_id in camera_ids:
                # Match camera to config (partial match on ID)
                config_id = None
                for cfg_id in self.camera_configs.keys():
                    if cfg_id in camera_id or camera_id in cfg_id:
                        config_id = cfg_id
                        break

                # Skip if no matching config
                if config_id is None:
                    # Use camera_id directly if no config match
                    config_id = camera_id

                frame = self.server.get_frame(camera_id)
                if frame is None:
                    continue

                self.stats['frames_processed'][camera_id] += 1

                # Detect markers
                detections = self._detect_markers(frame, config_id)
                all_detections.extend(detections)

                # Update stats
                for det in detections:
                    self.stats['detections'][camera_id] += 1

                    # Publish raw detection
                    self.raw_publisher.publish(det)

                # Store frame for preview
                with self._frame_lock:
                    self._latest_frames[camera_id] = frame.copy()
                    self._latest_detections[camera_id] = [
                        {'corners': d.corners, 'id': d.marker_id, 'position': d.position_world}
                        for d in detections
                    ]

            # Add all detections to fusion buffer
            for det in all_detections:
                self.fusion.add_detection(det)

            # Get fused poses and publish
            fused_poses = self.fusion.get_fused_poses(current_time)
            for pose in fused_poses:
                self.fused_publisher.publish(pose)
                self.stats['fused_poses'] += 1

                # Log multi-camera detections
                if len(pose.detecting_cameras) > 1:
                    cams = ", ".join([c[-12:] for c in pose.detecting_cameras])
                    print(f"Marker {pose.marker_id} fused from {len(pose.detecting_cameras)} cameras: [{cams}] "
                          f"-> ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})")

            # Print stats periodically
            if current_time - last_stats_time > 5.0:
                self._print_stats()
                last_stats_time = current_time

            # Small sleep to prevent CPU hogging
            if not all_detections:
                time.sleep(0.01)

    def _print_stats(self) -> None:
        """Print processing statistics."""
        elapsed = time.time() - self.stats['start_time']
        if elapsed < 1.0:
            return

        print(f"\n=== Multi-Camera Tracker Stats ===")
        for cam_id, count in self.stats['frames_processed'].items():
            fps = count / elapsed
            detections = self.stats['detections'].get(cam_id, 0)
            print(f"  {cam_id[-12:]}: {fps:.1f} FPS, {detections} detections")
        print(f"  Fused poses: {self.stats['fused_poses']}")
        print("===================================\n")

    def get_latest_frames(self) -> Dict[str, np.ndarray]:
        """Get latest frames from all cameras (thread-safe)."""
        with self._frame_lock:
            return {k: v.copy() for k, v in self._latest_frames.items()}

    def get_latest_detections(self) -> Dict[str, List[dict]]:
        """Get latest detections from all cameras (thread-safe)."""
        with self._frame_lock:
            return {k: list(v) for k, v in self._latest_detections.items()}

    def draw_detections_on_frame(self, frame: np.ndarray, detections: List[dict],
                                  camera_id: str) -> np.ndarray:
        """Draw detection overlays on a frame."""
        result = frame.copy()

        for det in detections:
            if det['corners'] is not None:
                corners = det['corners'].reshape((4, 2)).astype(int)
                cv2.polylines(result, [corners], True, (0, 255, 0), 2)

                center = corners.mean(axis=0).astype(int)
                cv2.putText(result, f"ID: {det['id']}",
                           (center[0] - 30, center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if det['position'] is not None:
                    pos = det['position']
                    cv2.putText(result, f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                               (center[0] - 70, center[1] + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # Camera label
        cv2.putText(result, camera_id[-12:], (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Detection count
        cv2.putText(result, f"Markers: {len(detections)}", (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return result
