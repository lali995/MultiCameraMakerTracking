#!/usr/bin/env python3
"""
Run ArUco marker tracking on video with 3D visualization.

This script reads frames from a video file, detects ArUco markers,
and visualizes them in a 3D scene with camera positions.

Usage:
    # Use first camera's extrinsic
    python scripts/run_video_with_visualization.py test_video/test_video.mp4

    # Use specific camera config
    python scripts/run_video_with_visualization.py test_video/test_video.mp4 \
        --camera-config config/SZVIDS-250515-DB77B5-8F194B-9AF224
"""

import os
import sys
import time
import argparse
import json
import threading
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from cv2 import aruco

from src.core.message_bus import MessageBus
from src.core.data_types import MarkerPose, CameraPose
from src.core.config_loader import ConfigLoader
from src.visualization.viewer import MarkerViewer

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


class VideoTrackerWithVisualization:
    """Video-based ArUco tracker with 3D visualization."""

    def __init__(self,
                 video_path: str,
                 message_bus: MessageBus,
                 marker_size_m: float = 0.05,
                 dictionary_type: str = "DICT_4X4_50",
                 camera_matrix: np.ndarray = None,
                 dist_coeffs: np.ndarray = None,
                 extrinsic_matrix: np.ndarray = None,
                 playback_speed: float = 1.0,
                 loop: bool = True,
                 show_preview: bool = True):
        """
        Initialize video tracker.

        Args:
            video_path: Path to video file
            message_bus: Message bus for publishing poses
            marker_size_m: Physical marker size in meters
            dictionary_type: ArUco dictionary type
            camera_matrix: Camera intrinsic matrix
            dist_coeffs: Distortion coefficients
            extrinsic_matrix: Camera to world transform (4x4)
            playback_speed: Video playback speed multiplier
            loop: Whether to loop video
            show_preview: Show video preview window
        """
        self.video_path = video_path
        self.message_bus = message_bus
        self.marker_size_m = marker_size_m
        self.playback_speed = playback_speed
        self.loop = loop
        self.show_preview = show_preview

        # Camera parameters
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        self.extrinsic_matrix = extrinsic_matrix if extrinsic_matrix is not None else np.eye(4)

        # Create publisher for marker poses
        self.pose_publisher = message_bus.create_publisher('/markers/poses', MarkerPose)

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

        # Video capture
        self.cap = None
        self.fps = 30.0
        self.frame_count = 0
        self.total_frames = 0

        # Threading
        self._running = False
        self._thread = None

    def _camera_to_world(self, position_camera: np.ndarray) -> tuple:
        """Transform position from camera frame to world frame."""
        pos_homogeneous = np.append(position_camera, 1)
        pos_world = self.extrinsic_matrix @ pos_homogeneous
        return (pos_world[0], pos_world[1], pos_world[2])

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> tuple:
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

    def _transform_orientation(self, quaternion: tuple) -> tuple:
        """Transform orientation quaternion from camera to world frame."""
        R_world = self.extrinsic_matrix[:3, :3]
        x, y, z, w = quaternion
        R_camera = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
        R_combined = R_world @ R_camera
        return self._rotation_matrix_to_quaternion(R_combined)

    def detect_and_publish(self, frame: np.ndarray) -> list:
        """Detect markers and publish poses."""
        detections = []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._use_new_api:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if ids is None:
            return detections

        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]

            if self.camera_matrix is not None:
                obj_points = np.array([
                    [-self.marker_size_m / 2, self.marker_size_m / 2, 0],
                    [self.marker_size_m / 2, self.marker_size_m / 2, 0],
                    [self.marker_size_m / 2, -self.marker_size_m / 2, 0],
                    [-self.marker_size_m / 2, -self.marker_size_m / 2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, marker_corners,
                    self.camera_matrix, self.dist_coeffs
                )

                if success:
                    position_camera = tvec.flatten()
                    position_world = self._camera_to_world(position_camera)

                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)
                    quaternion_world = self._transform_orientation(quaternion)

                    # Publish marker pose
                    marker_pose = MarkerPose(
                        marker_id=int(marker_id),
                        x=position_world[0],
                        y=position_world[1],
                        z=position_world[2],
                        timestamp=time.time(),
                        orientation=quaternion_world,
                        confidence=1.0
                    )
                    self.pose_publisher.publish(marker_pose)

                    detections.append({
                        'id': int(marker_id),
                        'position': position_world,
                        'corners': marker_corners,
                        'rvec': rvec,
                        'tvec': tvec
                    })

        return detections

    def start(self):
        """Start video processing in background thread."""
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video: {self.video_path}")

        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Set default camera matrix if not provided
        if self.camera_matrix is None:
            focal_length = width
            self.camera_matrix = np.array([
                [focal_length, 0, width / 2],
                [0, focal_length, height / 2],
                [0, 0, 1]
            ], dtype=np.float64)

        print(f"Video: {self.video_path}")
        print(f"Resolution: {width}x{height} @ {self.fps:.2f} FPS")
        print(f"Total frames: {self.total_frames}")

        self._running = True
        self._thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop video processing."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()

    def draw_detections(self, frame: np.ndarray, detections: list) -> np.ndarray:
        """Draw detected markers on frame."""
        result = frame.copy()

        for detection in detections:
            corners = detection['corners'].reshape((4, 2)).astype(int)

            # Draw marker boundary
            cv2.polylines(result, [corners], True, (0, 255, 0), 2)

            # Draw marker ID
            center = corners.mean(axis=0).astype(int)
            cv2.putText(result, f"ID: {detection['id']}",
                       (center[0] - 30, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Draw world position
            if 'position' in detection:
                pos = detection['position']
                cv2.putText(result, f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                           (center[0] - 80, center[1] + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Draw axes if available
            if 'rvec' in detection and 'tvec' in detection:
                cv2.drawFrameAxes(result, self.camera_matrix, self.dist_coeffs,
                                 detection['rvec'], detection['tvec'],
                                 self.marker_size_m * 0.5)

        # Add frame info
        cv2.putText(result, f"Frame: {self.frame_count}/{self.total_frames}",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result, f"Markers: {len(detections)}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return result

    def _processing_loop(self):
        """Main video processing loop."""
        frame_delay = 1.0 / (self.fps * self.playback_speed)

        # Create preview window if enabled
        if self.show_preview:
            cv2.namedWindow("Video Preview", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Video Preview", 960, 540)

        while self._running:
            ret, frame = self.cap.read()

            if not ret:
                if self.loop:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self.frame_count = 0
                    continue
                else:
                    break

            self.frame_count += 1

            # Detect and publish markers
            detections = self.detect_and_publish(frame)

            # Show preview with detections
            if self.show_preview:
                display = self.draw_detections(frame, detections)
                cv2.imshow("Video Preview", display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self._running = False
                    break

            if detections:
                for det in detections:
                    pos = det['position']
                    print(f"Frame {self.frame_count}: Marker {det['id']} at "
                          f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

            # Maintain playback speed
            time.sleep(frame_delay)

        if self.show_preview:
            cv2.destroyAllWindows()

        print("Video processing finished")


def load_camera_config(config_path: str) -> dict:
    """Load camera configuration from sensor_config.json."""
    sensor_config_path = os.path.join(config_path, 'sensor_config.json')
    if os.path.exists(sensor_config_path):
        with open(sensor_config_path, 'r') as f:
            return json.load(f)
    return {}


def main():
    parser = argparse.ArgumentParser(
        description="ArUco marker tracking on video with 3D visualization"
    )

    parser.add_argument("video", type=str, help="Path to video file")

    # Marker settings
    parser.add_argument("--marker-size", type=float, default=0.05,
                       help="Physical marker size in meters (default: 0.05)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50",
                       help="ArUco dictionary type (default: DICT_4X4_50)")

    # Camera settings
    parser.add_argument("--camera-config", type=str, default=None,
                       help="Path to camera config folder")
    parser.add_argument("--config", type=str, default="config",
                       help="Config directory for all cameras (default: config)")

    # Playback settings
    parser.add_argument("--speed", type=float, default=1.0,
                       help="Playback speed multiplier (default: 1.0)")
    parser.add_argument("--no-loop", action="store_true",
                       help="Don't loop video playback")

    args = parser.parse_args()

    # Check video exists
    if not os.path.exists(args.video):
        print(f"Error: Video not found: {args.video}")
        sys.exit(1)

    # Load camera config if specified
    camera_matrix = None
    dist_coeffs = None
    extrinsic_matrix = np.eye(4)

    if args.camera_config:
        config = load_camera_config(args.camera_config)
        if config:
            # Load intrinsics
            if 'Color Intrinsic' in config:
                intr = config['Color Intrinsic']
                camera_matrix = np.array([
                    [intr['Fx'], 0, intr['Cx']],
                    [0, intr['Fy'], intr['Cy']],
                    [0, 0, 1]
                ], dtype=np.float64)
                dist = intr.get('Distortion', {})
                dist_coeffs = np.array([
                    dist.get('K1', 0), dist.get('K2', 0),
                    dist.get('P1', 0), dist.get('P2', 0),
                    dist.get('K3', 0)
                ], dtype=np.float64)

            # Load extrinsic
            if 'Sensor to World Extrinsic' in config:
                extrinsic_matrix = np.array(config['Sensor to World Extrinsic']).reshape(4, 4)

            print(f"Loaded camera config from: {args.camera_config}")

    # Create message bus
    message_bus = MessageBus()

    # Load camera poses for scene
    config_loader = ConfigLoader(args.config)
    camera_poses = config_loader.get_camera_poses()
    print(f"Loaded {len(camera_poses)} camera poses")

    # Create video tracker
    tracker = VideoTrackerWithVisualization(
        video_path=args.video,
        message_bus=message_bus,
        marker_size_m=args.marker_size,
        dictionary_type=args.dict,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        extrinsic_matrix=extrinsic_matrix,
        playback_speed=args.speed,
        loop=not args.no_loop
    )

    # Create viewer
    viewer = MarkerViewer(
        message_bus=message_bus,
        show_trajectory=True
    )

    # Setup viewer with camera poses
    viewer.setup(camera_poses)

    print("\nStarting visualization...")
    print("Controls: Q=quit, P=pause, R=reset view, T=toggle trails, C=clear trails")

    try:
        # Start tracker
        tracker.start()

        # Run viewer (blocks until closed)
        viewer.run()

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        tracker.stop()
        print("Done.")


if __name__ == "__main__":
    main()
