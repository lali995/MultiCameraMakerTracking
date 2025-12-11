#!/usr/bin/env python3
"""
Run ArUco marker tracking with network camera streams and 3D visualization.

Detects markers, estimates 3D pose relative to camera, and visualizes in Open3D.

Usage:
    python scripts/run_network_tracker_with_visualization.py --preview
    python scripts/run_network_tracker_with_visualization.py --calibration calibration/calibration_result.json --reference 250514
"""

import os
import sys
import time
import argparse
import threading
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from src.streaming.stream_server import StreamServer
from src.core.message_bus import MessageBus
from src.core.data_types import MarkerPose, CameraPose
from src.core.config_loader import ConfigLoader
from src.visualization.viewer import MarkerViewer

# ArUco setup
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
}


class NetworkTrackerWithVisualization:
    """Network stream ArUco tracker with 3D visualization."""

    def __init__(self,
                 message_bus: MessageBus,
                 server: StreamServer,
                 camera_id_filter: str = None,
                 marker_size_m: float = 0.05,
                 dictionary_type: str = "DICT_4X4_50",
                 camera_matrix: np.ndarray = None,
                 dist_coeffs: np.ndarray = None,
                 extrinsic_matrix: np.ndarray = None):
        """
        Initialize network tracker with visualization.

        Args:
            message_bus: Message bus for publishing poses
            server: Stream server receiving frames
            camera_id_filter: Only process cameras with this substring in ID
            marker_size_m: Physical marker size in meters
            dictionary_type: ArUco dictionary type
            camera_matrix: Camera intrinsic matrix
            dist_coeffs: Distortion coefficients
            extrinsic_matrix: Camera to world transform (4x4)
        """
        self.message_bus = message_bus
        self.server = server
        self.camera_id_filter = camera_id_filter
        self.marker_size_m = marker_size_m

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

        # Threading
        self._running = False
        self._thread = None

        # Stats
        self.frame_count = 0
        self.detection_count = 0

        # Shared frame for preview (thread-safe)
        self._latest_frame = None
        self._latest_detections = []
        self._frame_lock = threading.Lock()

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

        # Set default camera matrix if not provided
        if self.camera_matrix is None:
            h, w = frame.shape[:2]
            focal_length = w
            self.camera_matrix = np.array([
                [focal_length, 0, w / 2],
                [0, focal_length, h / 2],
                [0, 0, 1]
            ], dtype=np.float64)

        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]

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
                    'position_camera': position_camera,
                    'corners': marker_corners,
                    'rvec': rvec,
                    'tvec': tvec
                })

        return detections

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
            pos = detection['position']
            cv2.putText(result, f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                       (center[0] - 80, center[1] + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Draw axes
            if 'rvec' in detection and 'tvec' in detection:
                cv2.drawFrameAxes(result, self.camera_matrix, self.dist_coeffs,
                                 detection['rvec'], detection['tvec'],
                                 self.marker_size_m * 0.5)

        # Add info
        cv2.putText(result, f"Markers: {len(detections)}",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return result

    def start(self):
        """Start processing in background thread."""
        self._running = True
        self._thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop processing."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _processing_loop(self):
        """Main processing loop."""
        last_log_time = time.time()

        while self._running:
            processed = False

            for camera_id in self.server.get_camera_ids():
                # Filter by camera ID if specified
                if self.camera_id_filter and self.camera_id_filter not in camera_id:
                    continue

                frame = self.server.get_frame(camera_id)
                if frame is None:
                    continue

                processed = True
                self.frame_count += 1

                # Detect and publish markers
                detections = self.detect_and_publish(frame)
                self.detection_count += len(detections)

                # Store frame for preview (thread-safe)
                with self._frame_lock:
                    self._latest_frame = frame.copy()
                    self._latest_detections = detections.copy()

                # Log detections
                if detections:
                    for det in detections:
                        pos = det['position']
                        pos_cam = det['position_camera']
                        print(f"Marker {det['id']}: camera({pos_cam[0]:.3f}, {pos_cam[1]:.3f}, {pos_cam[2]:.3f}) "
                              f"-> world({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

            # Log stats periodically
            if time.time() - last_log_time > 5.0:
                print(f"Frames: {self.frame_count}, Detections: {self.detection_count}")
                last_log_time = time.time()

            if not processed:
                time.sleep(0.01)

    def get_latest_frame(self):
        """Get the latest frame with detections drawn (thread-safe)."""
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self.draw_detections(self._latest_frame.copy(), self._latest_detections)


def main():
    parser = argparse.ArgumentParser(
        description="Network ArUco Tracker with 3D Visualization"
    )

    # Network settings
    parser.add_argument("--port", "-p", type=int, default=5555,
                       help="Stream server port (default: 5555)")
    parser.add_argument("--camera-filter", type=str, default="250514",
                       help="Camera ID substring filter (default: 250514)")

    # Marker settings
    parser.add_argument("--marker-size", type=float, default=0.05,
                       help="Physical marker size in meters (default: 0.05)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50",
                       help="ArUco dictionary type (default: DICT_4X4_50)")

    # Calibration settings
    parser.add_argument("--calibration", type=str, default=None,
                       help="Path to calibration_result.json")
    parser.add_argument("--reference", type=str, default="250514",
                       help="Reference camera ID substring (default: 250514)")

    # Display settings
    parser.add_argument("--preview", action="store_true",
                       help="Show video preview window with pose overlay")
    parser.add_argument("--no-visualization", action="store_true",
                       help="Disable 3D visualization (console output only)")

    args = parser.parse_args()

    print("=" * 50)
    print("Network ArUco Tracker with 3D Visualization")
    print("=" * 50)
    print(f"Port: {args.port}")
    print(f"Camera filter: {args.camera_filter}")
    print(f"Marker size: {args.marker_size}m")
    print(f"Reference camera: {args.reference}")
    print("=" * 50)

    # Create message bus
    message_bus = MessageBus()

    # Load camera poses from calibration
    camera_poses = []
    extrinsic_matrix = np.eye(4)

    if args.calibration:
        config_loader = ConfigLoader('config', reference_camera_id=args.reference)
        camera_poses = config_loader.load_from_calibration_file(
            args.calibration, args.reference
        )
        print(f"\nLoaded {len(camera_poses)} cameras from calibration")

        # Find the streaming camera's extrinsic
        for pose in camera_poses:
            if args.camera_filter in pose.camera_id:
                extrinsic_matrix = pose.extrinsic
                print(f"Using extrinsic from: {pose.camera_id}")
                break

    # Start stream server
    print(f"\nStarting stream server on port {args.port}...")
    server = StreamServer(port=args.port)
    server.start()

    # Create tracker
    tracker = NetworkTrackerWithVisualization(
        message_bus=message_bus,
        server=server,
        camera_id_filter=args.camera_filter,
        marker_size_m=args.marker_size,
        dictionary_type=args.dict,
        extrinsic_matrix=extrinsic_matrix
    )

    print("Waiting for camera connections...")
    print("Controls: Q=quit (in preview/3D window), close 3D window to exit")

    # Preview thread function
    preview_running = True
    def run_preview():
        nonlocal preview_running
        cv2.namedWindow("Video Preview", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Video Preview", 960, 540)
        while preview_running and tracker._running:
            frame = tracker.get_latest_frame()
            if frame is not None:
                cv2.imshow("Video Preview", frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                preview_running = False
                break
        cv2.destroyAllWindows()

    try:
        # Start tracker
        tracker.start()

        # Start preview thread if requested
        preview_thread = None
        if args.preview:
            preview_thread = threading.Thread(target=run_preview, daemon=True)
            preview_thread.start()
            print("Video preview started")

        if args.no_visualization:
            # Just run tracker without 3D visualization
            print("Running without 3D visualization. Press Ctrl+C to stop.")
            while tracker._running and preview_running:
                time.sleep(0.1)
        else:
            # Create and run 3D viewer
            viewer = MarkerViewer(
                message_bus=message_bus,
                show_trajectory=True
            )
            viewer.setup(camera_poses)
            print("\nStarting 3D visualization...")
            viewer.run()

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        preview_running = False
        tracker.stop()
        server.cleanup()
        if args.preview:
            time.sleep(0.5)  # Give preview thread time to cleanup
        print("Done.")


if __name__ == "__main__":
    main()
