#!/usr/bin/env python3
"""
Run ArUco marker tracking on a video file.

This script reads frames from a video file, detects ArUco markers,
and displays the results with optional 3D visualization.

Usage:
    # Basic tracking with visualization
    python scripts/run_video_tracker.py test_video/test_video.mp4

    # Track specific marker size
    python scripts/run_video_tracker.py test_video/test_video.mp4 --marker-size 0.05

    # Use different ArUco dictionary
    python scripts/run_video_tracker.py test_video/test_video.mp4 --dict DICT_6X6_50

    # Save output video
    python scripts/run_video_tracker.py test_video/test_video.mp4 --output output.mp4
"""

import os
import sys
import time
import argparse
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from cv2 import aruco

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


class VideoArucoTracker:
    """ArUco marker tracker for video files."""

    def __init__(self,
                 marker_size_m: float = 0.05,
                 dictionary_type: str = "DICT_4X4_50",
                 camera_matrix: np.ndarray = None,
                 dist_coeffs: np.ndarray = None):
        """
        Initialize video ArUco tracker.

        Args:
            marker_size_m: Physical size of markers in meters
            dictionary_type: ArUco dictionary to use
            camera_matrix: Camera intrinsic matrix (3x3)
            dist_coeffs: Distortion coefficients
        """
        self.marker_size_m = marker_size_m
        self.dictionary_type = dictionary_type

        # Camera parameters
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)

        # ArUco detector
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

    def set_camera_intrinsics(self, width: int, height: int):
        """Set default camera intrinsics based on frame size."""
        if self.camera_matrix is None:
            # Approximate focal length based on frame width
            focal_length = width
            self.camera_matrix = np.array([
                [focal_length, 0, width / 2],
                [0, focal_length, height / 2],
                [0, 0, 1]
            ], dtype=np.float64)

    def detect_markers(self, frame: np.ndarray):
        """
        Detect ArUco markers in a frame.

        Args:
            frame: BGR color image

        Returns:
            List of detection dictionaries with id, corners, position, orientation
        """
        detections = []

        # Convert to grayscale for detection
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

        # Process each detected marker
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]

            detection = {
                'id': int(marker_id),
                'corners': marker_corners,
            }

            # Estimate pose if camera matrix is available
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
                    detection['rvec'] = rvec
                    detection['tvec'] = tvec
                    detection['position'] = tvec.flatten()

            detections.append(detection)

        return detections

    def draw_detections(self, frame: np.ndarray, detections: list, draw_axes: bool = True):
        """
        Draw detected markers on frame.

        Args:
            frame: BGR image to draw on
            detections: List of detection dictionaries
            draw_axes: Whether to draw coordinate axes

        Returns:
            Frame with detections drawn
        """
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

            # Draw position if available
            if 'position' in detection:
                pos = detection['position']
                cv2.putText(result, f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})m",
                           (center[0] - 80, center[1] + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Draw coordinate axes
            if draw_axes and 'rvec' in detection and 'tvec' in detection:
                cv2.drawFrameAxes(result, self.camera_matrix, self.dist_coeffs,
                                 detection['rvec'], detection['tvec'],
                                 self.marker_size_m * 0.5)

        return result


def main():
    parser = argparse.ArgumentParser(
        description="ArUco marker tracking on video file",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument("video", type=str,
                       help="Path to video file")

    # Marker settings
    parser.add_argument("--marker-size", type=float, default=0.05,
                       help="Physical marker size in meters (default: 0.05 = 50mm)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50",
                       help="ArUco dictionary type (default: DICT_4X4_50)")

    # Output settings
    parser.add_argument("--output", "-o", type=str, default=None,
                       help="Output video path (optional)")
    parser.add_argument("--no-display", action="store_true",
                       help="Don't display video window")
    parser.add_argument("--loop", action="store_true",
                       help="Loop video playback")

    args = parser.parse_args()

    # Check video file exists
    if not os.path.exists(args.video):
        print(f"Error: Video file not found: {args.video}")
        sys.exit(1)

    # Open video
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"Error: Could not open video: {args.video}")
        sys.exit(1)

    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"\nVideo: {args.video}")
    print(f"Resolution: {width}x{height}")
    print(f"FPS: {fps:.2f}")
    print(f"Total frames: {total_frames}")
    print(f"\nMarker settings:")
    print(f"  Size: {args.marker_size * 1000:.1f}mm")
    print(f"  Dictionary: {args.dict}")

    # Create tracker
    tracker = VideoArucoTracker(
        marker_size_m=args.marker_size,
        dictionary_type=args.dict
    )
    tracker.set_camera_intrinsics(width, height)

    # Setup output video writer
    writer = None
    if args.output:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(args.output, fourcc, fps, (width, height))
        print(f"\nSaving output to: {args.output}")

    # Create window
    if not args.no_display:
        window_name = "ArUco Video Tracker"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, min(width, 1280), min(height, 720))

    print("\nControls:")
    print("  Space - Pause/Resume")
    print("  Q/ESC - Quit")
    print("  S     - Save current frame")
    print("")

    paused = False
    frame_count = 0
    detection_count = 0
    marker_0_count = 0

    try:
        while True:
            if not paused:
                ret, frame = cap.read()

                if not ret:
                    if args.loop:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    else:
                        break

                frame_count += 1

                # Detect markers
                detections = tracker.detect_markers(frame)

                # Track marker_0 specifically
                for det in detections:
                    if det['id'] == 0:
                        marker_0_count += 1
                        if 'position' in det:
                            pos = det['position']
                            print(f"Frame {frame_count}: Marker 0 detected at "
                                  f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})m")
                        else:
                            print(f"Frame {frame_count}: Marker 0 detected")

                if detections:
                    detection_count += len(detections)

                # Draw detections
                display = tracker.draw_detections(frame, detections)

                # Add info overlay
                cv2.putText(display, f"Frame: {frame_count}/{total_frames}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display, f"Detected: {len(detections)} markers",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Save to output video
                if writer:
                    writer.write(display)

            else:
                # When paused, use the last frame
                pass

            # Display
            if not args.no_display:
                cv2.imshow(window_name, display)

                key = cv2.waitKey(int(1000 / fps)) & 0xFF

                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space
                    paused = not paused
                    print("Paused" if paused else "Resumed")
                elif key == ord('s'):  # Save frame
                    save_path = f"frame_{frame_count:05d}.png"
                    cv2.imwrite(save_path, display)
                    print(f"Saved: {save_path}")

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

    # Print summary
    print(f"\n--- Summary ---")
    print(f"Processed frames: {frame_count}")
    print(f"Total detections: {detection_count}")
    print(f"Marker 0 detections: {marker_0_count}")
    if frame_count > 0:
        print(f"Detection rate: {detection_count / frame_count:.2f} markers/frame")
        print(f"Marker 0 presence: {marker_0_count / frame_count * 100:.1f}% of frames")


if __name__ == "__main__":
    main()
