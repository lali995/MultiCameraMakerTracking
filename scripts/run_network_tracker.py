#!/usr/bin/env python3
"""
Run ArUco marker tracking with network camera streams.

Usage:
    python scripts/run_network_tracker.py --preview
"""

import os
import sys
import time
import argparse
import signal

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np

from src.streaming.stream_server import StreamServer

# ArUco setup
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
}


def detect_markers(frame, aruco_dict, aruco_params, marker_size=0.05):
    """Detect ArUco markers in frame."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    return corners, ids


def draw_markers(frame, corners, ids):
    """Draw detected markers on frame."""
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, corner in enumerate(corners):
            center = corner[0].mean(axis=0).astype(int)
            cv2.putText(frame, f"ID:{ids[i][0]}",
                       (center[0]-20, center[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame


def main():
    parser = argparse.ArgumentParser(description="Network ArUco Tracker")
    parser.add_argument("--port", "-p", type=int, default=5555)
    parser.add_argument("--marker-size", type=float, default=0.05)
    parser.add_argument("--dict", type=str, default="DICT_4X4_50")
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()

    # ArUco setup
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT.get(args.dict, cv2.aruco.DICT_4X4_50))
    if hasattr(cv2.aruco, 'DetectorParameters'):
        aruco_params = cv2.aruco.DetectorParameters()
    else:
        aruco_params = cv2.aruco.DetectorParameters_create()

    # Start server
    print(f"Starting stream server on port {args.port}...")
    server = StreamServer(port=args.port)
    server.start()

    running = True
    def signal_handler(sig, frame):
        nonlocal running
        print("\nStopping...")
        running = False
    signal.signal(signal.SIGINT, signal_handler)

    print("Waiting for camera connections...")
    print("Press 'q' to quit\n")

    preview_windows = {}
    frame_count = 0
    marker_detections = 0
    start_time = time.time()
    last_marker_ids = None

    try:
        while running:
            # Get frames from all cameras
            for camera_id in server.get_camera_ids():
                frame = server.get_frame(camera_id)
                if frame is None:
                    continue

                # Detect markers
                corners, ids = detect_markers(frame, aruco_dict, aruco_params, args.marker_size)

                # Track marker detections
                num_markers = len(ids) if ids is not None else 0
                if num_markers > 0:
                    marker_detections += num_markers
                    # Log when markers change
                    current_ids = tuple(sorted(ids.flatten().tolist())) if ids is not None else ()
                    if current_ids != last_marker_ids:
                        print(f"[{camera_id}] Detected markers: {list(ids.flatten())}")
                        last_marker_ids = current_ids

                # Draw results
                display = draw_markers(frame.copy(), corners, ids)

                # Add info
                cv2.putText(display, f"Markers: {num_markers}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display, f"Camera: {camera_id}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                # Show preview
                if args.preview:
                    if camera_id not in preview_windows:
                        window_name = f"Camera: {camera_id}"
                        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                        preview_windows[camera_id] = window_name
                    cv2.imshow(preview_windows[camera_id], display)

                frame_count += 1

            # Handle key press
            if args.preview:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

            # Print FPS and detection stats
            elapsed = time.time() - start_time
            if elapsed >= 2.0:
                fps = frame_count / elapsed
                cameras = server.get_camera_ids()
                if cameras:
                    print(f"Cameras: {cameras}, FPS: {fps:.1f}, Markers detected: {marker_detections}")
                frame_count = 0
                marker_detections = 0
                start_time = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        server.cleanup()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
