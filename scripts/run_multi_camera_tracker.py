#!/usr/bin/env python3
"""
Multi-Camera ArUco Marker Tracking Server with 3D Visualization.

This is the SERVER component for multi-camera ArUco marker tracking.
It receives video streams from remote cameras via ZMQ, detects ArUco markers,
fuses pose observations from multiple viewpoints, and provides real-time
3D visualization.

Architecture:
    Remote Cameras (stream_client_standalone.py)
           |
           | ZMQ PUSH/PULL (TCP)
           v
    This Server (run_multi_camera_tracker.py)
           |
           +-> ArUco Detection (per camera)
           +-> Pose Fusion (weighted average, median, or least squares)
           +-> 3D Visualization (Open3D)

Features:
    - Receives video streams from multiple remote cameras via ZMQ
    - Detects ArUco markers in each camera feed independently
    - Fuses marker poses from multiple viewpoints for improved accuracy
    - Real-time 3D visualization with camera frustums and detection lines
    - Integrated viewer shows both 3D scene and camera feed thumbnails
    - Console output mode for headless operation

Requirements:
    pip install opencv-python pyzmq numpy open3d scipy

Usage:
    # Basic usage (requires calibration file)
    python scripts/run_multi_camera_tracker.py --calibration calibration/calibration_result.json

    # With specific reference camera (coordinate origin)
    python scripts/run_multi_camera_tracker.py -c calibration/calibration_result.json --reference 250514

    # With integrated camera preview panel (recommended)
    python scripts/run_multi_camera_tracker.py -c calibration/calibration_result.json --preview

    # Headless mode (console output only, no visualization)
    python scripts/run_multi_camera_tracker.py -c calibration/calibration_result.json --no-visualization

    # Custom marker size and fusion method
    python scripts/run_multi_camera_tracker.py -c calibration/calibration_result.json \\
        --marker-size 0.10 --fusion least_squares

Network Setup:
    1. Start this server on a machine accessible to all cameras
    2. Note the server's IP address
    3. Start stream_client_standalone.py on each camera machine:
       python stream_client_standalone.py -s <server_ip> -c <camera_id>

    The camera_id should match or partially match the CameraID in the
    calibration file for proper pose fusion.

Controls (3D Viewer):
    - Mouse drag: Rotate view
    - Scroll: Zoom
    - Q: Quit
    - Menu > View > Reset View: Reset camera position
"""

import os
import sys
import time
import argparse
import threading

import cv2
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.streaming.stream_server import StreamServer
from src.core.message_bus import MessageBus
from src.core.config_loader import ConfigLoader
from src.core.data_types import CameraPose, FusedMarkerPose
from src.tracking.multi_camera_tracker import MultiCameraTracker, CameraConfig
from src.visualization.viewer import MarkerViewer
from src.visualization.integrated_viewer import IntegratedViewer


def create_camera_configs(camera_poses):
    """
    Create camera configurations from camera poses.

    Args:
        camera_poses: List of CameraPose objects from calibration

    Returns:
        Dict mapping camera_id to CameraConfig
    """
    configs = {}
    for pose in camera_poses:
        configs[pose.camera_id] = CameraConfig(
            camera_id=pose.camera_id,
            extrinsic=pose.extrinsic,
            intrinsic=None,  # Will use default based on frame size
            dist_coeffs=None
        )
    return configs


def create_preview_grid(frames, detections, camera_ids, cols=2):
    """
    Create a grid image showing all camera feeds.

    Args:
        frames: Dict of camera_id -> frame
        detections: Dict of camera_id -> list of detections
        camera_ids: List of camera IDs to show
        cols: Number of columns in grid

    Returns:
        Grid image
    """
    target_size = (640, 360)
    rows = (len(camera_ids) + cols - 1) // cols
    grid_frames = []

    for cam_id in camera_ids:
        # Find matching frame (partial match)
        frame = None
        for fid, f in frames.items():
            if cam_id in fid or fid in cam_id:
                frame = f
                break

        if frame is not None:
            frame = cv2.resize(frame, target_size)

            # Draw camera ID
            short_id = cam_id[-12:] if len(cam_id) > 12 else cam_id
            cv2.putText(frame, short_id, (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # Draw detections
            dets = None
            for did, d in detections.items():
                if cam_id in did or did in cam_id:
                    dets = d
                    break

            if dets:
                cv2.putText(frame, f"Markers: {len(dets)}", (10, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                for det in dets:
                    if det.get('corners') is not None:
                        # Scale corners to resized frame
                        corners = det['corners'].copy()
                        # Assume original was 1280x720, scale to target
                        scale_x = target_size[0] / 1280
                        scale_y = target_size[1] / 720
                        corners[:, 0] *= scale_x
                        corners[:, 1] *= scale_y
                        corners = corners.reshape((4, 2)).astype(int)
                        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

            grid_frames.append(frame)
        else:
            # Placeholder for missing camera
            placeholder = np.zeros((*target_size[::-1], 3), dtype=np.uint8)
            short_id = cam_id[-12:] if len(cam_id) > 12 else cam_id
            cv2.putText(placeholder, f"No signal: {short_id}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            grid_frames.append(placeholder)

    # Pad to fill grid
    while len(grid_frames) < rows * cols:
        grid_frames.append(np.zeros((*target_size[::-1], 3), dtype=np.uint8))

    # Arrange in grid
    row_images = []
    for r in range(rows):
        row_frames = grid_frames[r * cols:(r + 1) * cols]
        row_images.append(np.hstack(row_frames))

    return np.vstack(row_images)


def main():
    parser = argparse.ArgumentParser(
        description="Multi-Camera ArUco Marker Tracking with 3D Visualization"
    )

    # Required
    parser.add_argument("--calibration", "-c", type=str, required=True,
                       help="Path to calibration_result.json")

    # Camera settings
    parser.add_argument("--reference", "-r", type=str, default="250514",
                       help="Reference camera ID substring for coordinate origin")
    parser.add_argument("--port", "-p", type=int, default=5555,
                       help="Stream server port (default: 5555)")

    # Marker settings
    parser.add_argument("--marker-size", type=float, default=0.05,
                       help="Physical marker size in meters (default: 0.05)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50",
                       choices=["DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_50", "DICT_6X6_50"],
                       help="ArUco dictionary type")

    # Fusion settings
    parser.add_argument("--fusion", type=str, default="weighted_average",
                       choices=["weighted_average", "median", "least_squares"],
                       help="Pose fusion method (default: weighted_average)")

    # Display settings
    parser.add_argument("--preview", action="store_true",
                       help="Show video preview grid of all cameras")
    parser.add_argument("--no-visualization", action="store_true",
                       help="Disable 3D visualization (console output only)")

    args = parser.parse_args()

    # --preview now uses integrated Open3D viewer with camera feeds
    # No longer need to disable 3D visualization

    print("=" * 60)
    print("Multi-Camera ArUco Marker Tracking")
    print("=" * 60)
    print(f"Calibration: {args.calibration}")
    print(f"Reference camera: {args.reference}")
    print(f"Fusion method: {args.fusion}")
    print(f"Marker size: {args.marker_size}m")
    print(f"Port: {args.port}")
    print("=" * 60)

    # Load calibration
    config_loader = ConfigLoader('config', reference_camera_id=args.reference)
    camera_poses = config_loader.load_from_calibration_file(
        args.calibration, args.reference
    )

    if not camera_poses:
        print("ERROR: No cameras found in calibration file")
        sys.exit(1)

    print(f"\nLoaded {len(camera_poses)} cameras:")
    for pose in camera_poses:
        print(f"  - {pose.camera_id}")

    # Create camera configs
    camera_configs = create_camera_configs(camera_poses)

    # Create message bus
    message_bus = MessageBus()

    # Start stream server
    print(f"\nStarting stream server on port {args.port}...")
    server = StreamServer(port=args.port)
    server.start()

    # Create multi-camera tracker
    tracker = MultiCameraTracker(
        message_bus=message_bus,
        server=server,
        camera_configs=camera_configs,
        marker_size_m=args.marker_size,
        dictionary_type=args.dict,
        fusion_method=args.fusion
    )

    print("\nWaiting for camera connections...")
    print("Start stream clients on each camera with:")
    print("  python stream_client.py --server <this_ip> --camera-id <camera_id>")
    print("\nControls: Q=quit, P=pause, R=reset view, C=clear trails, T=toggle trails")

    try:
        # Start tracker
        tracker.start()

        if args.no_visualization:
            # Console output only
            print("\nRunning without visualization. Press Ctrl+C to stop.")
            while tracker._running:
                time.sleep(0.1)
        elif args.preview:
            # Integrated viewer with camera feeds in side panel (no separate OpenCV window)
            print("\nStarting integrated 3D visualization with camera feeds...")
            integrated_viewer = IntegratedViewer(
                message_bus=message_bus,
                tracker=tracker,
                window_title="Multi-Camera Tracking (3D + Camera Feeds)"
            )
            integrated_viewer.setup(camera_poses)
            integrated_viewer.run()
        else:
            # 3D viewer only
            viewer = MarkerViewer(
                message_bus=message_bus,
                window_title="Multi-Camera ArUco Tracking",
                show_trajectory=True
            )
            viewer.setup(camera_poses)
            print("\nStarting 3D visualization...")
            print("Detection lines show which cameras see each marker")
            viewer.run()

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        tracker.stop()
        server.cleanup()
        time.sleep(0.5)
        print("Done.")


if __name__ == "__main__":
    main()
