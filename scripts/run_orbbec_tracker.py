#!/usr/bin/env python3
"""
Run ArUco marker tracking with Orbbec RGB-D camera.

This script captures frames from an Orbbec camera, detects ArUco markers,
and publishes their 3D poses to the visualization system.

Usage:
    # Basic tracking with visualization
    python scripts/run_orbbec_tracker.py

    # Track 50mm markers
    python scripts/run_orbbec_tracker.py --marker-size 0.05

    # Use different ArUco dictionary
    python scripts/run_orbbec_tracker.py --dict DICT_6X6_50

    # Show camera preview window
    python scripts/run_orbbec_tracker.py --preview

    # Load camera extrinsic from config
    python scripts/run_orbbec_tracker.py --camera-config config/my_camera
"""

import os
import sys
import time
import argparse
import json
import signal
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from src.core.message_bus import MessageBus
from src.core.config_loader import ConfigLoader
from src.visualization.viewer import MarkerViewer


def load_camera_extrinsic(config_path: str) -> np.ndarray:
    """Load camera extrinsic matrix from sensor_config.json."""
    sensor_config_path = os.path.join(config_path, 'sensor_config.json')
    if os.path.exists(sensor_config_path):
        with open(sensor_config_path, 'r') as f:
            config = json.load(f)
        if 'Sensor to World Extrinsic' in config:
            return np.array(config['Sensor to World Extrinsic'])
    return np.eye(4)


def main():
    parser = argparse.ArgumentParser(
        description="ArUco marker tracking with Orbbec camera",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    # Marker settings
    parser.add_argument("--marker-size", type=float, default=0.05,
                       help="Physical marker size in meters (default: 0.05 = 50mm)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50",
                       help="ArUco dictionary type (default: DICT_4X4_50)")

    # Camera settings
    parser.add_argument("--color-width", type=int, default=1280,
                       help="Color stream width (default: 1280)")
    parser.add_argument("--color-height", type=int, default=720,
                       help="Color stream height (default: 720)")
    parser.add_argument("--depth-width", type=int, default=640,
                       help="Depth stream width (default: 640)")
    parser.add_argument("--depth-height", type=int, default=480,
                       help="Depth stream height (default: 480)")
    parser.add_argument("--camera-config", type=str, default=None,
                       help="Path to camera config folder with sensor_config.json")

    # Tracking settings
    parser.add_argument("--rate", type=float, default=30.0,
                       help="Tracking rate in Hz (default: 30)")

    # Display settings
    parser.add_argument("--preview", action="store_true",
                       help="Show camera preview window with detections")
    parser.add_argument("--no-visualization", action="store_true",
                       help="Disable 3D visualization")

    # Scene settings
    parser.add_argument("--config", type=str, default="config",
                       help="Config directory for camera poses (default: config)")
    parser.add_argument("--no-trajectory", action="store_true",
                       help="Disable marker trajectory trails")

    args = parser.parse_args()

    # Check for Orbbec SDK
    try:
        from src.tracking import OrbbecArucoTracker, ORBBEC_AVAILABLE
        if not ORBBEC_AVAILABLE:
            raise ImportError("Orbbec tracker not available")
    except ImportError as e:
        print("Error: Orbbec camera support not available.")
        print("Please install pyorbbecsdk:")
        print("  pip install pyorbbecsdk")
        print("\nAlternatively, run with simulated tracking:")
        print("  python scripts/run_full_system.py")
        sys.exit(1)

    # Load camera extrinsic if config provided
    extrinsic_matrix = None
    if args.camera_config:
        extrinsic_matrix = load_camera_extrinsic(args.camera_config)
        print(f"Loaded camera extrinsic from: {args.camera_config}")

    # Create message bus
    message_bus = MessageBus()

    # Create tracker
    print(f"\nInitializing Orbbec ArUco tracker...")
    print(f"  Marker size: {args.marker_size * 1000:.1f}mm")
    print(f"  Dictionary: {args.dict}")
    print(f"  Color resolution: {args.color_width}x{args.color_height}")
    print(f"  Depth resolution: {args.depth_width}x{args.depth_height}")

    tracker = OrbbecArucoTracker(
        message_bus=message_bus,
        marker_size_m=args.marker_size,
        dictionary_type=args.dict,
        rate_hz=args.rate,
        color_width=args.color_width,
        color_height=args.color_height,
        depth_width=args.depth_width,
        depth_height=args.depth_height,
        extrinsic_matrix=extrinsic_matrix
    )

    # Create viewer if visualization enabled
    viewer = None
    if not args.no_visualization:
        # Load camera poses for scene context
        config_loader = ConfigLoader(args.config)
        camera_poses = config_loader.get_camera_poses()

        viewer = MarkerViewer(
            message_bus=message_bus,
            camera_poses=camera_poses,
            show_trajectory=not args.no_trajectory
        )

    # Preview window callback
    preview_window = None
    if args.preview:
        preview_window = "Orbbec ArUco Tracker"
        cv2.namedWindow(preview_window, cv2.WINDOW_NORMAL)

        def frame_callback(color_image, depth_image, detections):
            # Draw detections on image
            display = tracker.draw_detections(color_image, detections)

            # Add info text
            cv2.putText(display, f"Detected: {len(detections)} markers",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display, f"Press 'q' to quit",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv2.imshow(preview_window, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                tracker.stop()
                if viewer:
                    viewer.request_close()

        tracker.set_frame_callback(frame_callback)

    # Handle Ctrl+C gracefully
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        print("\nShutting down...")
        running = False
        tracker.stop()
        if viewer:
            viewer.request_close()

    signal.signal(signal.SIGINT, signal_handler)

    # Start tracking
    try:
        print("\nStarting tracker...")
        tracker.start()
        print("Tracker started. Looking for ArUco markers...")
        print("\nControls:")
        if args.preview:
            print("  - Press 'q' in preview window to quit")
        if viewer:
            print("  - Press 'Q' or 'ESC' in 3D view to quit")
            print("  - Press 'P' to pause/resume")
            print("  - Press 'R' to reset view")
            print("  - Press 'T' to toggle trails")
        print("  - Press Ctrl+C to quit\n")

        if viewer:
            # Run viewer in main thread (blocks until closed)
            viewer.run()
        else:
            # Just run tracker
            while running and tracker._running:
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupted")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("Stopping tracker...")
        tracker.stop()

        if preview_window:
            cv2.destroyAllWindows()

        print("Done.")


if __name__ == "__main__":
    main()
