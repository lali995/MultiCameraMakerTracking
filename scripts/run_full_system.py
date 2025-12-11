#!/usr/bin/env python3
"""
Main entry point for the ArUco marker tracking system.
Launches the tracker and visualization together.

Usage:
    python scripts/run_full_system.py [options]

Options:
    --config PATH       Path to config directory (default: config)
    --num-markers N     Number of simulated markers (default: 3)
    --motion TYPE       Motion type: circular, linear, figure8, random (default: circular)
    --no-trajectory     Disable marker trails
    --speed SPEED       Motion speed multiplier (default: 1.0)
"""
import sys
import os
import argparse

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.core.message_bus import MessageBus
from src.core.config_loader import ConfigLoader
from src.tracking.simulated_tracker import SimulatedTracker
from src.visualization.viewer import MarkerViewer


def parse_args():
    parser = argparse.ArgumentParser(
        description='ArUco Marker Tracking System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Controls:
  Q/ESC  - Quit
  P      - Pause/Resume
  R      - Reset camera view
  C      - Clear marker trails
  T      - Toggle trail visibility

Examples:
  python scripts/run_full_system.py
  python scripts/run_full_system.py --num-markers 5 --motion figure8
  python scripts/run_full_system.py --config /path/to/config --speed 2.0
  python scripts/run_full_system.py --calibration calibration/calibration_result.json --reference 250514
        """
    )

    parser.add_argument(
        '--config',
        type=str,
        default='config',
        help='Path to config directory (default: config)'
    )
    parser.add_argument(
        '--calibration',
        type=str,
        default=None,
        help='Path to calibration_result.json file (overrides --config)'
    )
    parser.add_argument(
        '--reference',
        type=str,
        default=None,
        help='Camera ID substring to use as reference/origin (e.g., "250514")'
    )
    parser.add_argument(
        '--num-markers',
        type=int,
        default=3,
        help='Number of simulated markers (default: 3)'
    )
    parser.add_argument(
        '--motion',
        type=str,
        default='circular',
        choices=['circular', 'linear', 'figure8', 'random'],
        help='Motion type (default: circular)'
    )
    parser.add_argument(
        '--no-trajectory',
        action='store_true',
        help='Disable marker trajectory trails'
    )
    parser.add_argument(
        '--speed',
        type=float,
        default=1.0,
        help='Motion speed multiplier (default: 1.0)'
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=30.0,
        help='Tracker update rate in Hz (default: 30.0)'
    )

    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 50)
    print("ArUco Marker Tracking System")
    print("=" * 50)
    if args.calibration:
        print(f"Calibration file: {args.calibration}")
        print(f"Reference camera: {args.reference or '250514 (default)'}")
    else:
        print(f"Config path: {args.config}")
    print(f"Markers: {args.num_markers}")
    print(f"Motion: {args.motion}")
    print(f"Speed: {args.speed}x")
    print(f"Rate: {args.rate} Hz")
    print(f"Trajectories: {'disabled' if args.no_trajectory else 'enabled'}")
    print("=" * 50)

    # Create message bus
    bus = MessageBus()

    # Load camera configurations
    print("\nLoading camera configurations...")
    config_loader = ConfigLoader(args.config, reference_camera_id=args.reference)

    if args.calibration:
        # Load from calibration_result.json with reference camera
        reference = args.reference or '250514'  # Default reference camera
        camera_poses = config_loader.load_from_calibration_file(args.calibration, reference)
    else:
        # Load from sensor_config.json files in config directory
        camera_poses = config_loader.get_camera_poses()

    print(f"Found {len(camera_poses)} cameras")

    for pose in camera_poses:
        print(f"  - {pose.camera_id} at position ({pose.position[0]:.3f}, {pose.position[1]:.3f}, {pose.position[2]:.3f})")

    # Configure tracker
    tracker_config = {
        'num_markers': args.num_markers,
        'motion_type': args.motion,
        'speed': args.speed,
        'rate_hz': args.rate,
        'detection_noise': 0.005,  # Small noise for realism
        'drop_rate': 0.0,  # No dropped detections in simulation
    }

    # Create tracker
    print("\nInitializing tracker...")
    tracker = SimulatedTracker(bus, tracker_config)

    # Create viewer
    print("Initializing viewer...")
    viewer = MarkerViewer(
        bus,
        window_title="ArUco Marker Tracking",
        show_trajectory=not args.no_trajectory
    )

    # Setup viewer with camera poses
    viewer.setup(camera_poses)

    # Start tracker in background thread
    print("\nStarting tracker...")
    tracker.start()

    # Run viewer in main thread (required for Open3D)
    print("Starting visualization...\n")
    try:
        viewer.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Cleanup
        print("\nShutting down...")
        tracker.stop()
        print("Done.")


if __name__ == '__main__':
    main()
