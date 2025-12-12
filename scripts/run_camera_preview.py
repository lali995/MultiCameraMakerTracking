#!/usr/bin/env python3
"""
Standalone Multi-Camera Preview Window.

Shows all camera feeds in a grid with detection highlighting.
Run this alongside the main tracker for both visualizations.

Usage:
    # First, start the tracker (without --preview):
    python scripts/run_multi_camera_tracker.py --calibration calibration/calibration_result.json

    # Then in another terminal, run this preview:
    python scripts/run_camera_preview.py --calibration calibration/calibration_result.json
"""

import os
import sys
import time
import argparse
import cv2
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.streaming.stream_server import StreamServer
from src.core.config_loader import ConfigLoader

# Camera colors for consistent identification
CAMERA_COLORS = [
    (0, 255, 0),    # Green
    (0, 165, 255),  # Orange
    (255, 0, 255),  # Magenta
    (255, 255, 0),  # Cyan
    (0, 255, 255),  # Yellow
    (255, 0, 0),    # Blue
]


def get_camera_color(index):
    return CAMERA_COLORS[index % len(CAMERA_COLORS)]


def get_short_id(camera_id):
    if '-' in camera_id:
        return camera_id.split('-')[-1]
    return camera_id[-8:] if len(camera_id) > 8 else camera_id


def main():
    parser = argparse.ArgumentParser(description="Multi-Camera Preview Window")
    parser.add_argument("--calibration", "-c", type=str, required=True,
                       help="Path to calibration_result.json")
    parser.add_argument("--port", "-p", type=int, default=5556,
                       help="Stream server port (default: 5556 - different from main tracker)")
    parser.add_argument("--reference", "-r", type=str, default="250514",
                       help="Reference camera ID")
    args = parser.parse_args()

    print("=" * 60)
    print("Multi-Camera Preview")
    print("=" * 60)

    # Load camera configs
    config_loader = ConfigLoader('config', reference_camera_id=args.reference)
    camera_poses = config_loader.load_from_calibration_file(args.calibration, args.reference)

    if not camera_poses:
        print("ERROR: No cameras found in calibration file")
        sys.exit(1)

    camera_ids = [p.camera_id for p in camera_poses]
    print(f"Loaded {len(camera_ids)} cameras")

    # Assign colors
    camera_colors = {cam_id: get_camera_color(i) for i, cam_id in enumerate(camera_ids)}

    # Start stream server on different port
    print(f"\nStarting stream server on port {args.port}...")
    print("NOTE: Camera clients must connect to THIS port for preview")
    server = StreamServer(port=args.port)
    server.start()

    # Preview settings
    cell_size = (480, 270)
    cols = 2 if len(camera_ids) <= 4 else 3
    rows = (len(camera_ids) + cols - 1) // cols

    # Create window
    window_name = "Multi-Camera Preview"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, cols * cell_size[0], rows * cell_size[1] + 60)

    print(f"\nPreview started. Press Q to quit.")
    print("Waiting for camera connections...")

    # FPS tracking
    fps_counters = {}
    last_frame_times = {}

    try:
        while True:
            # Get frames from all cameras
            connected_cameras = server.get_camera_ids()

            cells = []
            for cam_id in camera_ids:
                # Find matching frame
                frame = None
                matched_id = None
                for conn_id in connected_cameras:
                    if cam_id in conn_id or conn_id in cam_id:
                        frame = server.get_frame(conn_id)
                        matched_id = conn_id
                        break

                color = camera_colors.get(cam_id, (128, 128, 128))
                short_id = get_short_id(cam_id)

                # Create cell
                cell = np.zeros((cell_size[1], cell_size[0], 3), dtype=np.uint8)

                if frame is not None:
                    # Update FPS
                    now = time.time()
                    if matched_id in last_frame_times:
                        dt = now - last_frame_times[matched_id]
                        if dt > 0:
                            fps_counters[matched_id] = fps_counters.get(matched_id, 0) * 0.9 + (1.0 / dt) * 0.1
                    last_frame_times[matched_id] = now

                    # Resize frame
                    h, w = frame.shape[:2]
                    scale = min(cell_size[0] / w, cell_size[1] / h)
                    new_w, new_h = int(w * scale), int(h * scale)
                    resized = cv2.resize(frame, (new_w, new_h))

                    # Center in cell
                    x_off = (cell_size[0] - new_w) // 2
                    y_off = (cell_size[1] - new_h) // 2
                    cell[y_off:y_off+new_h, x_off:x_off+new_w] = resized

                    # Border
                    cv2.rectangle(cell, (0, 0), (cell_size[0]-1, cell_size[1]-1), color, 2)

                    # Label
                    fps = fps_counters.get(matched_id, 0)
                    cv2.rectangle(cell, (0, 0), (cell_size[0], 30), (40, 40, 40), -1)
                    cv2.putText(cell, f"{short_id} | {fps:.1f} FPS", (10, 22),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                else:
                    # No signal
                    cv2.putText(cell, "NO SIGNAL", (cell_size[0]//2 - 60, cell_size[1]//2),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.rectangle(cell, (0, 0), (cell_size[0]-1, cell_size[1]-1), (64, 64, 64), 2)
                    cv2.rectangle(cell, (0, 0), (cell_size[0], 30), (40, 40, 40), -1)
                    cv2.putText(cell, short_id, (10, 22),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)

                cells.append(cell)

            # Pad cells
            empty = np.zeros((cell_size[1], cell_size[0], 3), dtype=np.uint8)
            while len(cells) < rows * cols:
                cells.append(empty.copy())

            # Build grid
            grid_rows = []
            for r in range(rows):
                row_cells = cells[r * cols:(r + 1) * cols]
                grid_rows.append(np.hstack(row_cells))
            grid = np.vstack(grid_rows)

            # Status bar
            bar = np.zeros((60, cols * cell_size[0], 3), dtype=np.uint8)
            bar[:] = (30, 30, 30)
            connected = len(connected_cameras)
            cv2.putText(bar, f"Connected: {connected}/{len(camera_ids)} cameras | Port: {args.port}",
                       (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            # Legend
            x = 10
            y = 50
            for i, cam_id in enumerate(camera_ids):
                color = camera_colors[cam_id]
                short = get_short_id(cam_id)
                cv2.circle(bar, (x, y - 5), 6, color, -1)
                cv2.putText(bar, short, (x + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                x += 100

            # Combine
            final = np.vstack([grid, bar])
            cv2.imshow(window_name, final)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        cv2.destroyAllWindows()
        server.cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
