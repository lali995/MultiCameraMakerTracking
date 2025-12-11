"""
Multi-Camera Stream Server

Receives video streams from multiple camera clients over ZMQ.
Provides frames to trackers for ArUco marker detection.
"""

import threading
import time
from collections import defaultdict
from typing import Dict, Callable, Optional, Tuple

import cv2
import zmq
import numpy as np


class StreamServer:
    """
    Server that receives video streams from multiple camera clients.

    Each client sends JPEG-encoded frames with metadata over ZMQ PUSH/PULL.
    The server stores the latest frame from each camera and provides
    access via get_frame() or callbacks.
    """

    def __init__(self, port: int = 5555):
        """
        Initialize stream server.

        Args:
            port: Port to listen on for incoming streams
        """
        self.port = port
        self.running = False

        # ZMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.bind(f"tcp://*:{port}")
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout

        # Frame storage - latest frame from each camera
        self._frames: Dict[str, np.ndarray] = {}
        self._metadata: Dict[str, dict] = {}
        self._frame_lock = threading.Lock()
        self._receive_thread: Optional[threading.Thread] = None

        # Stats
        self._frame_counts = defaultdict(int)
        self._last_stats_time = time.time()

        # Callbacks
        self._frame_callback: Optional[Callable] = None

    def set_frame_callback(self, callback: Callable[[str, np.ndarray, dict], None]):
        """
        Set callback for new frames.

        Args:
            callback: Function(camera_id, frame, metadata) called for each new frame
        """
        self._frame_callback = callback

    def get_frame(self, camera_id: str) -> Optional[np.ndarray]:
        """
        Get latest frame from a specific camera.

        Args:
            camera_id: ID of the camera

        Returns:
            Latest frame or None if no frame available
        """
        with self._frame_lock:
            frame = self._frames.get(camera_id)
            return frame.copy() if frame is not None else None

    def get_frame_with_metadata(self, camera_id: str) -> Tuple[Optional[np.ndarray], Optional[dict]]:
        """
        Get latest frame and metadata from a specific camera.

        Args:
            camera_id: ID of the camera

        Returns:
            Tuple of (frame, metadata) or (None, None) if not available
        """
        with self._frame_lock:
            frame = self._frames.get(camera_id)
            metadata = self._metadata.get(camera_id)
            return (
                frame.copy() if frame is not None else None,
                metadata.copy() if metadata is not None else None
            )

    def get_all_frames(self) -> Dict[str, np.ndarray]:
        """
        Get latest frames from all connected cameras.

        Returns:
            Dictionary mapping camera_id to frame
        """
        with self._frame_lock:
            return {k: v.copy() for k, v in self._frames.items()}

    def get_camera_ids(self) -> list:
        """
        Get list of connected camera IDs.

        Returns:
            List of camera ID strings
        """
        with self._frame_lock:
            return list(self._frames.keys())

    def start(self):
        """Start receiving frames in background thread."""
        if self.running:
            return

        self.running = True
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()
        print(f"Stream server listening on port {self.port}...")

    def stop(self):
        """Stop receiving frames."""
        self.running = False
        if self._receive_thread is not None:
            self._receive_thread.join(timeout=2.0)
            self._receive_thread = None

    def _receive_loop(self):
        """Main receive loop running in background thread."""
        while self.running:
            try:
                # Receive metadata
                metadata = self.socket.recv_json()
                # Receive frame data
                frame_data = self.socket.recv()

                # Decode frame
                frame = cv2.imdecode(
                    np.frombuffer(frame_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )

                if frame is not None:
                    camera_id = metadata.get('camera_id', 'unknown')

                    with self._frame_lock:
                        self._frames[camera_id] = frame
                        self._metadata[camera_id] = metadata

                    self._frame_counts[camera_id] += 1

                    # Call callback if set
                    if self._frame_callback:
                        self._frame_callback(camera_id, frame, metadata)

                # Print stats every 5 seconds
                now = time.time()
                if now - self._last_stats_time >= 5.0:
                    self._print_stats()
                    self._last_stats_time = now

            except zmq.Again:
                continue  # Timeout, check if still running
            except Exception as e:
                if self.running:
                    print(f"Error receiving frame: {e}")

    def _print_stats(self):
        """Print connection statistics."""
        if not self._frame_counts:
            return

        elapsed = 5.0
        stats = []
        for cam_id, count in self._frame_counts.items():
            fps = count / elapsed
            stats.append(f"{cam_id}: {fps:.1f} FPS")

        print(f"Connected cameras - {', '.join(stats)}")
        self._frame_counts.clear()

    def cleanup(self):
        """Clean up resources."""
        self.stop()
        self.socket.close()
        self.context.term()

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.cleanup()
        return False
