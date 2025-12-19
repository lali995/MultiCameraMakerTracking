"""
Camera Stream Client

Captures video from camera source and streams to server over ZMQ.
Supports RTSP streams (for Docker-based Orbbec cameras) and direct camera access.
"""

import subprocess
import time
import signal
import threading
from typing import Optional

import cv2
import zmq
import numpy as np


class StreamClient:
    """
    Client that captures video from a camera and streams to a server.

    Supports multiple video sources:
    - RTSP streams (e.g., from Docker containers)
    - Direct camera index (V4L2 devices)
    - Video files (for testing)
    """

    def __init__(self, server_ip: str, server_port: int = 5555, camera_id: str = "cam0"):
        """
        Initialize stream client.

        Args:
            server_ip: IP address of the stream server
            server_port: Port of the stream server
            camera_id: Unique identifier for this camera
        """
        self.server_ip = server_ip
        self.server_port = server_port
        self.camera_id = camera_id
        self.running = False

        # ZMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.connect(f"tcp://{server_ip}:{server_port}")
        self.socket.setsockopt(zmq.SNDHWM, 2)  # Limit queue to reduce latency

        # Stats
        self._frame_count = 0
        self._start_time = time.time()

    def run(self, source: str = "rtsp://127.0.0.1:8554/RGBD",
            jpeg_quality: int = 80):
        """
        Main streaming loop.

        Args:
            source: Video source - RTSP URL, camera index (as string), or file path
            jpeg_quality: JPEG encoding quality (0-100)
        """
        self.running = True

        # Open video source with optimized settings for RTSP
        if source.isdigit():
            cap = cv2.VideoCapture(int(source))
        elif source.startswith("rtsp://"):
            cap = cv2.VideoCapture(source)
        else:
            cap = cv2.VideoCapture(source)

        if not cap.isOpened():
            print(f"ERROR: Could not open video source: {source}")
            return

        print(f"Streaming from {source} to {self.server_ip}:{self.server_port}")
        print(f"Camera ID: {self.camera_id}")
        print("Press Ctrl+C to stop...")

        self._frame_count = 0
        self._start_time = time.time()

        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame, reconnecting...")
                time.sleep(1)
                cap.release()
                cap = cv2.VideoCapture(source if not source.isdigit() else int(source))
                continue

            # Encode frame as JPEG
            _, encoded = cv2.imencode('.jpg', frame,
                                      [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])

            # Send frame with metadata
            try:
                self.socket.send_json({
                    'camera_id': self.camera_id,
                    'timestamp': time.time(),
                    'width': frame.shape[1],
                    'height': frame.shape[0]
                }, zmq.SNDMORE)
                self.socket.send(encoded.tobytes(), zmq.NOBLOCK)
                self._frame_count += 1
            except zmq.Again:
                pass  # Skip frame if queue is full

            # Print FPS every 5 seconds
            elapsed = time.time() - self._start_time
            if elapsed >= 5.0:
                fps = self._frame_count / elapsed
                print(f"Streaming: {fps:.1f} FPS")
                self._frame_count = 0
                self._start_time = time.time()

        cap.release()

    def run_with_docker(self, rtsp_path: str = "/RGBD",
                        docker_container: str = "determ_flowbit_client",
                        jpeg_quality: int = 80):
        """
        Run streaming with Docker container management.

        Starts the Docker container if not running, connects to its RTSP stream.

        Args:
            rtsp_path: RTSP stream path (e.g., "/RGBD")
            docker_container: Name of Docker container to start
            jpeg_quality: JPEG encoding quality
        """
        print(f"Starting Docker container: {docker_container}")
        subprocess.run(["docker", "start", docker_container],
                      capture_output=True, timeout=10)
        time.sleep(5)  # Wait for camera initialization

        rtsp_url = f"rtsp://127.0.0.1:8554{rtsp_path}"
        self.run(source=rtsp_url, jpeg_quality=jpeg_quality)

    def stop(self):
        """Stop streaming."""
        self.running = False

    def cleanup(self):
        """Clean up resources."""
        self.socket.close()
        self.context.term()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        self.cleanup()
        return False
