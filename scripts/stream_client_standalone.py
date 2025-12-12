#!/usr/bin/env python3
"""
Standalone Camera Stream Client for MultiCameraMakerTracking

This script captures video from a camera source (RTSP, webcam, or file)
and streams frames to a remote server over ZMQ for multi-camera ArUco
marker tracking.

This is a STANDALONE script - copy it to any remote machine and run it.
No other project files are required.

Features:
    - Supports RTSP streams (Orbbec cameras via Docker), webcams, and video files
    - Uses TCP transport for RTSP to avoid H.264/HEVC reference frame errors
    - Auto-reconnects on connection loss with exponential backoff
    - JPEG compression with configurable quality

Requirements (install on remote machine):
    pip3 install opencv-python-headless pyzmq numpy

Usage:
    # Basic usage with RTSP (default source)
    python3 stream_client_standalone.py -s <server_ip> -c <camera_id>

    # With full camera ID from calibration (recommended for multi-camera tracking)
    python3 stream_client_standalone.py -s 192.168.1.195 \\
        -c SZVIDS-250514-07F138-1343C5-E52215 --rtsp rtsp://127.0.0.1:8554/RGBD

    # With webcam
    python3 stream_client_standalone.py -s 192.168.1.195 -c cam1 --source 0

    # With video file (for testing)
    python3 stream_client_standalone.py -s 192.168.1.195 -c cam1 --source video.mp4

    # Start Docker container automatically
    python3 stream_client_standalone.py -s 192.168.1.195 -c cam1 --start-docker

Note:
    The camera-id should match the CameraID in the calibration file for proper
    pose fusion across multiple cameras. Partial matching is supported.
"""

import argparse
import signal
import subprocess
import sys
import time

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV not installed. Run: pip3 install opencv-python-headless")
    sys.exit(1)

try:
    import zmq
except ImportError:
    print("ERROR: PyZMQ not installed. Run: pip3 install pyzmq")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Stream video to MultiCameraMakerTracking server",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Stream from Docker RTSP (Orbbec camera)
  python3 stream_client_standalone.py -s 192.168.1.100 -c cam1

  # Stream from webcam (device 0)
  python3 stream_client_standalone.py -s 192.168.1.100 -c cam1 --source 0

  # Stream from video file
  python3 stream_client_standalone.py -s 192.168.1.100 -c cam1 --source video.mp4

  # Start Docker container automatically
  python3 stream_client_standalone.py -s 192.168.1.100 -c cam1 --start-docker
        """
    )
    parser.add_argument("--server", "-s", type=str, required=True,
                        help="Server IP address (required)")
    parser.add_argument("--port", "-p", type=int, default=5555,
                        help="Server port (default: 5555)")
    parser.add_argument("--camera-id", "-c", type=str, default="cam0",
                        help="Unique camera identifier (default: cam0)")
    parser.add_argument("--source", type=str, default=None,
                        help="Video source: camera index (0,1,...), file path, or RTSP URL. "
                             "If not specified, uses --rtsp value")
    parser.add_argument("--rtsp", type=str, default="rtsp://127.0.0.1:8554/RGBD",
                        help="RTSP URL for Docker camera (default: rtsp://127.0.0.1:8554/RGBD)")
    parser.add_argument("--quality", "-q", type=int, default=80,
                        help="JPEG quality 0-100 (default: 80)")
    parser.add_argument("--start-docker", action="store_true",
                        help="Start Docker container before streaming")
    parser.add_argument("--docker-container", type=str, default="determ_flowbit_client",
                        help="Docker container name (default: determ_flowbit_client)")
    args = parser.parse_args()

    # Determine video source
    if args.source is not None:
        source = args.source
    else:
        source = args.rtsp

    # Start Docker if requested
    if args.start_docker:
        print(f"Starting Docker container: {args.docker_container}")
        try:
            subprocess.run(["docker", "start", args.docker_container],
                          capture_output=True, timeout=10)
            print("Waiting 5 seconds for camera initialization...")
            time.sleep(5)
        except Exception as e:
            print(f"Warning: Could not start Docker container: {e}")

    # ZMQ setup
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.connect(f"tcp://{args.server}:{args.port}")
    socket.setsockopt(zmq.SNDHWM, 2)  # Limit queue to reduce latency

    # Signal handler for graceful shutdown
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        print("\nShutting down...")
        running = False
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Open video source
    print(f"Opening video source: {source}")
    if source.isdigit():
        cap = cv2.VideoCapture(int(source))
    elif source.startswith("rtsp://"):
        # Use TCP transport to avoid "could not find ref with POC" errors
        # This happens with UDP packet loss causing missing H.264/HEVC reference frames
        import os
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"
        print("Using TCP transport for RTSP (more reliable)")
        cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    else:
        cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        print(f"ERROR: Could not open video source: {source}")
        print("\nTroubleshooting:")
        print("  - For RTSP: Make sure Docker container is running")
        print("  - For webcam: Check device index (try 0, 1, 2)")
        print("  - For file: Verify file path exists")
        socket.close()
        context.term()
        sys.exit(1)

    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"\n{'='*50}")
    print(f"Stream Client Started")
    print(f"{'='*50}")
    print(f"  Camera ID:   {args.camera_id}")
    print(f"  Source:      {source}")
    print(f"  Resolution:  {width}x{height}")
    print(f"  Source FPS:  {fps:.1f}")
    print(f"  Server:      {args.server}:{args.port}")
    print(f"  Quality:     {args.quality}")
    print(f"{'='*50}")
    print("Press Ctrl+C to stop\n")

    frame_count = 0
    start_time = time.time()
    reconnect_delay = 1.0

    while running:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to read frame, reconnecting in {reconnect_delay}s...")
            time.sleep(reconnect_delay)
            cap.release()
            if source.isdigit():
                cap = cv2.VideoCapture(int(source))
            elif source.startswith("rtsp://"):
                cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            else:
                cap = cv2.VideoCapture(source)
            reconnect_delay = min(reconnect_delay * 2, 30)  # Exponential backoff
            continue

        reconnect_delay = 1.0  # Reset on successful read

        # Encode frame as JPEG
        _, encoded = cv2.imencode('.jpg', frame,
                                  [cv2.IMWRITE_JPEG_QUALITY, args.quality])

        # Send frame with metadata
        try:
            socket.send_json({
                'camera_id': args.camera_id,
                'timestamp': time.time(),
                'width': frame.shape[1],
                'height': frame.shape[0]
            }, zmq.SNDMORE)
            socket.send(encoded.tobytes(), zmq.NOBLOCK)
            frame_count += 1
        except zmq.Again:
            pass  # Skip frame if queue is full (server not keeping up)

        # Print stats every 5 seconds
        elapsed = time.time() - start_time
        if elapsed >= 5.0:
            fps_actual = frame_count / elapsed
            print(f"[{args.camera_id}] Streaming: {fps_actual:.1f} FPS, "
                  f"Frame size: {len(encoded)/1024:.1f} KB")
            frame_count = 0
            start_time = time.time()

    # Cleanup
    cap.release()
    socket.close()
    context.term()
    print("Stream client stopped.")


if __name__ == "__main__":
    main()
