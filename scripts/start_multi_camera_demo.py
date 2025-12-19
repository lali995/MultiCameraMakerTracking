#!/usr/bin/env python3
"""
Start multi-camera tracking demo.

1. Reads camera IPs from calibration file
2. SSHs into each camera machine and starts stream_client.py
3. Runs the multi-camera tracker with 3D visualization

Usage:
    python scripts/start_multi_camera_demo.py
"""

import os
import sys
import json
import time
import subprocess
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import paramiko

# SSH credentials (same for all cameras)
SSH_USER = "determtech"
SSH_PASSWORD = "determtech"

# Server settings
SERVER_IP = None  # Will be auto-detected
SERVER_PORT = 5555


def get_local_ip():
    """Get local IP address."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('192.168.1.1', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


def load_camera_configs(calibration_path):
    """Load camera configs including IPs from calibration file."""
    with open(calibration_path, 'r') as f:
        data = json.load(f)

    cameras = []
    for cam in data.get('CalibratedCameras', []):
        cameras.append({
            'camera_id': cam.get('CameraID'),
            'ip': cam.get('IP'),
            'index': cam.get('CameraIndex')
        })
    return cameras


def start_stream_client_on_remote(camera_ip, camera_id, server_ip, server_port):
    """SSH into remote and start stream_client.py."""
    print(f"  Connecting to {camera_ip} for {camera_id[-12:]}...")

    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(camera_ip, username=SSH_USER, password=SSH_PASSWORD, timeout=10)

        # Kill any existing stream_client
        ssh.exec_command('pkill -f stream_client')
        time.sleep(0.5)

        # Start stream_client in background
        cmd = f'nohup python3 /home/{SSH_USER}/stream_client.py --server {server_ip} --port {server_port} --camera-id {camera_id} > /tmp/stream_client.log 2>&1 &'
        stdin, stdout, stderr = ssh.exec_command(cmd)
        time.sleep(2)

        # Verify it started
        stdin, stdout, stderr = ssh.exec_command('ps aux | grep stream_client | grep -v grep')
        result = stdout.read().decode()

        ssh.close()

        if 'stream_client' in result:
            print(f"  ✓ Started stream client on {camera_ip}")
            return True
        else:
            print(f"  ✗ Failed to start on {camera_ip}")
            return False

    except Exception as e:
        print(f"  ✗ Error connecting to {camera_ip}: {e}")
        return False


def stop_stream_client_on_remote(camera_ip):
    """SSH into remote and stop stream_client.py."""
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(camera_ip, username=SSH_USER, password=SSH_PASSWORD, timeout=10)
        ssh.exec_command('pkill -f stream_client')
        ssh.close()
        print(f"  Stopped stream client on {camera_ip}")
    except Exception as e:
        print(f"  Error stopping on {camera_ip}: {e}")


def main():
    global SERVER_IP

    calibration_path = "calibration/calibration_result.json"

    print("=" * 60)
    print("Multi-Camera Tracking Demo Launcher")
    print("=" * 60)

    # Get local IP
    SERVER_IP = get_local_ip()
    print(f"Server IP: {SERVER_IP}")
    print(f"Server Port: {SERVER_PORT}")

    # Load camera configs
    print(f"\nLoading cameras from {calibration_path}...")
    cameras = load_camera_configs(calibration_path)

    if not cameras:
        print("ERROR: No cameras found in calibration file")
        sys.exit(1)

    print(f"Found {len(cameras)} cameras:")
    for cam in cameras:
        print(f"  - {cam['camera_id'][-20:]} @ {cam['ip']}")

    # Start stream clients on all cameras
    print("\nStarting stream clients on remote cameras...")
    started = []
    for cam in cameras:
        if start_stream_client_on_remote(cam['ip'], cam['camera_id'], SERVER_IP, SERVER_PORT):
            started.append(cam)

    if not started:
        print("\nERROR: No stream clients started!")
        sys.exit(1)

    print(f"\n{len(started)}/{len(cameras)} stream clients started")

    # Give clients time to connect
    print("\nWaiting for connections...")
    time.sleep(3)

    # Run the multi-camera tracker
    print("\nStarting multi-camera tracker...")
    print("=" * 60)

    try:
        # Run tracker
        cmd = [
            'python3', 'scripts/run_multi_camera_tracker.py',
            '--calibration', calibration_path,
            '--reference', '250514',
            '--fusion', 'weighted_average'
        ]

        # Check if DISPLAY is set
        env = os.environ.copy()
        if 'DISPLAY' not in env:
            env['DISPLAY'] = ':1'

        subprocess.run(cmd, env=env)

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        # Stop all stream clients
        print("\nStopping stream clients...")
        for cam in cameras:
            stop_stream_client_on_remote(cam['ip'])

        print("Done.")


if __name__ == "__main__":
    main()
