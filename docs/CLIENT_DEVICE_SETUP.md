# Client Device Setup Guide

This guide explains how to set up camera client devices for the Multi-Camera Marker Tracking system.

## Prerequisites

Each camera client device requires:
- Ubuntu 20.04 (Focal Fossa) - ARM64 or x86_64
- Orbbec Femto Bolt 3D Camera (connected via USB)
- Docker (for running the camera driver)
- Python 3.8+ with required packages
- Network connectivity to the tracking server

## Architecture

```
Camera Device                          Tracking Server
┌─────────────────────┐                ┌──────────────────┐
│ Orbbec Camera       │                │                  │
│        │            │                │  StreamServer    │
│        v            │    ZMQ/TCP     │  (port 5555)     │
│ Docker Container    │ ───────────>   │        │         │
│ (flowbit_client)    │                │        v         │
│   RTSP :8554        │                │ MultiCameraTracker│
│        │            │                │        │         │
│        v            │                │        v         │
│ stream_client.py    │                │  3D Visualization │
└─────────────────────┘                └──────────────────┘
```

## Step 1: Install Docker

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Logout and login again for group changes to take effect
```

## Step 2: Set Up the Orbbec Camera Docker Image

The camera driver runs inside a Docker container that provides an RTSP stream.

```bash
# Pull the flowbit client image (if available from registry)
docker pull determtech/determ_flowbit_client:v1.4.x

# Or load from a tar file if provided
docker load < determ_flowbit_client_v1.4.x.tar
```

## Step 3: Install Python Dependencies

```bash
# Install required system packages
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv python3-zmq python3-numpy

# Or via pip (if apt packages not available)
pip3 install opencv-python-headless pyzmq numpy
```

## Step 4: Deploy the Stream Client Script

Copy the `stream_client.py` script to the camera device:

```bash
# On the tracking server, copy to client:
scp src/streaming/stream_client.py determtech@<camera_ip>:/home/determtech/

# Make it executable
ssh determtech@<camera_ip> "chmod +x /home/determtech/stream_client.py"
```

**stream_client.py** reads from the local RTSP stream and sends frames to the tracking server via ZMQ.

## Step 5: Start the Docker Container

```bash
# Start the camera container
docker run -d \
    --name determ_flowbit_client \
    --privileged \
    --restart unless-stopped \
    -v /dev:/dev \
    determtech/determ_flowbit_client:v1.4.x

# Verify it's running
docker ps
```

The container provides an RTSP stream at `rtsp://127.0.0.1:8554/RGBD`.

## Step 6: Start the Stream Client

```bash
# Start streaming to the server
python3 /home/determtech/stream_client.py \
    --server <SERVER_IP> \
    --port 5555 \
    --camera-id <CAMERA_ID> \
    --rtsp rtsp://127.0.0.1:8554/RGBD
```

### Parameters:
- `--server`: IP address of the tracking server
- `--port`: Server port (default: 5555)
- `--camera-id`: Unique camera identifier (use the full ID from calibration file)
- `--rtsp`: Local RTSP URL (default: rtsp://127.0.0.1:8554/RGBD)

### Example:
```bash
python3 /home/determtech/stream_client.py \
    --server 192.168.1.195 \
    --port 5555 \
    --camera-id SZVIDS-250514-07F138-1343C5-E52215 \
    --rtsp rtsp://127.0.0.1:8554/RGBD
```

## Step 7: Run as a Service (Optional)

To run the stream client automatically at boot:

```bash
# Create systemd service file
sudo tee /etc/systemd/system/stream-client.service << EOF
[Unit]
Description=Camera Stream Client
After=docker.service
Requires=docker.service

[Service]
Type=simple
User=determtech
ExecStartPre=/usr/bin/docker start determ_flowbit_client
ExecStartPre=/bin/sleep 10
ExecStart=/usr/bin/python3 /home/determtech/stream_client.py --server 192.168.1.195 --port 5555 --camera-id CAMERA_ID_HERE --rtsp rtsp://127.0.0.1:8554/RGBD
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Enable and start
sudo systemctl enable stream-client
sudo systemctl start stream-client
```

## Troubleshooting

### Camera not detected
```bash
# Check USB devices
lsusb | grep Orbbec

# Check Docker container logs
docker logs determ_flowbit_client
```

### RTSP stream not working
```bash
# Check if port 8554 is listening
ss -tuln | grep 8554

# Test RTSP with OpenCV
python3 -c "import cv2; cap=cv2.VideoCapture('rtsp://127.0.0.1:8554/RGBD'); print('opened:', cap.isOpened()); cap.release()"
```

### Stream client not connecting
```bash
# Check network connectivity to server
ping <SERVER_IP>

# Check if stream_client is running
ps aux | grep stream_client

# Check logs
cat /tmp/stream_client.log
```

### Docker container crashing
```bash
# Restart container
docker restart determ_flowbit_client

# Check container logs
docker logs --tail 50 determ_flowbit_client
```

## Camera IPs and IDs

| Camera | IP Address | Camera ID |
|--------|------------|-----------|
| 250514 | 192.168.1.80 | SZVIDS-250514-07F138-1343C5-E52215 |
| 250515 | 192.168.1.85 | SZVIDS-250515-EAA839-91E21B-292366 |
| 250513 | 192.168.1.91 | SZVIDS-250513-5001E3-E1F1BA-799662 |

## Quick Setup Script

For quick deployment on a new camera device:

```bash
#!/bin/bash
# quick_setup.sh

SERVER_IP="192.168.1.195"
CAMERA_ID="YOUR_CAMERA_ID"

# Install dependencies
sudo apt-get update
sudo apt-get install -y python3-opencv python3-zmq python3-numpy

# Start Docker container
docker start determ_flowbit_client || echo "Container may need to be created first"
sleep 10

# Start streaming
python3 /home/determtech/stream_client.py \
    --server $SERVER_IP \
    --port 5555 \
    --camera-id $CAMERA_ID \
    --rtsp rtsp://127.0.0.1:8554/RGBD
```
