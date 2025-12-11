# Network Camera Streaming & ArUco Marker Tracking

This document describes how to set up and run the network-based video streaming system for ArUco marker tracking with remote Orbbec cameras.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Remote Machine (192.168.1.80)                   │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │
│  │ Docker Container│    │   RTSP Stream   │    │  Stream Client  │  │
│  │ (Orbbec Camera) │───>│ rtsp://.../RGBD │───>│ stream_client.py│  │
│  │  Port 8554      │    │   720p @ 30fps  │    │   ZMQ PUSH      │  │
│  └─────────────────┘    └─────────────────┘    └────────┬────────┘  │
└─────────────────────────────────────────────────────────┼───────────┘
                                                          │
                                                    TCP Port 5555
                                                    (JPEG frames)
                                                          │
┌─────────────────────────────────────────────────────────┼───────────┐
│                     Local Machine (Server)              │           │
│  ┌─────────────────┐    ┌─────────────────┐    ┌───────▼─────────┐  │
│  │  Preview Window │<───│  ArUco Tracker  │<───│  Stream Server  │  │
│  │   (OpenCV GUI)  │    │  (Detection)    │    │   ZMQ PULL      │  │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘  │
│                                                                     │
│                    MultiCameraMakerTracking/                        │
└─────────────────────────────────────────────────────────────────────┘
```

## Components

### 1. Stream Client (Remote Machine)
- **Source**: `scripts/stream_client_standalone.py` (copy to remote machine)
- **Purpose**: Captures video from Docker's RTSP stream and sends frames via ZMQ
- **Protocol**: ZMQ PUSH socket with JPEG-encoded frames
- **Standalone**: No project dependencies - just copy and run

### 2. Stream Server (Local Machine)
- **Location**: `src/streaming/stream_server.py`
- **Purpose**: Receives frames from multiple remote cameras
- **Protocol**: ZMQ PULL socket

### 3. Network Tracker (Local Machine)
- **Location**: `scripts/run_network_tracker.py`
- **Purpose**: Detects ArUco markers in received frames with live preview

### 4. Stream Client Class (For Integration)
- **Location**: `src/streaming/stream_client.py`
- **Purpose**: Python class for programmatic use in other scripts

---

## Quick Start

### On the Local Machine (First Time Setup)

```bash
cd /home/determ/MultiCameraMakerTracking

# Copy the standalone client to the remote machine
scp scripts/stream_client_standalone.py determtech@192.168.1.80:~/stream_client.py
# Password: determtech
```

### On the Remote Machine (192.168.1.80)

```bash
# SSH into the remote machine
ssh determtech@192.168.1.80
# Password: determtech

# Start the Docker container (if not already running)
docker start determ_flowbit_client

# Wait 5 seconds for camera initialization
sleep 5

# Run the stream client
python3 stream_client.py --server <YOUR_LOCAL_IP> --port 5555 --camera-id cam1
```

### On the Local Machine

```bash
cd /home/determ/MultiCameraMakerTracking

# Run the tracker with preview
DISPLAY=:1 python3 scripts/run_network_tracker.py --preview
```

---

## Detailed Instructions

### Remote Machine Setup (One-time)

1. **Copy the standalone client to the remote machine** (from local machine):
   ```bash
   cd /home/determ/MultiCameraMakerTracking
   scp scripts/stream_client_standalone.py determtech@192.168.1.80:~/stream_client.py
   # Password: determtech
   ```

2. **SSH into the remote machine and install dependencies**:
   ```bash
   ssh determtech@192.168.1.80
   # Password: determtech

   # Install required packages
   pip3 install opencv-python-headless pyzmq numpy
   ```

3. **Verify the script works**:
   ```bash
   python3 stream_client.py --help
   ```

---

## Running the System

### Step 1: Start Docker Container (Remote Machine)

```bash
# On remote machine (192.168.1.80)
docker start determ_flowbit_client
sleep 5  # Wait for camera initialization
```

### Step 2: Start Stream Client (Remote Machine)

```bash
# On remote machine
python3 ~/stream_client.py \
    --server <LOCAL_MACHINE_IP> \
    --port 5555 \
    --camera-id cam1

# Or with auto Docker start
python3 ~/stream_client.py \
    --server <LOCAL_MACHINE_IP> \
    --camera-id cam1 \
    --start-docker
```

**Arguments**:
| Argument | Description | Default |
|----------|-------------|---------|
| `--server`, `-s` | Server IP address | Required |
| `--port`, `-p` | Server port | 5555 |
| `--camera-id`, `-c` | Unique camera identifier | cam0 |
| `--source` | Video source (camera index, file, or URL) | Uses --rtsp |
| `--rtsp` | RTSP stream URL | rtsp://127.0.0.1:8554/RGBD |
| `--quality`, `-q` | JPEG quality (0-100) | 80 |
| `--start-docker` | Start Docker container before streaming | Off |
| `--docker-container` | Docker container name | determ_flowbit_client |

### Step 3: Start Tracker (Local Machine)

```bash
# On local machine
cd /home/determ/MultiCameraMakerTracking
DISPLAY=:1 python3 scripts/run_network_tracker.py --preview
```

**Arguments**:
| Argument | Description | Default |
|----------|-------------|---------|
| `--port`, `-p` | Listen port | 5555 |
| `--preview` | Show visualization window | Off |
| `--dict` | ArUco dictionary type | DICT_4X4_50 |
| `--marker-size` | Marker size in meters | 0.05 |

**Supported ArUco dictionaries**:
- `DICT_4X4_50`
- `DICT_4X4_100`
- `DICT_5X5_50`
- `DICT_6X6_50`

---

## Stopping the System

### Stop Tracker (Local Machine)

```bash
# Option 1: Press 'q' in the preview window

# Option 2: Press Ctrl+C in the terminal

# Option 3: Kill by process name
pkill -f run_network_tracker
```

### Stop Stream Client (Remote Machine)

```bash
# Option 1: Press Ctrl+C in the SSH terminal

# Option 2: Kill by process name (run on remote machine)
pkill -f stream_client.py
```

### Stop Docker Container (Remote Machine)

```bash
# On remote machine
docker stop determ_flowbit_client
```

---

## Restarting the System

### Full Restart

```bash
# === On Local Machine ===
# Stop tracker
pkill -f run_network_tracker

# === On Remote Machine (192.168.1.80) ===
# Stop client
pkill -f stream_client.py

# Restart Docker
docker restart determ_flowbit_client
sleep 5

# Start client
python3 /home/determtech/stream_client.py --server <LOCAL_IP> --port 5555 --camera-id cam1

# === On Local Machine ===
# Start tracker
DISPLAY=:1 python3 scripts/run_network_tracker.py --preview
```

### Quick Restart (Tracker Only)

```bash
# On local machine
pkill -f run_network_tracker
sleep 2
DISPLAY=:1 python3 scripts/run_network_tracker.py --preview
```

---

## Running in Background

### Background Tracker (Local Machine)

```bash
# Start in background with logging
DISPLAY=:1 nohup python3 scripts/run_network_tracker.py --preview > tracker.log 2>&1 &

# View logs
tail -f tracker.log

# Stop
pkill -f run_network_tracker
```

### Background Client (Remote Machine)

```bash
# Start in background
nohup python3 /home/determtech/stream_client.py --server <IP> --port 5555 --camera-id cam1 > stream.log 2>&1 &

# View logs
tail -f stream.log

# Stop
pkill -f stream_client.py
```

---

## Troubleshooting

### No Frames Received

1. **Check network connectivity**:
   ```bash
   ping 192.168.1.80
   ```

2. **Verify stream client is running** (on remote):
   ```bash
   ps aux | grep stream_client
   ```

3. **Check if Docker is running** (on remote):
   ```bash
   docker ps | grep determ_flowbit
   ```

4. **Test RTSP stream** (on remote):
   ```bash
   ffprobe rtsp://127.0.0.1:8554/RGBD
   ```

### Port Already in Use

```bash
# Find process using port 5555
lsof -i :5555

# Kill the process
kill <PID>

# Or wait for ZMQ socket timeout (usually ~30 seconds)
```

### Preview Window Not Showing

1. **Check DISPLAY variable**:
   ```bash
   echo $DISPLAY
   # Should be :0 or :1
   ```

2. **Run with explicit display**:
   ```bash
   DISPLAY=:1 python3 scripts/run_network_tracker.py --preview
   ```

### High CPU Usage

This is normal during video processing. The tracker processes ~60 frames/second. To reduce CPU:
- Lower JPEG quality on client: `--quality 60`
- Reduce frame rate by modifying stream client

### Markers Not Detected

1. **Verify ArUco dictionary matches your markers**:
   ```bash
   python3 scripts/run_network_tracker.py --dict DICT_4X4_50 --preview
   ```

2. **Check lighting conditions** - markers need good contrast

3. **Verify marker size** (for accurate pose estimation):
   ```bash
   python3 scripts/run_network_tracker.py --marker-size 0.05 --preview
   ```

---

## Multiple Cameras

To stream from multiple cameras, run multiple clients with different IDs:

### Remote Machine 1 (192.168.1.80)
```bash
python3 stream_client.py --server <LOCAL_IP> --camera-id cam1
```

### Remote Machine 2 (192.168.1.81)
```bash
python3 stream_client.py --server <LOCAL_IP> --camera-id cam2
```

The tracker will automatically create separate preview windows for each camera.

---

## Output Format

The tracker logs detection information:

```
Starting stream server on port 5555...
Stream server listening on port 5555...
Waiting for camera connections...
Press 'q' to quit

Cameras: ['cam1'], FPS: 65.9, Markers detected: 86
[cam1] Detected markers: [0, 1, 5]
Cameras: ['cam1'], FPS: 58.1, Markers detected: 89
```

- **FPS**: Processing frame rate
- **Markers detected**: Total marker detections in last 2-second window
- **Detected markers**: List of marker IDs when they enter/exit frame

---

## File Locations

### Local Machine (MultiCameraMakerTracking/)
```
├── scripts/
│   ├── run_network_tracker.py        # Main tracker script
│   └── stream_client_standalone.py   # Standalone client (copy to remote)
├── src/
│   ├── streaming/
│   │   ├── __init__.py
│   │   ├── stream_server.py          # ZMQ frame receiver
│   │   └── stream_client.py          # Client class for integration
│   └── tracking/
│       ├── __init__.py
│       └── network_stream_tracker.py # Tracker class for integration
└── docs/
    └── NETWORK_STREAMING.md          # This documentation
```

### Remote Machine (192.168.1.80)
```
/home/determtech/
└── stream_client.py              # Standalone streaming client
```

---

## Dependencies

### Local Machine
```
numpy>=1.21.0
opencv-contrib-python>=4.8.0
pyzmq>=25.0.0
```

### Remote Machine
```
numpy
opencv-python-headless
pyzmq
```

Install with:
```bash
pip3 install numpy opencv-python-headless pyzmq
```
