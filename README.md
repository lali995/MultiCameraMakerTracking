# Multi-Camera Marker Tracking

A real-time ArUco marker tracking and visualization system for multi-camera setups. Designed for robot navigation with ROS-compatible architecture.

![Marker Tracking Visualization](docs/screenshots/marker_tracking.png)

*3D visualization showing marker spheres with orientation axes, camera frustums, coordinate frame, and ground grid*

## Features

- **Real-time marker tracking** with Open3D visualization
- **ArUco marker generator** for creating printable markers
- **Orbbec camera integration** for RGB-D marker tracking
- **Network camera streaming** for remote Orbbec cameras via ZMQ
- **Marker orientation visualization** with XYZ axes (red=X, green=Y, blue=Z)
- **Simulated tracker** for testing without camera hardware
- **Multiple motion patterns**: circular, linear, figure-8, random walk
- **Pub/sub architecture** ready for ROS migration
- Camera pose visualization from sensor configuration files
- Marker trajectory trails with configurable length
- 60+ FPS network streaming with JPEG compression
- 30Hz tracking rate for robot navigation

## Project Structure

```
MultiCameraMakerTracking/
├── src/
│   ├── core/           # Message bus, data types, config loader
│   ├── markers/        # ArUco marker generation
│   ├── tracking/       # Tracker implementations (simulated, Orbbec, network)
│   ├── streaming/      # Network video streaming (ZMQ-based)
│   └── visualization/  # Open3D viewer and scene management
├── scripts/
│   ├── run_full_system.py           # Simulated tracking demo
│   ├── run_orbbec_tracker.py        # Orbbec camera tracking
│   ├── run_network_tracker.py       # Network camera tracking
│   ├── stream_client_standalone.py  # Client to copy to remote machines
│   └── generate_markers.py          # Generate printable markers
├── docs/
│   └── NETWORK_STREAMING.md    # Network streaming documentation
├── config/             # Camera configurations
├── markers/            # Generated marker images (output)
├── main.py             # Legacy Plotly visualization
└── requirements.txt
```

## Requirements

- Python 3.x
- numpy
- open3d
- opencv-contrib-python (for ArUco detection)
- pyzmq (for network streaming)
- matplotlib (optional, for legacy visualization)
- plotly (optional, for legacy visualization)
- pyorbbecsdk (optional, for Orbbec cameras)

## Installation

```bash
pip install -r requirements.txt
```

For Orbbec camera support:
```bash
pip install pyorbbecsdk
```

## Usage

### 1. Generate Printable Markers

First, generate ArUco markers to print:

```bash
# Generate markers 0-3 as individual files
python scripts/generate_markers.py --id 0 1 2 3

# Generate a sheet with markers for easy printing
python scripts/generate_markers.py --sheet --id 0 1 2 3 4 5 6 7

# Generate 50mm markers at 300 DPI for accurate physical size
python scripts/generate_markers.py --size-mm 50 --dpi 300 --id 0 1 2

# List available ArUco dictionaries
python scripts/generate_markers.py --list-dicts
```

Output files are saved to `markers/` directory by default.

### 2. Track with Orbbec Camera

Run real marker tracking with an Orbbec RGB-D camera:

```bash
# Basic tracking with 3D visualization
python scripts/run_orbbec_tracker.py

# Track 50mm markers (must match printed marker size!)
python scripts/run_orbbec_tracker.py --marker-size 0.05

# Show camera preview with detections
python scripts/run_orbbec_tracker.py --preview

# Use specific ArUco dictionary (must match generated markers!)
python scripts/run_orbbec_tracker.py --dict DICT_4X4_50
```

Options:
```
--marker-size M     Physical marker size in meters (default: 0.05 = 50mm)
--dict TYPE         ArUco dictionary type (default: DICT_4X4_50)
--preview           Show camera preview window with detections
--no-visualization  Disable 3D Open3D visualization
--camera-config     Path to camera config with extrinsic matrix
```

### 3. Network Camera Streaming (Remote Orbbec Cameras)

Track markers from cameras on remote machines over the network:

```bash
# First time: copy the standalone client to remote machine
scp scripts/stream_client_standalone.py determtech@192.168.1.80:~/stream_client.py

# On remote machine (192.168.1.80) - start the stream client
python3 stream_client.py --server <YOUR_LOCAL_IP> --camera-id cam1 --start-docker

# On local machine - start the tracker with preview
DISPLAY=:1 python3 scripts/run_network_tracker.py --preview
```

**Quick Commands**:
```bash
# Stop tracker
pkill -f run_network_tracker

# Restart tracker
pkill -f run_network_tracker && sleep 2 && DISPLAY=:1 python3 scripts/run_network_tracker.py --preview
```

For full setup instructions, see [docs/NETWORK_STREAMING.md](docs/NETWORK_STREAMING.md).

### 4. Simulated Tracking (No Camera Required)

Test the system without hardware:

```bash
python scripts/run_full_system.py
```

Options:
```
--config PATH       Path to config directory (default: config)
--num-markers N     Number of simulated markers (default: 3)
--motion TYPE       Motion type: circular, linear, figure8, random (default: circular)
--no-trajectory     Disable marker trails
--speed SPEED       Motion speed multiplier (default: 1.0)
--rate HZ           Tracker update rate (default: 30.0)
```

Examples:
```bash
# 5 markers moving in figure-8 pattern
python scripts/run_full_system.py --num-markers 5 --motion figure8

# Fast random motion without trails
python scripts/run_full_system.py --motion random --speed 2.0 --no-trajectory
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| Q / ESC | Quit |
| P | Pause/Resume |
| R | Reset camera view |
| C | Clear marker trails |
| T | Toggle trail visibility |

### Legacy Visualization (Plotly)

```bash
python main.py
```

Opens an interactive 3D view in your browser.

## Configuration

Place camera configuration folders in the `config/` directory:

```
config/
├── SZVIDS-250515-DB77B5-8F194B-9AF224/
│   └── sensor_config.json
├── SZVIDS-250513-21892C-CAEB69-EC3063/
│   └── sensor_config.json
└── ...
```

Each `sensor_config.json` must contain a `Sensor to World Extrinsic` 4x4 transformation matrix.

## Architecture

The system uses a pub/sub message bus pattern designed for ROS compatibility:

```
SimulatedTracker (30Hz) --> MessageBus --> MarkerViewer (60Hz render)
        |                  /markers/poses         |
   Publishes MarkerPose    <--    Subscribes & updates visualization
   (position + orientation)       (spheres + XYZ axes + trails)
```

### Marker Visualization

Each marker is displayed with:
- **Colored sphere**: Position indicator (colors cycle through red, green, blue, yellow, magenta, cyan, orange, purple)
- **XYZ axes**: Orientation indicator showing marker heading
  - Red axis = X (right)
  - Green axis = Y (up)
  - Blue axis = Z (forward)
- **Trajectory trail**: Movement history (optional, toggle with 'T' key)

### Future ROS Migration

The `MessageBus` class can be swapped with a ROS-based implementation:
- Topics map to ROS topics
- `MarkerPose` maps to `geometry_msgs/PoseStamped`
- `spin_once()` maps to `rospy.spin()`

## Coordinate System

- **X-axis**: Left to right
- **Y-axis**: Up
- **Z-axis**: Front to back
