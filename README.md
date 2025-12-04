# Multi-Camera Marker Tracking

A real-time ArUco marker tracking and visualization system for multi-camera setups. Designed for robot navigation with ROS-compatible architecture.

![Marker Tracking Visualization](docs/screenshots/marker_tracking.png)

*3D visualization showing marker spheres with orientation axes, camera frustums, coordinate frame, and ground grid*

## Features

- **Real-time marker tracking** with Open3D visualization
- **Marker orientation visualization** with XYZ axes (red=X, green=Y, blue=Z)
- **Simulated tracker** for testing without camera hardware
- **Multiple motion patterns**: circular, linear, figure-8, random walk
- **Pub/sub architecture** ready for ROS migration
- Camera pose visualization from sensor configuration files
- Marker trajectory trails with configurable length
- 30Hz tracking rate for robot navigation

## Project Structure

```
MultiCameraMakerTracking/
├── src/
│   ├── core/           # Message bus, data types, config loader
│   ├── tracking/       # Tracker implementations (simulated, future ArUco)
│   └── visualization/  # Open3D viewer and scene management
├── scripts/
│   └── run_full_system.py  # Main entry point
├── config/             # Camera configurations
├── main.py             # Legacy Plotly visualization
└── requirements.txt
```

## Requirements

- Python 3.x
- numpy
- open3d
- matplotlib (optional, for legacy visualization)
- plotly (optional, for legacy visualization)

## Installation

```bash
pip install -r requirements.txt
```

Or manually:
```bash
pip install numpy open3d
```

## Usage

### Real-time Marker Tracking (Open3D)

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
