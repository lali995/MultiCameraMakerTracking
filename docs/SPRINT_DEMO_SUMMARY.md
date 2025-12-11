# ArUco Marker Tracking System - Sprint Demo Summary

## Overview

This document summarizes the implementation of two ArUco marker tracking demos:
1. **Real-time 3D Visualization System** - Live/simulated tracking with Open3D visualization
2. **Video-based Tracking System** - Process recorded video with 3D visualization

---

## Demo 1: Real-time 3D Visualization System (No Video)

**Script:** `scripts/run_full_system.py`

### Implementation Steps

#### 1. Core Architecture Setup
- Created a **pub/sub message bus** (`src/core/message_bus.py`) for thread-safe communication between components
- Designed ROS-compatible topic patterns (`/markers/poses`, `/tracker/status`) for future migration path
- Implemented singleton pattern for global message bus access

#### 2. Data Types Definition
- Defined core data structures (`src/core/data_types.py`):
  - `MarkerPose`: 3D position (x, y, z), orientation (quaternion), timestamp, confidence
  - `CameraPose`: Camera position, direction, extrinsic matrix
  - `TrackerStatus`: FPS, active marker count, tracker state

#### 3. Configuration Loader
- Built `ConfigLoader` (`src/core/config_loader.py`) to parse camera configurations
- Reads `sensor_config.json` files from `config/SZVIDS-*` folders
- Extracts camera intrinsics, extrinsics (sensor-to-world transform), and distortion parameters

#### 4. Tracker Base Class
- Created abstract `TrackerBase` (`src/tracking/tracker_base.py`) with:
  - Rate-controlled tracking loop
  - Start/stop lifecycle management
  - Message bus integration for pose publishing

#### 5. Simulated Tracker
- Implemented `SimulatedTracker` (`src/tracking/simulated_tracker.py`) for demo/testing
- Generates realistic marker movement patterns (circular, random walk, oscillation)
- Configurable marker count, update rate, and movement parameters

#### 6. 3D Visualization with Open3D
- **Scene Manager** (`src/visualization/scene_manager.py`):
  - Manages marker spheres, camera frustums, coordinate axes
  - Trajectory trail rendering with color gradients
  - Dynamic geometry updates

- **Geometry Factory** (`src/visualization/geometry_factory.py`):
  - Creates camera frustum meshes
  - Generates marker spheres with configurable colors
  - Builds coordinate axis visualizations

- **Marker Viewer** (`src/visualization/viewer.py`):
  - Real-time Open3D visualization window
  - Keyboard controls: Q=quit, P=pause, R=reset view, T=toggle trails
  - Subscribes to marker poses via message bus
  - 60 FPS render target with rate limiting

### How to Run
```bash
python scripts/run_full_system.py
```

### Key Features
- Real-time 3D visualization of camera positions and marker tracking
- Trajectory trails showing marker movement history
- Interactive camera controls (rotate, zoom, pan)
- Modular architecture for easy extension

---

## Demo 2: Video-based Tracking System

**Script:** `scripts/run_video_with_visualization.py`

### Implementation Steps

#### 1. Video ArUco Detection
- Integrated OpenCV ArUco detection (`cv2.aruco`)
- Support for multiple dictionary types (DICT_4X4_50, DICT_5X5_50, DICT_6X6_50, etc.)
- Compatible with both OpenCV 4.5.x and 4.7+ APIs

#### 2. Pose Estimation Pipeline
- **Marker Detection**: Grayscale conversion → ArUco detection → Corner extraction
- **Pose Estimation**: `cv2.solvePnP()` with known marker size and camera intrinsics
- **Coordinate Transform**: Camera frame → World frame using extrinsic matrix

#### 3. Camera-to-World Transformation
- Loads camera extrinsic from `sensor_config.json`
- Applies 4x4 homogeneous transformation to marker positions
- Transforms orientation quaternion to world frame

#### 4. Video Processing Thread
- Background thread for video frame processing
- Configurable playback speed and loop mode
- Rate-limited to match video FPS

#### 5. Dual Window Display
- **Video Preview Window**: OpenCV window showing:
  - Detected marker boundaries (green polygons)
  - Marker IDs and world coordinates
  - Coordinate axes overlay on markers
  - Frame counter and detection count

- **3D Visualization Window**: Open3D viewer showing:
  - All configured camera positions
  - Real-time marker position in world coordinates
  - Movement trajectory trails

#### 6. Integration with Existing System
- Reuses message bus for pose publishing
- Reuses MarkerViewer for 3D visualization
- Reuses ConfigLoader for camera configurations

### How to Run
```bash
# With camera configuration
python scripts/run_video_with_visualization.py test_video/test_video.mp4 \
    --camera-config config/SZVIDS-250515-DB77B5-8F194B-9AF224

# Additional options
python scripts/run_video_with_visualization.py test_video/test_video2.mp4 \
    --camera-config config/SZVIDS-250515-DB77B5-8F194B-9AF224 \
    --marker-size 0.05 \
    --speed 1.0
```

### Key Features
- Process pre-recorded video files
- Real-time marker detection and pose estimation
- World coordinate transformation using camera calibration
- Synchronized video preview and 3D visualization
- Support for looping playback

---

## Test Results

### Test Video 1: `test_video.mp4`
- Resolution: 1792x1008 @ 29.38 FPS
- Frames: 347
- Marker 0 detected in 74.4% of frames
- World position range: X(0.50-0.54m), Y(2.24-2.28m), Z(-0.43 to -0.39m)

### Test Video 2: `test_video2.mp4`
- Resolution: 2160x3840 @ 29.93 FPS
- Frames: 645
- Marker 0 shows significant movement (person walking with marker)
- Y-axis range: 1.9m to 2.3m (height variation during movement)

---

## Architecture Diagram

```
┌─────────────────┐     ┌─────────────────┐
│  Video File     │     │  Orbbec Camera  │
│  (test_video)   │     │  (Live Stream)  │
└────────┬────────┘     └────────┬────────┘
         │                       │
         ▼                       ▼
┌─────────────────────────────────────────┐
│         ArUco Detection Module          │
│  - Marker detection (OpenCV)            │
│  - Pose estimation (solvePnP)           │
│  - Camera→World transform               │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│           Message Bus                   │
│  - Topic: /markers/poses                │
│  - Thread-safe pub/sub                  │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│        3D Visualization (Open3D)        │
│  - Camera frustums                      │
│  - Marker spheres                       │
│  - Trajectory trails                    │
└─────────────────────────────────────────┘
```

---

## Files Created/Modified

| File | Description |
|------|-------------|
| `src/core/message_bus.py` | Thread-safe pub/sub message bus |
| `src/core/data_types.py` | MarkerPose, CameraPose data classes |
| `src/core/config_loader.py` | Camera configuration loader |
| `src/tracking/tracker_base.py` | Abstract tracker base class |
| `src/tracking/simulated_tracker.py` | Simulated marker movement |
| `src/tracking/orbbec_tracker.py` | Orbbec camera ArUco tracker |
| `src/visualization/viewer.py` | Open3D real-time viewer |
| `src/visualization/scene_manager.py` | 3D scene geometry management |
| `src/visualization/geometry_factory.py` | Geometry creation utilities |
| `scripts/run_full_system.py` | Demo 1: Simulated tracking |
| `scripts/run_video_with_visualization.py` | Demo 2: Video-based tracking |
| `scripts/run_video_tracker.py` | Video tracking (no 3D viz) |

---

## Next Steps

1. Multi-camera triangulation for improved 3D accuracy
2. Marker ID assignment and tracking persistence
3. Recording and playback of tracking sessions
4. Integration with ROS2 for robot control applications
