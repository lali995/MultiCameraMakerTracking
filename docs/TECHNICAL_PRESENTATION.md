# Multi-Camera ArUco Marker Tracking System

## Technical Presentation Document

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Architecture](#2-architecture)
3. [Client Setup - Video Streaming](#3-client-setup---video-streaming)
4. [Server Setup - Marker Detection](#4-server-setup---marker-detection)
5. [Camera Calibration](#5-camera-calibration)
6. [ArUco Marker Detection](#6-aruco-marker-detection)
7. [Coordinate Transformation](#7-coordinate-transformation)
8. [Multi-Camera Pose Fusion](#8-multi-camera-pose-fusion)
9. [Distance Calculation](#9-distance-calculation)
10. [3D Visualization](#10-3d-visualization)
11. [Running the System](#11-running-the-system)
12. [Demo Video](#12-demo-video)
13. [End-to-End Data Flow](#13-end-to-end-data-flow)

---

## 1. System Overview

This system tracks ArUco markers in 3D space using multiple calibrated cameras. Each camera streams video to a central server, which detects markers, transforms positions to world coordinates, fuses observations from multiple cameras, and visualizes results in real-time 3D.

### Key Features

- **Multi-camera support**: Simultaneous tracking from 3+ cameras
- **Real-time streaming**: ZMQ-based video transport
- **Pose fusion**: Combines multiple observations for improved accuracy
- **3D visualization**: Open3D-based viewer with camera feeds
- **Distance estimation**: Real-time marker-to-camera distance display

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        REMOTE CAMERAS                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │  Camera 1   │  │  Camera 2   │  │  Camera 3   │             │
│  │  (Jetson)   │  │  (Jetson)   │  │  (Jetson)   │             │
│  │ 192.168.1.85│  │ 192.168.1.91│  │ 192.168.1.80│             │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
│         │                │                │                     │
│         │  RTSP Stream   │  RTSP Stream   │  RTSP Stream       │
│         ▼                ▼                ▼                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │   Stream    │  │   Stream    │  │   Stream    │             │
│  │   Client    │  │   Client    │  │   Client    │             │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
└─────────┼────────────────┼────────────────┼─────────────────────┘
          │                │                │
          │    ZMQ PUSH    │    ZMQ PUSH    │    ZMQ PUSH
          │    (TCP:5555)  │    (TCP:5555)  │    (TCP:5555)
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      CENTRAL SERVER                             │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                  Stream Server (ZMQ PULL)                │   │
│  │                  Receives JPEG frames                    │   │
│  └─────────────────────────┬───────────────────────────────┘   │
│                            ▼                                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Multi-Camera Tracker                        │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐      │   │
│  │  │   ArUco     │  │  Coordinate │  │    Pose     │      │   │
│  │  │  Detection  │──│  Transform  │──│   Fusion    │      │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘      │   │
│  └─────────────────────────┬───────────────────────────────┘   │
│                            ▼                                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              3D Visualization (Open3D)                   │   │
│  │  - Camera frustums    - Detection rays                   │   │
│  │  - Marker spheres     - Distance labels                  │   │
│  │  - Camera feed panel                                     │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Client Setup - Video Streaming

### Stream Client (`stream_client_standalone.py`)

The client runs on each remote camera device and streams JPEG-encoded frames to the server via ZMQ.

#### Key Components

**1. Video Capture (RTSP from Docker container)**

```python
# Open RTSP stream from Orbbec camera Docker container
source = "rtsp://127.0.0.1:8554/RGBD"

if source.startswith("rtsp://"):
    # Use TCP transport to avoid H.264/HEVC POC reference frame errors
    import os
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
    cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
```

**2. ZMQ PUSH Socket**

```python
import zmq

# Create ZMQ context and PUSH socket
context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect(f"tcp://{server_ip}:{port}")  # e.g., tcp://192.168.1.195:5555
socket.setsockopt(zmq.SNDHWM, 2)  # Limit queue to reduce latency
```

**3. Frame Encoding and Transmission**

```python
while running:
    ret, frame = cap.read()
    if not ret:
        continue

    # Encode frame as JPEG (configurable quality)
    _, encoded = cv2.imencode('.jpg', frame,
                              [cv2.IMWRITE_JPEG_QUALITY, 80])

    # Send metadata + frame data
    socket.send_json({
        'camera_id': camera_id,      # e.g., "SZVIDS-250514-07F138-1343C5-E52215"
        'timestamp': time.time(),
        'width': frame.shape[1],
        'height': frame.shape[0]
    }, zmq.SNDMORE)
    socket.send(encoded.tobytes(), zmq.NOBLOCK)
```

#### Running the Client

```bash
# On each remote camera (Jetson device)
python3 stream_client_standalone.py \
    -s 192.168.1.195 \
    -c SZVIDS-250514-07F138-1343C5-E52215
```

---

## 4. Server Setup - Marker Detection

### Stream Server (`src/streaming/stream_server.py`)

The server receives frames from all cameras via ZMQ PULL socket.

```python
class StreamServer:
    def __init__(self, port: int = 5555):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.bind(f"tcp://*:{port}")

        # Non-blocking receive with polling
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

    def receive_frame(self, timeout_ms: int = 100):
        """Receive a frame from any connected client."""
        socks = dict(self.poller.poll(timeout_ms))
        if self.socket in socks:
            metadata = self.socket.recv_json()
            frame_data = self.socket.recv()

            # Decode JPEG to numpy array
            frame = cv2.imdecode(
                np.frombuffer(frame_data, dtype=np.uint8),
                cv2.IMREAD_COLOR
            )
            return metadata['camera_id'], frame, metadata['timestamp']
        return None, None, None
```

---

## 5. Camera Calibration

### What is Camera Calibration?

Camera calibration is the process of determining the geometric properties of a camera system. For multi-camera tracking, we need to know:

1. **Intrinsic Parameters** - Internal camera properties (focal length, optical center, lens distortion)
2. **Extrinsic Parameters** - Camera position and orientation in 3D space relative to a world coordinate system

### Why is Calibration Important?

Without proper calibration:
- We cannot convert 2D image coordinates to 3D world coordinates
- Multiple cameras would not share a common reference frame
- Distance measurements would be inaccurate
- Pose fusion from multiple cameras would fail

### The Extrinsic Matrix Explained

The **extrinsic matrix** is a 4x4 transformation matrix that describes where a camera is located and how it's oriented in the world coordinate system.

```
Extrinsic Matrix (4x4):
┌                                    ┐
│  r11  r12  r13  │  tx             │
│  r21  r22  r23  │  ty             │  ← Rotation (3x3) | Translation (3x1)
│  r31  r32  r33  │  tz             │
│─────────────────┼─────────────────│
│   0    0    0   │   1             │  ← Homogeneous row
└                                    ┘
```

- **Rotation (R)**: 3x3 matrix describing camera orientation
- **Translation (t)**: 3x1 vector describing camera position
- The matrix transforms points FROM camera coordinates TO world coordinates

### Reference Camera Concept

In our system, one camera is designated as the **reference camera** (origin). Its extrinsic matrix is the identity matrix:

```
Identity (Reference Camera):
┌                 ┐
│ 1  0  0  │  0  │
│ 0  1  0  │  0  │   ← No rotation, no translation
│ 0  0  1  │  0  │   ← Camera frame = World frame
│ 0  0  0  │  1  │
└                 ┘
```

All other cameras are positioned relative to this reference.

### Calibration File Format (`calibration/calibration_result.json`)

```json
{
    "CalibrationTime": "2025-12-11 16:41:22",
    "NumberOfCalibratedCameras": 3,
    "CalibratedCameras": [
        {
            "CameraIndex": 0,
            "CameraID": "SZVIDS-250515-EAA839-91E21B-292366",
            "CameraExtrinsic": "-0.9813794, 0.190601, -0.02375974, -1.422575,
                                0.03869765, 0.07504921, -0.9964306, 3.702173,
                                -0.1881367, -0.9787935, -0.08102873, 3.617445,
                                0, 0, 0, 1",
            "IP": "192.168.1.85"
        },
        {
            "CameraIndex": 1,
            "CameraID": "SZVIDS-250513-5001E3-E1F1BA-799662",
            "CameraExtrinsic": "1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1",
            "IP": "192.168.1.91"
        }
    ]
}
```

**Note**: Camera 1 (192.168.1.91) has an identity matrix - this is our reference camera at the world origin.

### Loading Calibration

```python
def load_calibration(filepath: str) -> List[CameraPose]:
    with open(filepath, 'r') as f:
        data = json.load(f)

    camera_poses = []
    for cam in data['CalibratedCameras']:
        # Parse 4x4 extrinsic matrix from comma-separated string
        values = [float(v) for v in cam['CameraExtrinsic'].split(',')]
        extrinsic = np.array(values).reshape(4, 4)

        camera_poses.append(CameraPose(
            camera_id=cam['CameraID'],
            extrinsic=extrinsic,
            intrinsic=default_intrinsic_matrix()
        ))

    return camera_poses
```

---

## 6. ArUco Marker Detection

### What are ArUco Markers?

ArUco markers are square fiducial markers with a unique binary pattern inside a black border. They are widely used in computer vision for:

- **Camera pose estimation** - Determining where the camera is relative to the marker
- **Object tracking** - Following objects with markers attached
- **Augmented reality** - Placing virtual objects in real-world scenes

### Why ArUco Markers?

| Advantage | Description |
|-----------|-------------|
| **Unique IDs** | Each marker has a unique binary pattern (ID 0-249 in DICT_4X4_250) |
| **Fast detection** | Optimized algorithms can detect markers in real-time |
| **Robust** | Works under varying lighting and partial occlusion |
| **Known geometry** | Fixed size enables accurate distance estimation |

### Marker Structure

```
┌─────────────────────┐
│ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  ← Black border (detection)
│ ▓                 ▓ │
│ ▓   ░░▓▓░░▓▓░░   ▓ │
│ ▓   ▓▓░░▓▓░░▓▓   ▓ │  ← Binary pattern (ID encoding)
│ ▓   ░░▓▓░░▓▓░░   ▓ │     4x4 grid = 16 bits
│ ▓   ▓▓░░▓▓░░▓▓   ▓ │
│ ▓                 ▓ │
│ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │
└─────────────────────┘
     5cm x 5cm
```

### Pose Estimation with solvePnP

Once we detect a marker's corners in the image, we use the **Perspective-n-Point (PnP)** algorithm to estimate the marker's 3D pose relative to the camera.

**The Problem**: Given 4 known 3D points (marker corners) and their corresponding 2D image projections, find the rotation and translation of the marker.

**Inputs to solvePnP**:
1. **Object points** - 3D coordinates of marker corners in marker's local frame
2. **Image points** - 2D pixel coordinates of detected corners
3. **Camera matrix** - Intrinsic parameters (focal length, optical center)
4. **Distortion coefficients** - Lens distortion correction

**Outputs from solvePnP**:
1. **rvec** - Rotation vector (axis-angle representation)
2. **tvec** - Translation vector (marker position in camera frame)

### Detection Pipeline (`src/tracking/multi_camera_tracker.py`)

```python
import cv2

class MultiCameraTracker:
    def __init__(self, marker_size_m: float = 0.05):
        self.marker_size_m = marker_size_m

        # ArUco dictionary and detector parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def detect_markers(self, frame: np.ndarray, camera_id: str):
        """Detect ArUco markers in a frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is None:
            return []

        detections = []
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]

            # Define 3D object points (marker corners in marker frame)
            obj_points = np.array([
                [-self.marker_size_m / 2,  self.marker_size_m / 2, 0],
                [ self.marker_size_m / 2,  self.marker_size_m / 2, 0],
                [ self.marker_size_m / 2, -self.marker_size_m / 2, 0],
                [-self.marker_size_m / 2, -self.marker_size_m / 2, 0]
            ], dtype=np.float32)

            # Solve PnP for pose estimation
            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners,
                camera_matrix, dist_coeffs
            )

            if success:
                position_camera = tvec.flatten()  # Position in camera frame
                position_world = self._camera_to_world(position_camera, extrinsic)

                detections.append(CameraDetection(
                    camera_id=camera_id,
                    marker_id=int(marker_id),
                    position_world=position_world,
                    position_camera=position_camera
                ))

        return detections
```

---

## 7. Coordinate Transformation

### Understanding Coordinate Frames

In a multi-camera system, we work with multiple coordinate frames:

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                   │
│    Camera 1 Frame          World Frame           Camera 2 Frame  │
│         ↑ Y                    ↑ Y                    ↑ Y        │
│         │                      │                      │          │
│         │                      │                      │          │
│         └──→ X                 └──→ X                 └──→ X     │
│        ╱                      ╱                      ╱           │
│       ↙ Z                    ↙ Z                    ↙ Z          │
│                                                                   │
│    (Local to each       (Shared reference       (Local to each   │
│     camera)              for all cameras)        camera)         │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

### Why Transform Coordinates?

When a camera detects a marker, it reports the marker's position **in the camera's local coordinate frame**. To combine observations from multiple cameras, we must transform all positions to a **common world frame**.

**Example**:
- Camera 1 sees marker at position (0.5, 0.3, 2.0) in its local frame
- Camera 2 sees the same marker at position (-0.2, 0.1, 1.8) in its local frame
- After transformation to world coordinates, both should report approximately the same position

### The Transformation Process

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Marker in      │     │   Extrinsic     │     │  Marker in      │
│  Camera Frame   │ ──→ │   Matrix        │ ──→ │  World Frame    │
│  [x_c, y_c, z_c]│     │   (4x4)         │     │  [x_w, y_w, z_w]│
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

### Homogeneous Coordinates

We use **homogeneous coordinates** to represent 3D points as 4D vectors. This allows us to apply rotation AND translation in a single matrix multiplication.

```
3D Point:           Homogeneous:
[x]                 [x]
[y]        →        [y]
[z]                 [z]
                    [1]  ← Added for matrix math
```

### Camera to World Transform

```python
def _camera_to_world(self, position_camera: np.ndarray,
                      extrinsic: np.ndarray) -> np.ndarray:
    """
    Transform position from camera frame to world frame.

    Args:
        position_camera: 3D point in camera coordinates [x, y, z]
        extrinsic: 4x4 camera-to-world transformation matrix

    Returns:
        3D point in world coordinates
    """
    # Convert to homogeneous coordinates
    pos_homogeneous = np.append(position_camera, 1)  # [x, y, z, 1]

    # Apply transformation: P_world = T_cw @ P_camera
    pos_world = extrinsic @ pos_homogeneous

    return pos_world[:3]  # Return [x, y, z]
```

### Transformation Matrix Math

```
Extrinsic Matrix (4x4):
┌                           ┐
│  r11  r12  r13  |  tx    │   R = Rotation (3x3)
│  r21  r22  r23  |  ty    │   t = Translation (3x1)
│  r31  r32  r33  |  tz    │
│ ──────────────────────── │
│   0    0    0   |   1    │
└                           ┘

Transformation:
┌      ┐   ┌                    ┐   ┌      ┐
│ x_w  │   │ r11 r12 r13 | tx  │   │ x_c  │
│ y_w  │ = │ r21 r22 r23 | ty  │ × │ y_c  │
│ z_w  │   │ r31 r32 r33 | tz  │   │ z_c  │
│  1   │   │  0   0   0  |  1  │   │  1   │
└      ┘   └                    ┘   └      ┘

Expanded:
x_w = r11*x_c + r12*y_c + r13*z_c + tx
y_w = r21*x_c + r22*y_c + r23*z_c + ty
z_w = r31*x_c + r32*y_c + r33*z_c + tz
```

### Practical Example

```python
# Marker detected at (0.5, 0.3, 2.0) in Camera 1's frame
position_camera = np.array([0.5, 0.3, 2.0])

# Camera 1's extrinsic (where it is in world space)
extrinsic = np.array([
    [-0.98,  0.19, -0.02, -1.42],  # Rotated and translated
    [ 0.04,  0.08, -1.00,  3.70],  # from world origin
    [-0.19, -0.98, -0.08,  3.62],
    [ 0.00,  0.00,  0.00,  1.00]
])

# Transform to world coordinates
pos_homogeneous = np.append(position_camera, 1)  # [0.5, 0.3, 2.0, 1]
pos_world = extrinsic @ pos_homogeneous           # Matrix multiplication
# Result: position in world frame
```

---

## 8. Multi-Camera Pose Fusion

### Why Fuse Multiple Observations?

When multiple cameras see the same marker, we get multiple position estimates. Fusing these observations provides:

1. **Improved Accuracy** - Averaging reduces random measurement errors
2. **Robustness** - If one camera has a bad reading, others compensate
3. **Reduced Occlusion Impact** - Marker visible from multiple angles
4. **Depth Uncertainty Reduction** - Cameras at different angles have different depth uncertainties

### The Fusion Challenge

```
Camera 1 says marker is at: (1.02, 0.48, 2.01)
Camera 2 says marker is at: (0.98, 0.52, 1.99)
Camera 3 says marker is at: (1.05, 0.47, 2.03)

What is the TRUE position?
```

### Fusion Methods (`src/tracking/pose_fusion.py`)

The system supports multiple fusion algorithms to combine marker observations from different cameras.

#### 8.1 Weighted Average Fusion

**Concept**: Not all observations are equally reliable. Cameras closer to the marker have less depth uncertainty, so we weight them higher.

**Weight Formula**:
```
weight = confidence / (depth + 0.1)
```

Where:
- `confidence` = detection quality (default 1.0)
- `depth` = distance from camera to marker
- `+0.1` = prevents division by zero

```python
def _fuse_weighted_average(self, detections: List[CameraDetection]) -> FusedMarkerPose:
    """
    Fuse using weighted average.

    Weights are computed based on:
    - Detection confidence
    - Inverse depth (closer cameras weighted higher, less depth uncertainty)
    """
    positions = np.array([d.position_world for d in detections])

    # Compute weights: confidence / depth
    weights = []
    for d in detections:
        depth = np.linalg.norm(d.position_camera)  # Distance from camera
        weight = d.confidence / (depth + 0.1)       # Inverse depth weighting
        weights.append(weight)

    weights = np.array(weights)
    weights = weights / weights.sum()  # Normalize to sum to 1

    # Weighted average position
    fused_position = np.average(positions, axis=0, weights=weights)

    return FusedMarkerPose(
        marker_id=detections[0].marker_id,
        x=float(fused_position[0]),
        y=float(fused_position[1]),
        z=float(fused_position[2]),
        detecting_cameras=[d.camera_id for d in detections]
    )
```

#### 8.2 Median Fusion (Robust to Outliers)

```python
def _fuse_median(self, detections: List[CameraDetection]) -> FusedMarkerPose:
    """
    Fuse using median position.
    Robust to outliers - useful when one camera has a bad estimate.
    """
    positions = np.array([d.position_world for d in detections])

    # Median position (component-wise)
    fused_position = np.median(positions, axis=0)

    # Confidence based on consistency
    distances = np.linalg.norm(positions - fused_position, axis=1)
    consistency = 1.0 / (1.0 + np.mean(distances))

    return FusedMarkerPose(
        marker_id=detections[0].marker_id,
        x=float(fused_position[0]),
        y=float(fused_position[1]),
        z=float(fused_position[2]),
        confidence=consistency,
        detecting_cameras=[d.camera_id for d in detections]
    )
```

#### 8.3 Time Synchronization

```python
def get_fused_poses(self, current_time: float) -> List[FusedMarkerPose]:
    """Get fused poses for markers with recent detections."""
    window_sec = self.detection_window_ms / 1000.0  # e.g., 50ms

    for marker_id, detections in self._detection_buffer.items():
        # Filter to recent detections within time window
        recent = [d for d in detections
                  if (current_time - d.timestamp) < window_sec]

        # Deduplicate by camera_id - keep most recent from each camera
        camera_detections = {}
        for det in recent:
            if det.camera_id not in camera_detections:
                camera_detections[det.camera_id] = det
            elif det.timestamp > camera_detections[det.camera_id].timestamp:
                camera_detections[det.camera_id] = det

        unique_detections = list(camera_detections.values())

        if unique_detections:
            fused_pose = self.fuse_detections(unique_detections)
            fused_poses.append(fused_pose)

    return fused_poses
```

---

## 9. Distance Calculation

### Understanding Distance in 3D Space

When tracking markers with multiple cameras, we often want to know how far each marker is from each camera. This information is useful for:

1. **Verifying Calibration** - If measured distances don't match physical measurements, calibration may be incorrect
2. **Quality Assessment** - Farther markers have higher depth uncertainty
3. **User Feedback** - Operators can verify the system is working correctly
4. **Range Limitations** - ArUco detection accuracy decreases with distance

### Euclidean Distance in World Coordinates

The **Euclidean distance** is the "straight-line" distance between two points in 3D space. This is the most intuitive measure - what you would measure with a tape measure.

```python
def calculate_distance(marker_pos: np.ndarray, cam_pos: np.ndarray) -> float:
    """
    Calculate Euclidean distance between marker and camera in world frame.

    Args:
        marker_pos: Fused marker position [x, y, z] in world coordinates
        cam_pos: Camera position [x, y, z] from extrinsic matrix

    Returns:
        Distance in meters
    """
    # Euclidean distance formula: d = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
    distance = np.linalg.norm(marker_pos - cam_pos)
    return distance
```

### The Math Behind Distance

The Euclidean distance formula in 3D is derived from the Pythagorean theorem:

```
Distance = √[(x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²]
```

In NumPy, this is computed efficiently with `np.linalg.norm()`:

```python
# These are equivalent:
distance = np.linalg.norm(marker_pos - cam_pos)
distance = np.sqrt(np.sum((marker_pos - cam_pos)**2))
```

### Getting Camera Position from Extrinsic Matrix

The camera's position in world coordinates is stored in the **translation column** of the extrinsic matrix:

```
Extrinsic Matrix:
┌                    ┐
│ R₁₁ R₁₂ R₁₃ | tx  │  ← tx, ty, tz form the camera
│ R₂₁ R₂₂ R₂₃ | ty  │    position in world coordinates
│ R₃₁ R₃₂ R₃₃ | tz  │
│  0   0   0  |  1  │
└                    ┘

Camera position = [tx, ty, tz] = extrinsic[:3, 3]
```

### Visualization of Distance

Distance labels are displayed as 3D text overlays on the detection rays, positioned at the midpoint between marker and camera for readability:

```python
# In integrated_viewer.py
marker_pos = np.array([pose.x, pose.y, pose.z])
cam_pos = cp.extrinsic[:3, 3]  # Camera translation from extrinsic

# Calculate distance
distance = np.linalg.norm(marker_pos - cam_pos)

# Add 3D label at midpoint of detection ray
midpoint = (marker_pos + cam_pos) / 2
label = scene_widget.add_3d_label(midpoint, f"{distance:.2f}m")
label.color = gui.Color(1.0, 1.0, 1.0)  # White
label.scale = 1.5  # Larger text
```

### Distance Accuracy Considerations

The accuracy of distance measurements depends on several factors:

| Factor | Impact |
|--------|--------|
| **Calibration quality** | Poor calibration leads to systematic errors |
| **Marker detection** | Corner localization affects pose estimation |
| **Camera distance** | Farther markers have higher depth uncertainty |
| **Marker angle** | Oblique viewing angles reduce accuracy |
| **Lighting** | Poor lighting affects corner detection |

---

## 10. 3D Visualization

### Why 3D Visualization?

Real-time 3D visualization serves several critical purposes:

1. **System Verification** - Confirms cameras are positioned correctly and markers are being tracked
2. **Debugging** - Visual anomalies (markers jumping, wrong positions) help identify issues
3. **Operator Awareness** - Users can see the tracking state at a glance
4. **Demonstration** - Effectively communicates system capabilities to stakeholders

### Open3D Library

We use **Open3D**, an open-source library for 3D data processing and visualization. Key features:

- **Hardware-accelerated rendering** - Uses GPU for smooth visualization
- **Interactive controls** - Mouse rotation, zoom, pan
- **GUI widgets** - Buttons, panels, image displays
- **3D labels** - Text overlays in 3D space
- **Geometry primitives** - Spheres, lines, meshes, coordinate frames

### Visualization Components

```
┌────────────────────────────────────────────────────────────────┐
│                    3D Visualization Scene                       │
│                                                                 │
│  ┌──────────────────────────────────────────┬───────────────┐  │
│  │                                          │               │  │
│  │           3D Scene View                  │   Camera 1    │  │
│  │                                          │   [feed]      │  │
│  │    📷 Camera frustums (colored)          ├───────────────┤  │
│  │    🔴 Marker spheres (red)               │               │  │
│  │    ─── Detection rays (to cameras)       │   Camera 2    │  │
│  │    📏 Distance labels (white)            │   [feed]      │  │
│  │    ⊕ Coordinate axes                     ├───────────────┤  │
│  │    ▦ Ground grid                         │               │  │
│  │                                          │   Camera 3    │  │
│  │                                          │   [feed]      │  │
│  └──────────────────────────────────────────┴───────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

### Open3D Integrated Viewer (`src/visualization/integrated_viewer.py`)

#### Window Layout

```python
class IntegratedViewer:
    def _create_window(self):
        # Create main window
        self.window = gui.Application.instance.create_window(
            "Multi-Camera Tracking", 1600, 900
        )

        # Layout: 3D scene (left) + Camera feeds panel (right)
        self.scene_widget = gui.SceneWidget()
        self.camera_panel = gui.Vert(0, gui.Margins(10, 10, 10, 10))

        # Add camera feed thumbnails to panel
        for camera_id in camera_ids:
            label = gui.Label(f"Camera: {camera_id[-6:]}")
            image_widget = gui.ImageWidget()
            self.camera_panel.add_child(label)
            self.camera_panel.add_child(image_widget)
```

#### Rendering Camera Frustums

```python
def _add_camera_frustums(self, camera_poses: List[CameraPose]):
    """Add camera visualization to scene."""
    for pose in camera_poses:
        # Create frustum geometry
        frustum = create_camera_frustum(
            pose.intrinsic,
            pose.extrinsic,
            scale=0.3
        )

        # Color-code each camera
        color = CAMERA_COLORS[i % len(CAMERA_COLORS)]
        frustum.paint_uniform_color(color)

        self.scene_widget.scene.add_geometry(
            f"cam_{pose.camera_id}",
            frustum,
            material
        )
```

#### Rendering Markers and Detection Rays

```python
def _update_markers(self):
    """Update marker positions in 3D scene."""
    for marker_id, pose in poses.items():
        # Create/update marker sphere
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        sphere.paint_uniform_color([1.0, 0.2, 0.2])  # Red

        # Position via transform
        transform = np.eye(4)
        transform[:3, 3] = [pose.x, pose.y, pose.z]
        scene.set_geometry_transform(f"marker_{marker_id}", transform)

        # Draw detection lines to each detecting camera
        for cam_id in pose.detecting_cameras:
            cam_pos = get_camera_position(cam_id)
            marker_pos = np.array([pose.x, pose.y, pose.z])

            # Create line from marker to camera
            line = o3d.geometry.LineSet()
            line.points = o3d.utility.Vector3dVector([
                marker_pos.tolist(),
                cam_pos.tolist()
            ])
            line.lines = o3d.utility.Vector2iVector([[0, 1]])
            line.colors = o3d.utility.Vector3dVector([camera_color])

            scene.add_geometry(f"line_{marker_id}_{cam_id}", line, mat)
```

### Geometry Lifecycle Management

Managing dynamic geometry in real-time visualization requires careful tracking:

```
For each detected marker:
┌──────────────────────────────────────────────────────────────┐
│  1. Check if geometry exists for this marker_id              │
│     ├─ YES: Update transform (position/orientation)          │
│     └─ NO:  Create new geometry, add to scene                │
│                                                               │
│  2. For detection rays (marker → camera lines):              │
│     ├─ Remove ALL old lines for this marker                  │
│     ├─ Create new lines for CURRENT detecting cameras        │
│     └─ Add distance labels at midpoints                      │
│                                                               │
│  3. Track what's added to avoid duplicates:                  │
│     ├─ _added_markers: set of marker IDs with geometry       │
│     ├─ _added_lines: set of line geometry names              │
│     └─ _marker_lines: dict[marker_id] → set of line names    │
└──────────────────────────────────────────────────────────────┘
```

This approach ensures:
- No duplicate geometry for the same marker
- Old detection rays are cleaned up when cameras stop seeing a marker
- Memory is managed properly as markers come and go

### The Render Loop

The visualization runs at approximately 60 FPS with this update cycle:

```python
def _on_update(self):
    """Called every frame by Open3D timer."""

    # 1. Process message bus (receive new marker poses)
    self.bus.spin_once()

    # 2. Update marker geometries from pending poses
    self._process_marker_updates()

    # 3. Update camera feed images in side panel
    self._update_camera_feeds()

    # 4. Request redraw
    self.window.post_redraw()
```

---

## 11. Running the System

### Step 1: Start the Server

```bash
# On the central server machine
cd MultiCameraMakerTracking

python scripts/run_multi_camera_tracker.py \
    -c calibration/calibration_result.json \
    --preview
```

### Step 2: Start Clients on Each Camera

```bash
# SSH into each Jetson device and run:

# Camera 1 (192.168.1.85)
python3 stream_client_standalone.py \
    -s 192.168.1.195 \
    -c SZVIDS-250515-EAA839-91E21B-292366

# Camera 2 (192.168.1.91)
python3 stream_client_standalone.py \
    -s 192.168.1.195 \
    -c SZVIDS-250513-5001E3-E1F1BA-799662

# Camera 3 (192.168.1.80)
python3 stream_client_standalone.py \
    -s 192.168.1.195 \
    -c SZVIDS-250514-07F138-1343C5-E52215
```

### Automated Client Startup (via SSH)

```python
import paramiko

cameras = [
    ("192.168.1.85", "SZVIDS-250515-EAA839-91E21B-292366"),
    ("192.168.1.91", "SZVIDS-250513-5001E3-E1F1BA-799662"),
    ("192.168.1.80", "SZVIDS-250514-07F138-1343C5-E52215")
]

for ip, camera_id in cameras:
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username="determtech", password="determtech")

    cmd = f"nohup python3 stream_client_standalone.py -s 192.168.1.195 -c {camera_id} &"
    ssh.exec_command(cmd)
    ssh.close()
```

---

## 12. Demo Video

[Attach video here]

---

## 13. End-to-End Data Flow

This section shows how data flows through the entire system, from camera capture to 3D visualization:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           COMPLETE DATA FLOW                                 │
└─────────────────────────────────────────────────────────────────────────────┘

STEP 1: CAPTURE (on each Jetson)
┌─────────────────────────────────────────────────────────────────────────────┐
│  Docker Container (Orbbec Camera)                                            │
│       │                                                                      │
│       │  RTSP Stream (rtsp://127.0.0.1:8554/RGBD)                           │
│       ▼                                                                      │
│  OpenCV VideoCapture                                                         │
│       │                                                                      │
│       │  BGR Frame (numpy array, e.g., 1280x720x3)                          │
│       ▼                                                                      │
│  JPEG Encoding (quality=80)                                                  │
│       │                                                                      │
│       │  Compressed bytes (~50-100KB per frame)                             │
│       ▼                                                                      │
│  ZMQ PUSH Socket → TCP:5555 to server                                       │
└─────────────────────────────────────────────────────────────────────────────┘

STEP 2: RECEIVE (on server)
┌─────────────────────────────────────────────────────────────────────────────┐
│  ZMQ PULL Socket (listening on TCP:5555)                                    │
│       │                                                                      │
│       │  Receives: {camera_id, timestamp, width, height} + JPEG bytes       │
│       ▼                                                                      │
│  JPEG Decoding (cv2.imdecode)                                               │
│       │                                                                      │
│       │  BGR Frame (numpy array)                                            │
│       ▼                                                                      │
│  Frame Buffer (stores latest frame per camera_id)                           │
└─────────────────────────────────────────────────────────────────────────────┘

STEP 3: DETECT (on server, per camera)
┌─────────────────────────────────────────────────────────────────────────────┐
│  BGR Frame                                                                   │
│       │                                                                      │
│       │  cv2.cvtColor() to grayscale                                        │
│       ▼                                                                      │
│  ArUco Detector (cv2.aruco.ArucoDetector)                                   │
│       │                                                                      │
│       │  Returns: corners[], ids[] for each detected marker                 │
│       ▼                                                                      │
│  For each marker:                                                            │
│       │                                                                      │
│       │  cv2.solvePnP(obj_points, corners, camera_matrix, dist_coeffs)      │
│       ▼                                                                      │
│  Pose in CAMERA frame: rvec, tvec → position_camera [x, y, z]               │
└─────────────────────────────────────────────────────────────────────────────┘

STEP 4: TRANSFORM (on server)
┌─────────────────────────────────────────────────────────────────────────────┐
│  position_camera [x_c, y_c, z_c]                                            │
│       │                                                                      │
│       │  Convert to homogeneous: [x_c, y_c, z_c, 1]                         │
│       ▼                                                                      │
│  extrinsic_matrix @ position_homogeneous                                    │
│       │                                                                      │
│       │  4x4 matrix multiplication                                          │
│       ▼                                                                      │
│  position_world [x_w, y_w, z_w]                                             │
│       │                                                                      │
│       │  Store as CameraDetection(camera_id, marker_id, position_world)     │
│       ▼                                                                      │
│  Detection Buffer (grouped by marker_id)                                     │
└─────────────────────────────────────────────────────────────────────────────┘

STEP 5: FUSE (on server)
┌─────────────────────────────────────────────────────────────────────────────┐
│  Detection Buffer: marker_id → [detection1, detection2, detection3, ...]    │
│       │                                                                      │
│       │  Filter by time window (keep recent detections within 50ms)         │
│       │  Deduplicate by camera_id (keep most recent from each)              │
│       ▼                                                                      │
│  If multiple cameras see same marker:                                        │
│       │                                                                      │
│       │  Weighted average: sum(weight_i * pos_i) / sum(weight_i)            │
│       │  where weight = confidence / depth                                   │
│       ▼                                                                      │
│  FusedMarkerPose(marker_id, x, y, z, detecting_cameras=[cam1, cam2, ...])   │
│       │                                                                      │
│       │  Publish to message bus: /markers/fused_poses                       │
│       ▼                                                                      │
│  Message Bus delivers to all subscribers                                     │
└─────────────────────────────────────────────────────────────────────────────┘

STEP 6: VISUALIZE (on server)
┌─────────────────────────────────────────────────────────────────────────────┐
│  Message Bus callback receives FusedMarkerPose                               │
│       │                                                                      │
│       │  Store in pending updates (thread-safe queue)                       │
│       ▼                                                                      │
│  Render loop (60 FPS):                                                       │
│       │                                                                      │
│       │  Process pending updates                                             │
│       ▼                                                                      │
│  For each marker:                                                            │
│       ├─ Create/update sphere at position [x, y, z]                         │
│       ├─ Create/update coordinate axes                                       │
│       └─ For each detecting camera:                                         │
│            ├─ Draw line from marker to camera                               │
│            ├─ Calculate distance = ||marker_pos - cam_pos||                 │
│            └─ Add 3D label with distance at line midpoint                   │
│       │                                                                      │
│       │  Open3D renders scene                                               │
│       ▼                                                                      │
│  User sees: 3D scene + camera feeds + markers + rays + distances            │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Timing Summary

| Stage | Typical Latency |
|-------|-----------------|
| Camera → RTSP | ~30ms |
| RTSP → Client | ~10ms |
| JPEG Encode | ~5ms |
| Network Transfer | ~5-20ms |
| JPEG Decode | ~3ms |
| ArUco Detection | ~10ms |
| Pose Estimation | ~2ms per marker |
| Coordinate Transform | <1ms |
| Pose Fusion | <1ms |
| Render Update | ~16ms (60 FPS) |
| **End-to-End** | **~80-100ms** |

---

## Summary

| Component | Technology | Purpose |
|-----------|------------|---------|
| Video Streaming | ZMQ PUSH/PULL | Low-latency frame transport |
| Video Encoding | OpenCV JPEG | Bandwidth-efficient compression |
| Marker Detection | OpenCV ArUco | Fiducial marker identification |
| Pose Estimation | cv2.solvePnP | 6-DOF marker pose |
| Coordinate Transform | 4x4 Matrix | Camera to world conversion |
| Pose Fusion | Weighted Average / Median | Multi-view combination |
| 3D Visualization | Open3D | Interactive scene rendering |
| Distance Display | Euclidean norm | Real-time measurements |

---

## Dependencies

```
opencv-python>=4.5
pyzmq>=22.0
numpy>=1.19
open3d>=0.15
scipy>=1.7
paramiko>=2.7  # For remote client management
```

---

*Document generated for technical presentation*
*MultiCameraMakerTracking Project*
