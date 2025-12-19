# Robot Navigation via External Marker Tracking

## Project Specification Document

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Development Methodology](#2-development-methodology)
3. [Phase 1: Tracking Validation](#3-phase-1-tracking-validation)
4. [Phase 2: ROS2 Integration](#4-phase-2-ros2-integration)
5. [Phase 3: Gazebo Simulation Environment](#5-phase-3-gazebo-simulation-environment)
6. [Phase 4: Point Cloud Obstacle Mapping](#6-phase-4-point-cloud-obstacle-mapping)
7. [Phase 5: Robot Navigation](#7-phase-5-robot-navigation)
8. [Technical Requirements](#8-technical-requirements)
9. [Validation Criteria](#9-validation-criteria)
10. [File Structure](#10-file-structure)

---

## 1. Project Overview

### Objective

Build a complete robot navigation system where a small robot car navigates through an environment using **only external camera-based tracking**. The robot has NO onboard sensors - all localization and mapping data comes from ceiling-mounted depth cameras tracking an ArUco marker on the robot.

### System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SIMULATION ENVIRONMENT                             │
│                                                                              │
│    ┌─────────────────────────────────────────────────────────────────────┐  │
│    │                      RECTANGULAR ROOM                                │  │
│    │                                                                      │  │
│    │   📷 Camera 1                                      📷 Camera 2      │  │
│    │   (Corner NW)                                      (Corner NE)      │  │
│    │   ↘ 45° down                                      ↙ 45° down        │  │
│    │                                                                      │  │
│    │          ┌─────┐     ┌─────┐                                        │  │
│    │          │ OBS │     │ OBS │    Random Obstacles                    │  │
│    │          └─────┘     └─────┘                                        │  │
│    │                                                                      │  │
│    │                    ┌───────────┐                                    │  │
│    │                    │   🤖      │                                    │  │
│    │                    │  ROBOT    │  ← ArUco Marker on top             │  │
│    │                    │   🎯      │                                    │  │
│    │                    └───────────┘                                    │  │
│    │                                                                      │  │
│    │          ┌─────┐              ┌─────┐                               │  │
│    │          │ OBS │              │ OBS │                               │  │
│    │          └─────┘              └─────┘                               │  │
│    │                                                                      │  │
│    │   📷 Camera 3                                      📷 Camera 4      │  │
│    │   (Corner SW)                                      (Corner SE)      │  │
│    │   ↗ 45° down                                      ↖ 45° down        │  │
│    │                                                                      │  │
│    └─────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘

Data Flow:
┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│   4 Depth    │───▶│   Marker     │───▶│    Pose      │───▶│   ROS2       │
│   Cameras    │    │   Tracking   │    │   Estimation │    │   Topics     │
└──────────────┘    └──────────────┘    └──────────────┘    └──────────────┘
                                                                   │
                                                                   ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│   Robot      │◀───│   Nav2       │◀───│   Obstacle   │◀───│   Point      │
│   Motion     │    │   Planner    │    │   Map        │    │   Cloud      │
└──────────────┘    └──────────────┘    └──────────────┘    └──────────────┘
```

### Key Constraints

- **NO onboard sensors** on the robot (no lidar, no camera, no IMU)
- All localization comes from external marker tracking
- All obstacle mapping comes from depth camera point clouds
- Robot receives pose updates via ROS2 topics
- Navigation uses Nav2 stack with custom localization

---

## 2. Development Methodology

### CRITICAL: Validation-Driven Development

**Each phase MUST be fully validated before proceeding to the next phase.** This is a strict gate - do NOT move forward until all validation criteria for the current phase pass.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        PHASE PROGRESSION WITH GATES                          │
└─────────────────────────────────────────────────────────────────────────────┘

  Phase 1                Phase 2                Phase 3
  Tracking    ──────▶    ROS2       ──────▶    Gazebo     ──────▶ ...
  Validation            Integration            Simulation
       │                     │                     │
       ▼                     ▼                     ▼
  ┌─────────┐           ┌─────────┐           ┌─────────┐
  │  GATE   │           │  GATE   │           │  GATE   │
  │  CHECK  │           │  CHECK  │           │  CHECK  │
  └────┬────┘           └────┬────┘           └────┬────┘
       │                     │                     │
   ✓ Pass?               ✓ Pass?               ✓ Pass?
   ✗ Fail → Fix          ✗ Fail → Fix          ✗ Fail → Fix
```

### Why Gazebo-First Validation?

Gazebo simulation provides **ground truth** that real-world testing cannot:

| Advantage | Description |
|-----------|-------------|
| **Known positions** | Robot and marker positions are exactly known in simulation |
| **Repeatable tests** | Same scenario can be run multiple times identically |
| **No hardware issues** | Eliminates camera calibration errors, network latency variability |
| **Safe testing** | Robot can crash into obstacles without damage |
| **Faster iteration** | No need to physically set up test scenarios |

### Phase Validation Protocol

For EACH phase, you must:

1. **Implement** the phase functionality
2. **Write validation tests** specific to that phase
3. **Run validation tests** in Gazebo simulation
4. **Document results** with pass/fail status
5. **Fix any failures** before proceeding
6. **Get explicit confirmation** that gate is passed

### Validation Test Output Format

Every validation test should produce output in this format:

```
╔══════════════════════════════════════════════════════════════════════════════╗
║                    PHASE X VALIDATION REPORT                                  ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Test: [Test Name]                                                            ║
║  Description: [What is being tested]                                          ║
║  Method: [How the test was conducted]                                         ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Expected: [Expected value/behavior]                                          ║
║  Actual:   [Measured value/behavior]                                          ║
║  Error:    [Deviation from expected]                                          ║
║  Tolerance: [Acceptable error range]                                          ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  STATUS: [ PASS ✓ ] or [ FAIL ✗ ]                                            ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

### Gate Criteria Summary

| Phase | Gate Criteria | Validation Method |
|-------|---------------|-------------------|
| **Phase 1** | Distance error < 5%, Orientation error < 5°, Velocity error < 10% | Gazebo with known marker positions |
| **Phase 2** | All ROS2 topics publishing correctly, TF tree complete | `ros2 topic echo`, `ros2 run tf2_tools view_frames` |
| **Phase 3** | All 4 cameras see marker, Robot controllable via cmd_vel | Gazebo visualization, teleop test |
| **Phase 4** | Point cloud stitched, Occupancy grid generated, Obstacles visible | RViz visualization of /map topics |
| **Phase 5** | Robot navigates to goal, Avoids obstacles, Reaches within 10cm | Multiple navigation goal tests |

---

## 3. Phase 1: Tracking Validation

> **IMPORTANT**: This phase validates the EXISTING tracking code before any ROS2 integration.
> Use Gazebo simulation to get ground truth positions for comparison.

### 3.1 Distance Validation

**Objective**: Verify that measured distances match ground truth within acceptable tolerance.

#### Test Procedure

```python
# Test cases for distance validation
DISTANCE_TEST_CASES = [
    {"actual_distance_m": 0.5, "tolerance_m": 0.02},   # 50cm ± 2cm
    {"actual_distance_m": 1.0, "tolerance_m": 0.03},   # 1m ± 3cm
    {"actual_distance_m": 2.0, "tolerance_m": 0.05},   # 2m ± 5cm
    {"actual_distance_m": 3.0, "tolerance_m": 0.08},   # 3m ± 8cm
    {"actual_distance_m": 4.0, "tolerance_m": 0.10},   # 4m ± 10cm
]
```

#### Validation Script Requirements

```python
class DistanceValidator:
    """
    Validate distance measurements from marker tracking.

    Test methodology:
    1. Place marker at known distance from camera (measured with tape/laser)
    2. Record tracked distance for N samples (N >= 100)
    3. Calculate: mean, std, min, max, error percentage
    4. Generate validation report
    """

    def validate_distance(self, actual_m: float, samples: List[float]) -> ValidationResult:
        """
        Args:
            actual_m: Ground truth distance in meters
            samples: List of measured distances

        Returns:
            ValidationResult with pass/fail and statistics
        """
        mean = np.mean(samples)
        std = np.std(samples)
        error = abs(mean - actual_m)
        error_pct = (error / actual_m) * 100

        return ValidationResult(
            passed=error < tolerance,
            mean=mean,
            std=std,
            error_m=error,
            error_pct=error_pct
        )
```

#### Expected Output

```
Distance Validation Report
==========================
Test 1: 0.5m actual
  - Mean measured: 0.512m
  - Std dev: 0.008m
  - Error: 0.012m (2.4%)
  - Status: PASS ✓

Test 2: 1.0m actual
  - Mean measured: 1.023m
  - Std dev: 0.015m
  - Error: 0.023m (2.3%)
  - Status: PASS ✓
...
```

### 3.2 Rotation/Orientation Validation

**Objective**: Verify that marker orientation (roll, pitch, yaw) is accurate.

#### Test Procedure

```python
# Test cases for rotation validation
ROTATION_TEST_CASES = [
    {"axis": "yaw", "angles_deg": [0, 45, 90, 135, 180, -45, -90], "tolerance_deg": 3.0},
    {"axis": "pitch", "angles_deg": [0, 15, 30, 45, -15, -30], "tolerance_deg": 5.0},
    {"axis": "roll", "angles_deg": [0, 15, 30, -15, -30], "tolerance_deg": 5.0},
]
```

#### Validation Script Requirements

```python
class OrientationValidator:
    """
    Validate orientation measurements from marker tracking.

    Test methodology:
    1. Mount marker on rotatable platform with angle markings
    2. Set marker to known orientation
    3. Record tracked orientation for N samples
    4. Compare Euler angles (roll, pitch, yaw)
    5. Also validate quaternion consistency
    """

    def validate_orientation(self, actual_euler: Tuple[float, float, float],
                            samples: List[Tuple]) -> ValidationResult:
        """
        Args:
            actual_euler: Ground truth (roll, pitch, yaw) in degrees
            samples: List of measured (roll, pitch, yaw) tuples

        Returns:
            ValidationResult with angular errors
        """
        # Convert to rotation matrices for proper comparison
        # Handle gimbal lock and angle wrapping
        pass

    def rotation_matrix_from_rvec(self, rvec: np.ndarray) -> np.ndarray:
        """Convert OpenCV rvec to 3x3 rotation matrix."""
        R, _ = cv2.Rodrigues(rvec)
        return R

    def rotation_to_euler(self, R: np.ndarray) -> Tuple[float, float, float]:
        """Extract Euler angles from rotation matrix (ZYX convention)."""
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2,1], R[2,2])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = np.arctan2(R[1,0], R[0,0])
        else:
            roll = np.arctan2(-R[1,2], R[1,1])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = 0

        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
```

### 3.3 Velocity Estimation Validation

**Objective**: Verify that velocity estimation is accurate for navigation.

#### Velocity Estimation Implementation

```python
class VelocityEstimator:
    """
    Estimate linear and angular velocity from pose history.

    Methods:
    1. Simple differentiation: v = (p2 - p1) / dt
    2. Kalman filter: Smooth estimates, predict during occlusion
    3. Moving average: Reduce noise in velocity estimates
    """

    def __init__(self, smoothing_window: int = 5):
        self.pose_history: deque = deque(maxlen=100)
        self.smoothing_window = smoothing_window

    def add_pose(self, timestamp: float, position: np.ndarray,
                 orientation: np.ndarray):
        """Add new pose observation."""
        self.pose_history.append({
            'time': timestamp,
            'position': position,
            'orientation': orientation
        })

    def get_linear_velocity(self) -> np.ndarray:
        """
        Calculate linear velocity [vx, vy, vz] in m/s.

        Uses central difference for smoother estimates:
        v(t) = (p(t+dt) - p(t-dt)) / (2*dt)
        """
        if len(self.pose_history) < 3:
            return np.zeros(3)

        # Use recent poses for velocity calculation
        recent = list(self.pose_history)[-self.smoothing_window:]

        # Fit line to positions over time window
        times = np.array([p['time'] for p in recent])
        positions = np.array([p['position'] for p in recent])

        # Linear regression for each axis
        velocities = []
        for axis in range(3):
            coeffs = np.polyfit(times - times[0], positions[:, axis], 1)
            velocities.append(coeffs[0])  # Slope = velocity

        return np.array(velocities)

    def get_angular_velocity(self) -> np.ndarray:
        """
        Calculate angular velocity [wx, wy, wz] in rad/s.

        Uses quaternion differentiation:
        w = 2 * q_dot * q_conjugate
        """
        if len(self.pose_history) < 2:
            return np.zeros(3)

        # Get two recent orientations
        q1 = self.pose_history[-2]['orientation']  # quaternion [w, x, y, z]
        q2 = self.pose_history[-1]['orientation']
        dt = self.pose_history[-1]['time'] - self.pose_history[-2]['time']

        if dt < 1e-6:
            return np.zeros(3)

        # Quaternion derivative
        q_dot = (q2 - q1) / dt

        # Angular velocity from quaternion derivative
        # w = 2 * q_dot * q_conjugate
        q_conj = np.array([q1[0], -q1[1], -q1[2], -q1[3]])
        w_quat = 2 * self._quat_multiply(q_dot, q_conj)

        return w_quat[1:4]  # Return [wx, wy, wz]
```

#### Velocity Validation Test

```python
VELOCITY_TEST_CASES = [
    # Linear velocity tests
    {"type": "linear", "direction": "x", "speed_mps": 0.1, "tolerance_mps": 0.02},
    {"type": "linear", "direction": "x", "speed_mps": 0.5, "tolerance_mps": 0.05},
    {"type": "linear", "direction": "y", "speed_mps": 0.3, "tolerance_mps": 0.03},

    # Angular velocity tests
    {"type": "angular", "axis": "yaw", "speed_dps": 30, "tolerance_dps": 5},
    {"type": "angular", "axis": "yaw", "speed_dps": 90, "tolerance_dps": 10},
]
```

### 3.4 Multi-Camera Fusion Validation

**Objective**: Verify that fused poses from multiple cameras are consistent and accurate.

```python
class FusionValidator:
    """
    Validate multi-camera pose fusion.

    Tests:
    1. Consistency: All cameras agree on position (within tolerance)
    2. Accuracy: Fused position matches ground truth
    3. Robustness: System handles single camera failure
    4. Latency: Fusion adds minimal delay
    """

    def validate_consistency(self, per_camera_poses: Dict[str, np.ndarray],
                            tolerance_m: float = 0.05) -> bool:
        """Check that all cameras report similar positions."""
        positions = list(per_camera_poses.values())

        # Calculate pairwise distances
        for i, p1 in enumerate(positions):
            for p2 in positions[i+1:]:
                dist = np.linalg.norm(p1 - p2)
                if dist > tolerance_m:
                    return False
        return True

    def validate_occlusion_handling(self):
        """
        Test system behavior when marker is occluded from some cameras.

        Scenarios:
        1. One camera occluded: Should use remaining cameras
        2. Two cameras occluded: Should still function with 2
        3. Three cameras occluded: Degraded mode, single camera
        4. All cameras occluded: Last known pose + dead reckoning
        """
        pass
```

### 3.5 PHASE 1 GATE CHECKPOINT

**Before proceeding to Phase 2, ALL of the following must pass:**

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                    PHASE 1 GATE CHECKLIST                                  ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃  [ ] Distance Validation                                                   ┃
┃      □ All distance tests pass (error < tolerance for each range)          ┃
┃      □ Repeatability test passes (std dev < 1cm over 100 samples)          ┃
┃                                                                            ┃
┃  [ ] Orientation Validation                                                ┃
┃      □ Yaw accuracy < 3° for all test angles                               ┃
┃      □ Pitch accuracy < 5° for all test angles                             ┃
┃      □ Roll accuracy < 5° for all test angles                              ┃
┃                                                                            ┃
┃  [ ] Velocity Estimation                                                   ┃
┃      □ Linear velocity error < 10% for tested speeds                       ┃
┃      □ Angular velocity error < 15% for tested rotation rates              ┃
┃      □ Velocity estimation stable (no wild jumps)                          ┃
┃                                                                            ┃
┃  [ ] Multi-Camera Fusion                                                   ┃
┃      □ All cameras report consistent positions (within 5cm)                ┃
┃      □ Fused position matches ground truth                                 ┃
┃      □ System handles single camera occlusion gracefully                   ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

**Validation Script to Run:**

```bash
# Run all Phase 1 validation tests
python3 validation/run_phase1_validation.py --gazebo

# Expected output: All tests PASS
# If ANY test fails, DO NOT proceed to Phase 2
```

**If gate fails:** Fix the tracking code, re-run validation, repeat until all pass.

---

## 4. Phase 2: ROS2 Integration

> **IMPORTANT**: Only start this phase after Phase 1 gate is passed.
> This phase wraps the validated tracking code into ROS2 nodes.

### 4.1 ROS2 Package Structure

```
multi_camera_robot_nav/
├── package.xml
├── setup.py
├── CMakeLists.txt
├── config/
│   ├── camera_params.yaml
│   ├── tracker_params.yaml
│   └── nav2_params.yaml
├── launch/
│   ├── tracking.launch.py
│   ├── simulation.launch.py
│   └── navigation.launch.py
├── multi_camera_robot_nav/
│   ├── __init__.py
│   ├── marker_tracker_node.py
│   ├── pose_publisher_node.py
│   ├── velocity_estimator_node.py
│   ├── point_cloud_stitcher_node.py
│   └── obstacle_map_node.py
├── msg/
│   ├── MarkerPose.msg
│   ├── TrackedRobot.msg
│   └── CameraStatus.msg
├── srv/
│   └── GetRobotPose.srv
├── urdf/
│   ├── robot.urdf.xacro
│   └── camera_mount.urdf.xacro
└── rviz/
    └── robot_nav.rviz
```

### 4.2 ROS2 Topics

```yaml
# Published Topics
/robot/pose:
  type: geometry_msgs/PoseStamped
  description: "Fused robot pose from marker tracking"
  frame_id: "world"
  rate: 30 Hz

/robot/velocity:
  type: geometry_msgs/TwistStamped
  description: "Estimated robot velocity"
  frame_id: "base_link"
  rate: 30 Hz

/robot/odom:
  type: nav_msgs/Odometry
  description: "Odometry from marker tracking (for Nav2)"
  frame_id: "odom"
  child_frame_id: "base_link"
  rate: 30 Hz

/cameras/camera_X/depth/points:
  type: sensor_msgs/PointCloud2
  description: "Depth point cloud from camera X"
  frame_id: "camera_X_optical_frame"
  rate: 15 Hz

/map/obstacle_cloud:
  type: sensor_msgs/PointCloud2
  description: "Stitched obstacle point cloud"
  frame_id: "world"
  rate: 5 Hz

/map/occupancy_grid:
  type: nav_msgs/OccupancyGrid
  description: "2D occupancy grid for navigation"
  frame_id: "world"
  rate: 1 Hz

# Subscribed Topics (by Nav2)
/cmd_vel:
  type: geometry_msgs/Twist
  description: "Velocity commands to robot"
```

### 4.3 TF Tree

```
world
├── camera_1_link
│   └── camera_1_optical_frame
├── camera_2_link
│   └── camera_2_optical_frame
├── camera_3_link
│   └── camera_3_optical_frame
├── camera_4_link
│   └── camera_4_optical_frame
├── odom
│   └── base_link
│       ├── marker_link
│       └── base_footprint
```

### 4.4 Marker Tracker Node

```python
#!/usr/bin/env python3
"""
ROS2 Node: Marker Tracker

Subscribes to camera images, detects ArUco markers,
publishes robot pose and velocity.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import cv2
from cv_bridge import CvBridge
import numpy as np


class MarkerTrackerNode(Node):
    def __init__(self):
        super().__init__('marker_tracker_node')

        # Parameters
        self.declare_parameter('marker_size', 0.10)  # 10cm marker
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('fusion_method', 'weighted_average')

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Subscribers (4 cameras)
        self.camera_subs = {}
        self.camera_info = {}
        for i in range(1, 5):
            self.camera_subs[f'camera_{i}'] = self.create_subscription(
                Image,
                f'/cameras/camera_{i}/color/image_raw',
                lambda msg, cam_id=f'camera_{i}': self.image_callback(msg, cam_id),
                10
            )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/robot/velocity', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Velocity estimator
        self.velocity_estimator = VelocityEstimator()

        # Detection buffer for fusion
        self.detection_buffer = {}

        self.bridge = CvBridge()
        self.get_logger().info('Marker Tracker Node initialized')

    def image_callback(self, msg: Image, camera_id: str):
        """Process image from a camera."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            detection = self.detect_marker(cv_image, camera_id)

            if detection is not None:
                self.detection_buffer[camera_id] = {
                    'detection': detection,
                    'timestamp': msg.header.stamp
                }

                # Attempt fusion and publish
                self.fuse_and_publish()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_marker(self, image: np.ndarray, camera_id: str):
        """Detect target marker and estimate pose."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        target_id = self.get_parameter('target_marker_id').value

        if ids is not None and target_id in ids.flatten():
            idx = np.where(ids.flatten() == target_id)[0][0]
            marker_corners = corners[idx][0]

            # Get camera intrinsics and extrinsics
            camera_matrix = self.camera_info[camera_id]['K']
            dist_coeffs = self.camera_info[camera_id]['D']
            extrinsic = self.camera_info[camera_id]['extrinsic']

            # Solve PnP
            marker_size = self.get_parameter('marker_size').value
            obj_points = np.array([
                [-marker_size/2,  marker_size/2, 0],
                [ marker_size/2,  marker_size/2, 0],
                [ marker_size/2, -marker_size/2, 0],
                [-marker_size/2, -marker_size/2, 0]
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners,
                camera_matrix, dist_coeffs
            )

            if success:
                # Transform to world coordinates
                pos_camera = tvec.flatten()
                pos_world = self.camera_to_world(pos_camera, extrinsic)

                # Get orientation in world frame
                R_marker, _ = cv2.Rodrigues(rvec)
                R_world = extrinsic[:3, :3] @ R_marker

                return {
                    'position': pos_world,
                    'rotation': R_world,
                    'camera_id': camera_id
                }

        return None

    def fuse_and_publish(self):
        """Fuse detections from multiple cameras and publish."""
        # Get recent detections (within 50ms window)
        current_time = self.get_clock().now()
        recent_detections = []

        for cam_id, data in self.detection_buffer.items():
            age_ns = (current_time - rclpy.time.Time.from_msg(data['timestamp'])).nanoseconds
            if age_ns < 50_000_000:  # 50ms
                recent_detections.append(data['detection'])

        if not recent_detections:
            return

        # Fuse positions (weighted average)
        positions = np.array([d['position'] for d in recent_detections])
        fused_position = np.mean(positions, axis=0)

        # Fuse orientations (average quaternions)
        fused_rotation = recent_detections[0]['rotation']  # Simplified

        # Update velocity estimator
        timestamp = current_time.nanoseconds / 1e9
        quat = self.rotation_to_quaternion(fused_rotation)
        self.velocity_estimator.add_pose(timestamp, fused_position, quat)

        # Publish pose
        self.publish_pose(fused_position, quat, current_time)

        # Publish velocity
        linear_vel = self.velocity_estimator.get_linear_velocity()
        angular_vel = self.velocity_estimator.get_angular_velocity()
        self.publish_velocity(linear_vel, angular_vel, current_time)

        # Publish odometry
        self.publish_odometry(fused_position, quat, linear_vel, angular_vel, current_time)

        # Broadcast TF
        self.broadcast_tf(fused_position, quat, current_time)

    def publish_pose(self, position, quaternion, stamp):
        """Publish PoseStamped message."""
        msg = PoseStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = float(quaternion[0])
        msg.pose.orientation.x = float(quaternion[1])
        msg.pose.orientation.y = float(quaternion[2])
        msg.pose.orientation.z = float(quaternion[3])
        self.pose_pub.publish(msg)

    def publish_odometry(self, position, quaternion, linear_vel, angular_vel, stamp):
        """Publish Odometry message for Nav2."""
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Pose
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.pose.pose.orientation.w = float(quaternion[0])
        msg.pose.pose.orientation.x = float(quaternion[1])
        msg.pose.pose.orientation.y = float(quaternion[2])
        msg.pose.pose.orientation.z = float(quaternion[3])

        # Twist
        msg.twist.twist.linear.x = float(linear_vel[0])
        msg.twist.twist.linear.y = float(linear_vel[1])
        msg.twist.twist.linear.z = float(linear_vel[2])
        msg.twist.twist.angular.x = float(angular_vel[0])
        msg.twist.twist.angular.y = float(angular_vel[1])
        msg.twist.twist.angular.z = float(angular_vel[2])

        self.odom_pub.publish(msg)

    def broadcast_tf(self, position, quaternion, stamp):
        """Broadcast TF transform from odom to base_link."""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.w = float(quaternion[0])
        t.transform.rotation.x = float(quaternion[1])
        t.transform.rotation.y = float(quaternion[2])
        t.transform.rotation.z = float(quaternion[3])
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4.5 RViz Configuration

```yaml
# rviz/robot_nav.rviz
Panels:
  - Class: rviz_common/Displays
  - Class: rviz_common/Views

Visualization Manager:
  Displays:
    # Robot Model
    - Class: rviz_default_plugins/RobotModel
      Name: Robot
      Robot Description: robot_description
      TF Prefix: ""

    # TF Frames
    - Class: rviz_default_plugins/TF
      Name: TF
      Show Names: true
      Show Axes: true

    # Robot Pose (from marker tracking)
    - Class: rviz_default_plugins/Pose
      Name: Robot Pose
      Topic: /robot/pose
      Color: 0; 255; 0  # Green
      Shaft Length: 0.2
      Head Length: 0.05

    # Point Cloud (obstacles)
    - Class: rviz_default_plugins/PointCloud2
      Name: Obstacle Cloud
      Topic: /map/obstacle_cloud
      Size: 0.02
      Color Transformer: FlatColor
      Color: 255; 0; 0  # Red

    # Occupancy Grid (2D map)
    - Class: rviz_default_plugins/Map
      Name: Obstacle Map
      Topic: /map/occupancy_grid
      Color Scheme: costmap

    # Path (Nav2 planned path)
    - Class: rviz_default_plugins/Path
      Name: Planned Path
      Topic: /plan
      Color: 255; 255; 0  # Yellow

    # Camera Frustums (visualization)
    - Class: rviz_default_plugins/MarkerArray
      Name: Camera Frustums
      Topic: /cameras/frustums

  Global Options:
    Fixed Frame: world
    Frame Rate: 30
```

### 4.6 PHASE 2 GATE CHECKPOINT

**Before proceeding to Phase 3, ALL of the following must pass:**

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                    PHASE 2 GATE CHECKLIST                                  ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃  [ ] ROS2 Package Builds                                                   ┃
┃      □ `colcon build` completes without errors                             ┃
┃      □ All Python nodes are executable                                     ┃
┃                                                                            ┃
┃  [ ] Topics Publishing                                                     ┃
┃      □ /robot/pose publishing at 30Hz                                      ┃
┃      □ /robot/odom publishing at 30Hz                                      ┃
┃      □ /robot/velocity publishing at 30Hz                                  ┃
┃      □ Data format matches message definitions                             ┃
┃                                                                            ┃
┃  [ ] TF Tree Complete                                                      ┃
┃      □ `ros2 run tf2_tools view_frames` shows complete tree                ┃
┃      □ world → map → odom → base_link chain exists                         ┃
┃      □ Camera frames all connected to world                                ┃
┃                                                                            ┃
┃  [ ] RViz Visualization                                                    ┃
┃      □ Robot model visible in RViz                                         ┃
┃      □ Pose arrow updates in real-time                                     ┃
┃      □ TF frames visible and updating                                      ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

**Validation Commands to Run:**

```bash
# Build the package
cd ~/ros2_ws && colcon build --packages-select multi_camera_robot_nav

# Source and run nodes
source install/setup.bash
ros2 launch multi_camera_robot_nav tracking.launch.py

# In separate terminals, verify:
ros2 topic hz /robot/pose          # Should show ~30 Hz
ros2 topic echo /robot/pose        # Should show valid pose data
ros2 run tf2_tools view_frames     # Should generate frames.pdf

# If ANY check fails, DO NOT proceed to Phase 3
```

**If gate fails:** Debug ROS2 nodes, fix publishers/TF, re-run validation.

---

## 5. Phase 3: Gazebo Simulation Environment

> **IMPORTANT**: Only start this phase after Phase 2 gate is passed.
> This phase creates the simulation environment for testing.

### 5.1 World File

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="marker_tracking_world">

    <!-- Physics -->
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane (Room Floor) -->
    <model name="floor">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>6 4 0.1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>6 4 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Room Walls -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 2 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>6 0.1 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>6 0.1 2</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_south">
      <static>true</static>
      <pose>0 -2 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>6 0.1 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>6 0.1 2</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_east">
      <static>true</static>
      <pose>3 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.1 4 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.1 4 2</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_west">
      <static>true</static>
      <pose>-3 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.1 4 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.1 4 2</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Ceiling -->
    <model name="ceiling">
      <static>true</static>
      <pose>0 0 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>6 4 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>6 4 0.1</size></box></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Include camera models (defined in separate files) -->
    <include>
      <uri>model://depth_camera</uri>
      <name>camera_1</name>
      <pose>-2.5 1.5 2.4 0 0.785 0.785</pose> <!-- NW corner, 45° pitch, looking SE -->
    </include>

    <include>
      <uri>model://depth_camera</uri>
      <name>camera_2</name>
      <pose>2.5 1.5 2.4 0 0.785 2.356</pose> <!-- NE corner, 45° pitch, looking SW -->
    </include>

    <include>
      <uri>model://depth_camera</uri>
      <name>camera_3</name>
      <pose>-2.5 -1.5 2.4 0 0.785 -0.785</pose> <!-- SW corner, 45° pitch, looking NE -->
    </include>

    <include>
      <uri>model://depth_camera</uri>
      <name>camera_4</name>
      <pose>2.5 -1.5 2.4 0 0.785 -2.356</pose> <!-- SE corner, 45° pitch, looking NW -->
    </include>

  </world>
</sdf>
```

### 5.2 Depth Camera Model (Orbbec-style)

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="depth_camera">
    <static>true</static>

    <link name="camera_link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <box><size>0.05 0.15 0.05</size></box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
        </material>
      </visual>

      <!-- RGB Camera Sensor -->
      <sensor name="color_camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
          <image>
            <width>1280</width>
            <height>720</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10.0</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>

        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/cameras</namespace>
            <remapping>image_raw:=camera_${name}/color/image_raw</remapping>
            <remapping>camera_info:=camera_${name}/color/camera_info</remapping>
          </ros>
          <camera_name>camera_${name}_color</camera_name>
          <frame_name>camera_${name}_color_optical_frame</frame_name>
        </plugin>
      </sensor>

      <!-- Depth Camera Sensor -->
      <sensor name="depth_camera" type="depth">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
            <near>0.3</near>
            <far>5.0</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <update_rate>15</update_rate>
        <visualize>true</visualize>

        <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/cameras</namespace>
            <remapping>image_raw:=camera_${name}/depth/image_raw</remapping>
            <remapping>camera_info:=camera_${name}/depth/camera_info</remapping>
            <remapping>points:=camera_${name}/depth/points</remapping>
          </ros>
          <camera_name>camera_${name}_depth</camera_name>
          <frame_name>camera_${name}_depth_optical_frame</frame_name>
          <min_depth>0.3</min_depth>
          <max_depth>5.0</max_depth>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
```

### 5.3 Robot Model with ArUco Marker

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="tracking_robot">

  <!-- Properties -->
  <xacro:property name="body_length" value="0.3"/>
  <xacro:property name="body_width" value="0.2"/>
  <xacro:property name="body_height" value="0.1"/>
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.02"/>
  <xacro:property name="marker_size" value="0.1"/>

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 ${wheel_radius + body_height/2}" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>

    <visual>
      <origin xyz="0 0 ${wheel_radius + body_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 ${wheel_radius + body_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
    </collision>
  </link>

  <!-- ArUco Marker on top -->
  <link name="marker_link">
    <visual>
      <origin xyz="0 0 0.001" rpy="0 0 0"/>
      <geometry>
        <box size="${marker_size} ${marker_size} 0.002"/>
      </geometry>
      <material name="aruco_marker">
        <!-- Texture would be applied here for actual ArUco pattern -->
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="marker_joint" type="fixed">
    <parent link="base_link"/>
    <child link="marker_link"/>
    <origin xyz="0 0 ${wheel_radius + body_height + 0.01}" rpy="0 0 0"/>
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0002"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 ${body_width/2 + wheel_width/2} ${wheel_radius}" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0002"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -${body_width/2 + wheel_width/2} ${wheel_radius}" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster Wheel (front) -->
  <link name="caster_wheel">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="${body_length/2 - 0.03} 0 0.02" rpy="0 0 0"/>
  </joint>

  <!-- Differential Drive Plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=wheel_odom</remapping>
      </ros>

      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>

      <wheel_separation>${body_width + wheel_width}</wheel_separation>
      <wheel_diameter>${wheel_radius * 2}</wheel_diameter>

      <max_wheel_torque>5</max_wheel_torque>
      <max_wheel_acceleration>2.0</max_wheel_acceleration>

      <publish_odom>false</publish_odom> <!-- We use marker tracking for odom -->
      <publish_odom_tf>false</publish_odom_tf>

      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

### 5.4 Random Obstacle Generator

```python
#!/usr/bin/env python3
"""
Generate random obstacles in Gazebo world.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import random
import xml.etree.ElementTree as ET


class ObstacleGenerator(Node):
    def __init__(self):
        super().__init__('obstacle_generator')

        # Parameters
        self.declare_parameter('num_obstacles', 8)
        self.declare_parameter('room_length', 6.0)
        self.declare_parameter('room_width', 4.0)
        self.declare_parameter('min_obstacle_size', 0.2)
        self.declare_parameter('max_obstacle_size', 0.5)
        self.declare_parameter('robot_clearance', 0.5)  # Don't spawn near robot start

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Wait for Gazebo
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo spawn service...')

        self.spawn_obstacles()

    def spawn_obstacles(self):
        """Spawn random obstacles in the room."""
        num_obs = self.get_parameter('num_obstacles').value
        room_l = self.get_parameter('room_length').value
        room_w = self.get_parameter('room_width').value
        min_size = self.get_parameter('min_obstacle_size').value
        max_size = self.get_parameter('max_obstacle_size').value
        clearance = self.get_parameter('robot_clearance').value

        for i in range(num_obs):
            # Random position (avoiding room edges and robot start position at origin)
            while True:
                x = random.uniform(-room_l/2 + 0.5, room_l/2 - 0.5)
                y = random.uniform(-room_w/2 + 0.5, room_w/2 - 0.5)

                # Check clearance from origin (robot start)
                if (x**2 + y**2) > clearance**2:
                    break

            # Random size
            size_x = random.uniform(min_size, max_size)
            size_y = random.uniform(min_size, max_size)
            height = random.uniform(0.3, 0.8)

            # Random color
            r = random.uniform(0.3, 0.8)
            g = random.uniform(0.3, 0.8)
            b = random.uniform(0.3, 0.8)

            # Generate SDF
            sdf = self.generate_obstacle_sdf(f'obstacle_{i}', size_x, size_y, height, r, g, b)

            # Spawn
            request = SpawnEntity.Request()
            request.name = f'obstacle_{i}'
            request.xml = sdf
            request.initial_pose.position.x = x
            request.initial_pose.position.y = y
            request.initial_pose.position.z = height / 2

            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info(f'Spawned obstacle_{i} at ({x:.2f}, {y:.2f})')
            else:
                self.get_logger().warn(f'Failed to spawn obstacle_{i}')

    def generate_obstacle_sdf(self, name, size_x, size_y, height, r, g, b):
        """Generate SDF string for a box obstacle."""
        return f'''
        <?xml version="1.0"?>
        <sdf version="1.7">
          <model name="{name}">
            <static>true</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box><size>{size_x} {size_y} {height}</size></box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box><size>{size_x} {size_y} {height}</size></box>
                </geometry>
                <material>
                  <ambient>{r} {g} {b} 1</ambient>
                  <diffuse>{r} {g} {b} 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        '''


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGenerator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.5 PHASE 3 GATE CHECKPOINT

**Before proceeding to Phase 4, ALL of the following must pass:**

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                    PHASE 3 GATE CHECKLIST                                  ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃  [ ] Gazebo World Loads                                                    ┃
┃      □ Room with 4 walls and ceiling spawns correctly                      ┃
┃      □ All 4 cameras visible in simulation                                 ┃
┃      □ Random obstacles spawn without collision errors                     ┃
┃                                                                            ┃
┃  [ ] Robot Spawns and Functions                                            ┃
┃      □ Robot model spawns at origin                                        ┃
┃      □ ArUco marker visible on robot top                                   ┃
┃      □ Robot responds to /cmd_vel commands (teleop test)                   ┃
┃      □ Robot wheels move correctly (differential drive)                    ┃
┃                                                                            ┃
┃  [ ] Camera Functionality                                                  ┃
┃      □ All 4 cameras publishing RGB images                                 ┃
┃      □ All 4 cameras publishing depth images                               ┃
┃      □ All 4 cameras publishing point clouds                               ┃
┃      □ Marker visible in at least 2 camera views at any position           ┃
┃                                                                            ┃
┃  [ ] Marker Detection in Simulation                                        ┃
┃      □ ArUco marker detected in simulated camera images                    ┃
┃      □ Detected position matches Gazebo ground truth (within 5cm)          ┃
┃      □ Detection works as robot moves around                               ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

**Validation Commands to Run:**

```bash
# Launch Gazebo simulation
ros2 launch multi_camera_robot_nav simulation.launch.py

# Verify cameras publishing (in separate terminals)
ros2 topic hz /cameras/camera_1/color/image_raw  # Should be ~30 Hz
ros2 topic hz /cameras/camera_1/depth/points     # Should be ~15 Hz

# Test robot teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot/cmd_vel

# Compare tracked pose vs ground truth
ros2 topic echo /robot/pose &
ros2 service call /gazebo/get_model_state gazebo_msgs/GetModelState "{model_name: 'tracking_robot'}"

# Positions should match within 5cm
# If ANY check fails, DO NOT proceed to Phase 4
```

**Ground Truth Validation Script:**

```python
#!/usr/bin/env python3
"""
Validate tracked pose against Gazebo ground truth.
Run this with simulation running.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
import numpy as np

class GroundTruthValidator(Node):
    def __init__(self):
        super().__init__('ground_truth_validator')
        self.tracked_pose = None

        self.create_subscription(PoseStamped, '/robot/pose', self.pose_callback, 10)
        self.gazebo_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        self.create_timer(1.0, self.validate)

    def pose_callback(self, msg):
        self.tracked_pose = msg

    def validate(self):
        if self.tracked_pose is None:
            self.get_logger().warn('No tracked pose received')
            return

        # Get ground truth from Gazebo
        req = GetModelState.Request()
        req.model_name = 'tracking_robot'
        future = self.gazebo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        gt = future.result()
        tracked = self.tracked_pose.pose.position

        error = np.sqrt(
            (gt.pose.position.x - tracked.x)**2 +
            (gt.pose.position.y - tracked.y)**2 +
            (gt.pose.position.z - tracked.z)**2
        )

        status = "PASS ✓" if error < 0.05 else "FAIL ✗"
        self.get_logger().info(f'Position error: {error:.3f}m [{status}]')
```

**If gate fails:** Fix Gazebo models, camera configs, or marker detection, re-run validation.

---

## 6. Phase 4: Point Cloud Obstacle Mapping

> **IMPORTANT**: Only start this phase after Phase 3 gate is passed.
> This phase creates the obstacle map from depth camera point clouds.

### 6.1 Point Cloud Stitching Node

```python
#!/usr/bin/env python3
"""
ROS2 Node: Point Cloud Stitcher

Combines point clouds from multiple depth cameras into a single
unified obstacle map in world coordinates.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from tf2_ros import Buffer, TransformListener
import open3d as o3d


class PointCloudStitcherNode(Node):
    def __init__(self):
        super().__init__('point_cloud_stitcher_node')

        # Parameters
        self.declare_parameter('voxel_size', 0.02)  # 2cm voxel for downsampling
        self.declare_parameter('floor_height', 0.0)
        self.declare_parameter('ceiling_height', 2.0)
        self.declare_parameter('robot_height', 0.2)
        self.declare_parameter('update_rate', 5.0)  # Hz

        # TF buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Point cloud storage
        self.camera_clouds = {
            'camera_1': None,
            'camera_2': None,
            'camera_3': None,
            'camera_4': None
        }

        # Subscribers for each camera
        for cam_id in self.camera_clouds.keys():
            self.create_subscription(
                PointCloud2,
                f'/cameras/{cam_id}/depth/points',
                lambda msg, cid=cam_id: self.cloud_callback(msg, cid),
                10
            )

        # Publishers
        self.stitched_pub = self.create_publisher(
            PointCloud2, '/map/obstacle_cloud', 10
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/map/occupancy_grid', 10
        )

        # Timer for periodic stitching
        rate = self.get_parameter('update_rate').value
        self.create_timer(1.0 / rate, self.stitch_and_publish)

        self.get_logger().info('Point Cloud Stitcher initialized')

    def cloud_callback(self, msg: PointCloud2, camera_id: str):
        """Store incoming point cloud from a camera."""
        try:
            # Transform to world frame
            transform = self.tf_buffer.lookup_transform(
                'world',
                msg.header.frame_id,
                rclpy.time.Time()
            )

            # Convert to numpy
            points = self.pointcloud2_to_numpy(msg)

            # Apply transform
            points_world = self.transform_points(points, transform)

            self.camera_clouds[camera_id] = points_world

        except Exception as e:
            self.get_logger().debug(f'Transform error for {camera_id}: {e}')

    def pointcloud2_to_numpy(self, cloud_msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 to numpy array."""
        points = []
        for p in pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        return np.array(points)

    def transform_points(self, points: np.ndarray, transform) -> np.ndarray:
        """Apply TF transform to points."""
        # Extract rotation and translation
        t = transform.transform.translation
        r = transform.transform.rotation

        # Quaternion to rotation matrix
        R = self.quat_to_rotation_matrix(r.w, r.x, r.y, r.z)
        translation = np.array([t.x, t.y, t.z])

        # Apply transform
        return (R @ points.T).T + translation

    def stitch_and_publish(self):
        """Combine all camera point clouds and publish."""
        # Collect all valid clouds
        all_points = []
        for cam_id, points in self.camera_clouds.items():
            if points is not None and len(points) > 0:
                all_points.append(points)

        if not all_points:
            return

        # Concatenate
        combined = np.vstack(all_points)

        # Filter by height (remove floor and ceiling)
        floor_h = self.get_parameter('floor_height').value
        ceiling_h = self.get_parameter('ceiling_height').value
        robot_h = self.get_parameter('robot_height').value

        # Keep only obstacle-relevant heights
        mask = (combined[:, 2] > floor_h + 0.05) & (combined[:, 2] < robot_h + 0.1)
        obstacles = combined[mask]

        # Voxel downsampling using Open3D
        voxel_size = self.get_parameter('voxel_size').value
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(obstacles)
        pcd_down = pcd.voxel_down_sample(voxel_size)
        obstacles_filtered = np.asarray(pcd_down.points)

        # Publish stitched cloud
        self.publish_point_cloud(obstacles_filtered)

        # Generate and publish occupancy grid
        self.publish_occupancy_grid(obstacles_filtered)

    def publish_point_cloud(self, points: np.ndarray):
        """Publish stitched point cloud."""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # Create fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()

        self.stitched_pub.publish(msg)

    def publish_occupancy_grid(self, points: np.ndarray):
        """Generate 2D occupancy grid from 3D points."""
        if len(points) == 0:
            return

        # Grid parameters
        resolution = 0.05  # 5cm per cell

        # Compute grid bounds
        min_x, max_x = -3.5, 3.5  # Room bounds + margin
        min_y, max_y = -2.5, 2.5

        width = int((max_x - min_x) / resolution)
        height = int((max_y - min_y) / resolution)

        # Initialize grid (unknown = -1, free = 0, occupied = 100)
        grid = np.zeros((height, width), dtype=np.int8)

        # Project points to 2D grid
        for p in points:
            gx = int((p[0] - min_x) / resolution)
            gy = int((p[1] - min_y) / resolution)

            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = 100  # Occupied

        # Create message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = min_x
        msg.info.origin.position.y = min_y
        msg.info.origin.position.z = 0.0
        msg.data = grid.flatten().tolist()

        self.grid_pub.publish(msg)

    @staticmethod
    def quat_to_rotation_matrix(w, x, y, z):
        """Convert quaternion to 3x3 rotation matrix."""
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudStitcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.2 PHASE 4 GATE CHECKPOINT

**Before proceeding to Phase 5, ALL of the following must pass:**

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                    PHASE 4 GATE CHECKLIST                                  ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃  [ ] Point Cloud Stitching                                                 ┃
┃      □ /map/obstacle_cloud topic publishing                                ┃
┃      □ Point clouds from all 4 cameras are being combined                  ┃
┃      □ Points are correctly transformed to world frame                     ┃
┃      □ Voxel downsampling working (cloud is not too dense)                 ┃
┃                                                                            ┃
┃  [ ] Obstacle Detection                                                    ┃
┃      □ Spawned obstacles visible in stitched point cloud                   ┃
┃      □ Walls visible in point cloud                                        ┃
┃      □ Floor and ceiling filtered out (only obstacles remain)              ┃
┃                                                                            ┃
┃  [ ] Occupancy Grid                                                        ┃
┃      □ /map/occupancy_grid topic publishing                                ┃
┃      □ Grid shows obstacles as occupied cells                              ┃
┃      □ Grid resolution appropriate (5cm recommended)                       ┃
┃      □ Grid updates as robot moves (if obstacles change)                   ┃
┃                                                                            ┃
┃  [ ] RViz Visualization                                                    ┃
┃      □ Obstacle point cloud visible in RViz (red points)                   ┃
┃      □ Occupancy grid visible as 2D map overlay                            ┃
┃      □ Robot position correct relative to obstacles                        ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

**Validation Commands to Run:**

```bash
# With simulation already running:

# Check point cloud publishing
ros2 topic hz /map/obstacle_cloud      # Should be ~5 Hz
ros2 topic echo /map/obstacle_cloud --once | head -20

# Check occupancy grid
ros2 topic hz /map/occupancy_grid      # Should be ~1 Hz
ros2 topic echo /map/occupancy_grid --once | head -20

# Visualize in RViz
# - Add PointCloud2 display for /map/obstacle_cloud
# - Add Map display for /map/occupancy_grid
# - Verify obstacles visible and match Gazebo positions

# If ANY check fails, DO NOT proceed to Phase 5
```

**Obstacle Position Validation:**

```python
#!/usr/bin/env python3
"""
Validate that obstacles in point cloud match Gazebo ground truth.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from gazebo_msgs.srv import GetModelState
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class ObstacleValidator(Node):
    def __init__(self):
        super().__init__('obstacle_validator')

        self.create_subscription(
            PointCloud2, '/map/obstacle_cloud', self.cloud_callback, 10
        )
        self.gazebo_client = self.create_client(GetModelState, '/gazebo/get_model_state')

    def cloud_callback(self, msg):
        # Get obstacle positions from Gazebo
        obstacle_positions = []
        for i in range(8):  # Assuming 8 obstacles
            req = GetModelState.Request()
            req.model_name = f'obstacle_{i}'
            future = self.gazebo_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                pos = future.result().pose.position
                obstacle_positions.append([pos.x, pos.y])

        # Check if point cloud has points near each obstacle
        points = np.array([[p[0], p[1]] for p in
                          pc2.read_points(msg, field_names=('x', 'y'), skip_nans=True)])

        for obs_pos in obstacle_positions:
            distances = np.linalg.norm(points - obs_pos, axis=1)
            min_dist = np.min(distances) if len(distances) > 0 else float('inf')

            status = "PASS ✓" if min_dist < 0.3 else "FAIL ✗"
            self.get_logger().info(
                f'Obstacle at ({obs_pos[0]:.2f}, {obs_pos[1]:.2f}): '
                f'nearest point {min_dist:.2f}m [{status}]'
            )
```

**If gate fails:** Fix point cloud stitching, transforms, or filtering, re-run validation.

---

## 7. Phase 5: Robot Navigation

> **IMPORTANT**: Only start this phase after Phase 4 gate is passed.
> This phase configures Nav2 for autonomous navigation.

### 7.1 Nav2 Configuration

```yaml
# config/nav2_params.yaml

amcl:
  ros__parameters:
    use_sim_time: true
    # AMCL is NOT used - we use external marker tracking for localization

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: world
    robot_base_frame: base_link
    odom_topic: /robot/odom
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.1
      yaw_goal_tolerance: 0.1
      stateful: True

    # DWB Local Planner
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.3
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.3
      min_speed_theta: 0.0
      acc_lim_x: 1.0
      acc_lim_y: 0.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.0
      decel_lim_y: 0.0
      decel_lim_theta: -2.0
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.5
      xy_goal_tolerance: 0.1
      trans_stopped_velocity: 0.1
      short_circuit_trajectory_evaluation: True

planner_server:
  ros__parameters:
    use_sim_time: true
    expected_planner_frequency: 20.0

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      footprint: "[[0.15, 0.1], [0.15, -0.1], [-0.15, -0.1], [-0.15, 0.1]]"

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: point_cloud
        point_cloud:
          topic: /map/obstacle_cloud
          sensor_frame: world
          observation_persistence: 0.0
          expected_update_rate: 5.0
          data_type: "PointCloud2"
          clearing: True
          marking: True
          max_obstacle_height: 2.0
          min_obstacle_height: 0.1
          obstacle_max_range: 5.0
          obstacle_min_range: 0.0
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.3

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: world
      robot_base_frame: base_link
      update_frequency: 1.0
      publish_frequency: 1.0
      width: 8
      height: 6
      origin_x: -4.0
      origin_y: -3.0
      resolution: 0.05
      track_unknown_space: true
      footprint: "[[0.15, 0.1], [0.15, -0.1], [-0.15, -0.1], [-0.15, 0.1]]"

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: point_cloud
        point_cloud:
          topic: /map/obstacle_cloud
          sensor_frame: world
          observation_persistence: 0.0
          expected_update_rate: 5.0
          data_type: "PointCloud2"
          clearing: True
          marking: True

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.3
```

### 7.2 Custom Localization (Marker-Based)

Since we're using external marker tracking instead of AMCL, we need a custom localization node that publishes the required transforms:

```python
#!/usr/bin/env python3
"""
ROS2 Node: Marker-Based Localization

Replaces AMCL with external marker tracking for robot localization.
Publishes map -> odom transform based on marker detections.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np


class MarkerLocalizationNode(Node):
    def __init__(self):
        super().__init__('marker_localization_node')

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transform: world -> map (identity)
        self.publish_static_world_map_tf()

        # Subscribe to marker tracking pose
        self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )

        # Timer to publish map -> odom transform
        self.create_timer(0.02, self.publish_map_odom_tf)  # 50 Hz

        # Current pose
        self.current_pose = None

        self.get_logger().info('Marker Localization Node initialized')

    def publish_static_world_map_tf(self):
        """Publish world -> map as identity (they are the same)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def pose_callback(self, msg: PoseStamped):
        """Store current robot pose from marker tracking."""
        self.current_pose = msg

    def publish_map_odom_tf(self):
        """
        Publish map -> odom transform.

        In our case, since marker tracking gives us ground truth in world frame,
        and we define world = map = odom, this is identity.

        For a more sophisticated system, this would account for drift
        between odometry and marker-based localization.
        """
        if self.current_pose is None:
            return

        # For now, map = odom (no drift correction needed with perfect marker tracking)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0  # Identity

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 7.3 Launch File

```python
#!/usr/bin/env python3
"""
Launch file for complete simulation with navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('multi_camera_robot_nav')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': os.path.join(pkg_share, 'worlds', 'tracking_room.world'),
            'verbose': 'true'
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tracking_robot',
            '-file', os.path.join(pkg_share, 'urdf', 'robot.urdf'),
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(os.path.join(pkg_share, 'urdf', 'robot.urdf')).read(),
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn random obstacles (delayed to let Gazebo start)
    spawn_obstacles = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='multi_camera_robot_nav',
                executable='obstacle_generator',
                parameters=[{'num_obstacles': 8}],
                output='screen'
            )
        ]
    )

    # Marker tracker node
    marker_tracker = Node(
        package='multi_camera_robot_nav',
        executable='marker_tracker_node',
        parameters=[{
            'marker_size': 0.1,
            'target_marker_id': 0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Point cloud stitcher
    cloud_stitcher = Node(
        package='multi_camera_robot_nav',
        executable='point_cloud_stitcher_node',
        parameters=[{
            'voxel_size': 0.02,
            'update_rate': 5.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Marker localization
    marker_localization = Node(
        package='multi_camera_robot_nav',
        executable='marker_localization_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Nav2 stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        }.items()
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_world,
        spawn_robot,
        robot_state_publisher,
        spawn_obstacles,
        marker_tracker,
        cloud_stitcher,
        marker_localization,
        TimerAction(period=10.0, actions=[nav2_bringup]),
        TimerAction(period=12.0, actions=[rviz])
    ])
```

### 7.4 PHASE 5 FINAL GATE CHECKPOINT

**This is the final validation - the complete system test:**

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                    PHASE 5 GATE CHECKLIST (FINAL)                          ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃  [ ] Nav2 Stack Running                                                    ┃
┃      □ All Nav2 nodes start without errors                                 ┃
┃      □ Costmaps visualized correctly in RViz                               ┃
┃      □ Planner server responding                                           ┃
┃      □ Controller server responding                                        ┃
┃                                                                            ┃
┃  [ ] Goal Navigation - Test 1: Simple Path                                 ┃
┃      □ Send goal 1m away with clear path                                   ┃
┃      □ Robot plans path within 2 seconds                                   ┃
┃      □ Robot follows path smoothly                                         ┃
┃      □ Robot reaches goal within 10cm accuracy                             ┃
┃      □ Robot orientation within 5° of goal                                 ┃
┃                                                                            ┃
┃  [ ] Goal Navigation - Test 2: Obstacle Avoidance                          ┃
┃      □ Send goal with obstacle between robot and goal                      ┃
┃      □ Path planned around obstacle                                        ┃
┃      □ Robot navigates without collision                                   ┃
┃      □ Robot reaches goal successfully                                     ┃
┃                                                                            ┃
┃  [ ] Goal Navigation - Test 3: Multiple Waypoints                          ┃
┃      □ Send sequence of 3 goals                                            ┃
┃      □ Robot visits all waypoints in order                                 ┃
┃      □ No collisions during entire sequence                                ┃
┃                                                                            ┃
┃  [ ] Robustness Tests                                                      ┃
┃      □ Recovery behavior when path blocked                                 ┃
┃      □ System handles temporary marker occlusion (<1s)                     ┃
┃      □ Robot stops safely if tracking lost (>2s)                           ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

**Navigation Test Commands:**

```bash
# Launch full system
ros2 launch multi_camera_robot_nav navigation.launch.py

# Test 1: Simple goal (clear path)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Wait for completion, verify robot reached goal

# Test 2: Goal behind obstacle
# First, note obstacle positions from Gazebo
# Send goal that requires path around obstacle
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'world'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Test 3: Waypoint sequence
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: 'world'}, pose: {position: {x: 1.5, y: -0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}
  ]}"
```

**Automated Navigation Validation Script:**

```python
#!/usr/bin/env python3
"""
Automated navigation validation with ground truth comparison.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

class NavigationValidator(Node):
    def __init__(self):
        super().__init__('navigation_validator')

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.gazebo_client = self.create_client(GetModelState, '/gazebo/get_model_state')

        self.test_goals = [
            {'x': 1.0, 'y': 0.0, 'name': 'Simple forward'},
            {'x': 1.5, 'y': 1.0, 'name': 'Diagonal with potential obstacle'},
            {'x': -1.0, 'y': 0.5, 'name': 'Reverse direction'},
        ]

    def run_tests(self):
        results = []

        for goal in self.test_goals:
            self.get_logger().info(f'Testing: {goal["name"]}')

            # Send navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'world'
            goal_msg.pose.pose.position.x = goal['x']
            goal_msg.pose.pose.position.y = goal['y']
            goal_msg.pose.pose.orientation.w = 1.0

            self.nav_client.wait_for_server()
            future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()
            if not goal_handle.accepted:
                results.append({'name': goal['name'], 'status': 'FAIL', 'reason': 'Goal rejected'})
                continue

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

            # Check final position against ground truth
            req = GetModelState.Request()
            req.model_name = 'tracking_robot'
            gt_future = self.gazebo_client.call_async(req)
            rclpy.spin_until_future_complete(self, gt_future)

            gt_pos = gt_future.result().pose.position
            error = np.sqrt((gt_pos.x - goal['x'])**2 + (gt_pos.y - goal['y'])**2)

            if error < 0.10:
                results.append({'name': goal['name'], 'status': 'PASS', 'error': error})
            else:
                results.append({'name': goal['name'], 'status': 'FAIL', 'error': error})

        # Print summary
        print('\n' + '='*60)
        print('NAVIGATION VALIDATION SUMMARY')
        print('='*60)
        for r in results:
            status = "✓ PASS" if r['status'] == 'PASS' else "✗ FAIL"
            error_str = f"error: {r.get('error', 'N/A'):.3f}m" if 'error' in r else r.get('reason', '')
            print(f"{r['name']}: [{status}] {error_str}")

        all_passed = all(r['status'] == 'PASS' for r in results)
        print('='*60)
        print(f"OVERALL: {'ALL TESTS PASSED ✓' if all_passed else 'SOME TESTS FAILED ✗'}")
        print('='*60)

        return all_passed
```

**SUCCESS CRITERIA:**
- All 3 navigation tests pass (goal reached within 10cm)
- No collisions during any test
- Robot tracks smoothly (no erratic movements)
- System recovers from temporary occlusions

**If tests fail:** Debug Nav2 configuration, costmaps, or localization integration.

---

## 8. Technical Requirements

### 8.1 Software Dependencies

```yaml
# ROS2 packages
ros2_packages:
  - ros-humble-desktop  # Full ROS2 Humble desktop
  - ros-humble-gazebo-ros-pkgs
  - ros-humble-nav2-bringup
  - ros-humble-navigation2
  - ros-humble-robot-state-publisher
  - ros-humble-joint-state-publisher
  - ros-humble-tf2-ros
  - ros-humble-cv-bridge
  - ros-humble-image-transport

# Python packages
python_packages:
  - opencv-python>=4.5
  - numpy>=1.20
  - open3d>=0.15
  - scipy>=1.7
  - transforms3d>=0.3.1

# System packages
system_packages:
  - gazebo11
  - libgazebo11-dev
```

### 8.2 Hardware Requirements (for real deployment)

```yaml
cameras:
  model: "Orbbec Astra / Femto"
  quantity: 4
  mounting: "Ceiling corners, 45° downward pitch"
  connection: "USB 3.0 or Ethernet (Femto)"

compute:
  option_1:
    device: "Single powerful PC"
    cpu: "Intel i7/i9 or AMD Ryzen 7/9"
    gpu: "NVIDIA RTX 3060 or better"
    ram: "32GB"
    note: "Runs all processing centrally"

  option_2:
    device: "Distributed Jetson setup"
    edge_devices: "4x NVIDIA Jetson Orin Nano (one per camera)"
    central_server: "PC for fusion and navigation"

robot:
  type: "Differential drive mobile base"
  marker: "10cm ArUco marker (DICT_4X4_250, ID 0)"
  communication: "WiFi to central server"
```

---

## 9. Validation Criteria

### 9.1 Tracking Accuracy

| Metric | Target | Test Method |
|--------|--------|-------------|
| Position accuracy | < 3cm at 2m distance | Compare to tape measurement |
| Position repeatability | < 1cm std dev | 100 samples at fixed position |
| Orientation accuracy | < 3° for yaw | Rotatable platform with angle marks |
| Velocity accuracy | < 5% error | Controlled speed tests |
| Update rate | > 25 Hz | Timestamp analysis |
| Latency | < 100ms end-to-end | Timestamp comparison |

### 9.2 Navigation Performance

| Metric | Target | Test Method |
|--------|--------|-------------|
| Path following | < 10cm deviation | Predefined path test |
| Obstacle avoidance | 100% success | Random obstacle course |
| Goal reaching | < 5cm position, < 5° heading | Repeated navigation tests |
| Recovery behavior | Successful recovery from stuck | Intentional blocking test |

### 9.3 System Robustness

| Scenario | Expected Behavior |
|----------|-------------------|
| Single camera failure | Continue with 3 cameras |
| Two camera failure | Continue with reduced accuracy |
| Temporary marker occlusion (< 1s) | Predict using velocity |
| Network latency spike | Buffer and interpolate |
| Lighting change | Maintain detection |

---

## 10. File Structure

```
multi_camera_robot_nav/
├── package.xml
├── setup.py
├── CMakeLists.txt
│
├── config/
│   ├── camera_calibration/
│   │   ├── camera_1.yaml
│   │   ├── camera_2.yaml
│   │   ├── camera_3.yaml
│   │   └── camera_4.yaml
│   ├── tracker_params.yaml
│   ├── nav2_params.yaml
│   └── rviz_config.yaml
│
├── launch/
│   ├── simulation.launch.py      # Gazebo + everything
│   ├── real_robot.launch.py      # Real hardware
│   ├── tracking_only.launch.py   # Just tracking (for validation)
│   └── navigation.launch.py      # Nav2 stack
│
├── multi_camera_robot_nav/
│   ├── __init__.py
│   ├── marker_tracker_node.py
│   ├── velocity_estimator.py
│   ├── pose_fusion.py
│   ├── point_cloud_stitcher_node.py
│   ├── marker_localization_node.py
│   └── obstacle_generator.py
│
├── validation/
│   ├── distance_validator.py
│   ├── orientation_validator.py
│   ├── velocity_validator.py
│   └── fusion_validator.py
│
├── worlds/
│   └── tracking_room.world
│
├── models/
│   ├── depth_camera/
│   │   ├── model.config
│   │   └── model.sdf
│   └── aruco_marker/
│       ├── model.config
│       └── model.sdf
│
├── urdf/
│   ├── robot.urdf.xacro
│   └── sensors.xacro
│
├── rviz/
│   └── robot_nav.rviz
│
├── msg/
│   ├── MarkerDetection.msg
│   └── TrackedRobot.msg
│
├── srv/
│   └── GetRobotPose.srv
│
└── test/
    ├── test_tracking.py
    ├── test_fusion.py
    └── test_navigation.py
```

---

## Summary

This document provides a complete specification for building a robot navigation system using external marker tracking. The implementation phases are:

1. **Phase 1**: Validate existing tracking code for distance, orientation, and velocity accuracy
2. **Phase 2**: Integrate with ROS2, publishing poses and odometry
3. **Phase 3**: Create Gazebo simulation with 4 ceiling-mounted depth cameras
4. **Phase 4**: Implement point cloud stitching for obstacle mapping
5. **Phase 5**: Configure Nav2 for autonomous navigation

**Key Innovation**: The robot has NO onboard sensors. All localization and mapping comes from external infrastructure, enabling simpler robot hardware and potentially better accuracy through multiple viewpoints.

---

*Document Version: 1.0*
*Created for: Multi-Camera Marker Tracking Robot Navigation Project*
