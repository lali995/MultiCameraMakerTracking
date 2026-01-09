# MultiCameraMakerTracking - Branch Documentation

This document provides detailed documentation for all branches in the repository, describing the work completed, features implemented, and the purpose of each branch.

---

## Table of Contents

1. [Repository Overview](#repository-overview)
2. [Branch: master](#branch-master)
3. [Branch: feature/robot-navigation-ros2](#branch-featurerobot-navigation-ros2)
4. [Branch Comparison](#branch-comparison)

---

## Repository Overview

**Project Name**: MultiCameraMakerTracking

**Description**: A comprehensive multi-camera ArUco marker tracking system designed for robot localization and navigation. The system uses ceiling-mounted depth cameras to track ArUco markers attached to robots, providing external localization without requiring onboard sensors.

**Key Technologies**:
- Python 3.x
- OpenCV (ArUco detection)
- Open3D (3D visualization)
- ZMQ (network streaming)
- ROS2 Humble (robot integration)
- Nav2 (robot navigation)
- Gazebo (simulation)

---

## Branch: master

### Overview

The `master` branch contains the core multi-camera marker tracking system with real-time 3D visualization. This branch provides all the foundational functionality for ArUco marker detection, pose estimation, multi-camera fusion, and network streaming capabilities.

### Commit History

```
27ea1e5 Stream client modified
4da8e26 Add technical documentation and fix visualization issues
dac0ca8 Revert experimental GStreamer changes in stream client
0fd1067 Update documentation and add TCP transport for RTSP streams
ba28150 Fix UI blocking in integrated viewer
18a225c Add multi-camera tracking with pose fusion and integrated viewer
237f424 Add video tracker scripts and sprint demo summary
bc76eac Add network tracker with 3D visualization and video preview
b4a328a Add calibration file support and remove ground grid from visualization
56ea55d Add network camera streaming for remote ArUco marker tracking
9e6ca8a Add Orbbec camera support and ArUco marker generation
f1e6139 Add real-time ArUco marker tracking system with Open3D visualization
aa6e96f Add README with usage instructions
a70f8b7 Add camera folder name to hover info and origin marker
14149d0 Add interactive camera pose visualization
```

### Features

#### 1. ArUco Marker Detection
- **Real-time detection** using OpenCV's ArUco module
- **Multiple dictionary support** (DICT_4X4_50, DICT_4X4_100, etc.)
- **Pose estimation** using solvePnP algorithm
- **Configurable marker sizes** for accurate distance estimation

#### 2. Multi-Camera Support
- **Orbbec RGB-D camera integration** via pyorbbecsdk
- **Network camera streaming** using ZMQ PUSH/PULL sockets
- **Camera calibration file support** with extrinsic matrices
- **Reference camera selection** for coordinate origin

#### 3. Pose Fusion
Three algorithms for combining observations from multiple cameras:
| Method | Description | Use Case |
|--------|-------------|----------|
| **Weighted Average** | Weights by confidence and inverse depth | General purpose |
| **Median** | Component-wise median position | Outlier rejection |
| **Least Squares** | Minimizes reprojection error | High accuracy |

#### 4. 3D Visualization
- **Open3D-based viewer** with interactive controls
- **Camera frustum visualization** showing camera positions and orientations
- **Marker spheres** with unique colors per marker ID
- **Orientation axes** (XYZ) on each marker
- **Detection lines** connecting markers to detecting cameras
- **Trajectory trails** showing marker movement history
- **Integrated viewer** with 3D scene + camera feed panel

#### 5. Network Streaming
- **ZMQ-based transport** for low-latency streaming
- **JPEG compression** for bandwidth efficiency
- **RTSP input support** via FFmpeg backend
- **TCP transport option** for reliable delivery
- **Standalone stream client** for remote deployment

### Project Structure

```
MultiCameraMakerTracking/
├── src/
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config_loader.py      # Camera configuration parsing
│   │   ├── data_types.py         # MarkerPose, CameraPose, FusedMarkerPose
│   │   ├── message_bus.py        # Pub/sub message bus (ROS-compatible)
│   │   └── timing.py             # Performance timing utilities
│   ├── markers/
│   │   ├── __init__.py
│   │   └── marker_generator.py   # ArUco marker generation
│   ├── streaming/
│   │   ├── __init__.py
│   │   ├── stream_client.py      # ZMQ frame sender
│   │   └── stream_server.py      # ZMQ frame receiver
│   ├── tracking/
│   │   ├── __init__.py
│   │   ├── tracker_base.py       # Abstract tracker interface
│   │   ├── simulated_tracker.py  # Simulated markers for testing
│   │   ├── orbbec_tracker.py     # Orbbec camera tracker
│   │   ├── network_stream_tracker.py  # Network camera tracker
│   │   ├── multi_camera_tracker.py    # Multi-camera with fusion
│   │   └── pose_fusion.py        # Pose fusion algorithms
│   └── visualization/
│       ├── __init__.py
│       ├── viewer.py             # Main 3D viewer
│       ├── scene_manager.py      # Scene geometry management
│       ├── geometry_factory.py   # 3D primitive creation
│       ├── integrated_viewer.py  # 3D + camera panel viewer
│       └── multi_camera_preview.py  # Camera grid preview
├── scripts/
│   ├── generate_markers.py       # Create printable ArUco markers
│   ├── run_full_system.py        # Simulated tracking demo
│   ├── run_orbbec_tracker.py     # Single Orbbec camera tracking
│   ├── run_network_tracker.py    # Network camera tracking
│   ├── run_network_tracker_with_visualization.py  # Network + 3D viz
│   ├── run_multi_camera_tracker.py  # Multi-camera fusion tracking
│   ├── run_video_tracker.py      # Video file processing
│   ├── run_video_with_visualization.py  # Video + 3D viz
│   ├── run_camera_preview.py     # Camera preview window
│   ├── start_multi_camera_demo.py  # Demo launcher
│   └── stream_client_standalone.py  # Deployable stream client
├── docs/
│   ├── CLIENT_DEVICE_SETUP.md    # Remote device setup guide
│   ├── NETWORK_STREAMING.md      # Network streaming documentation
│   ├── SPRINT_DEMO_SUMMARY.md    # Sprint demo documentation
│   └── TECHNICAL_PRESENTATION.md # Full technical documentation
├── calibration/
│   └── calibration_result.json   # Camera extrinsic calibration
├── config/
│   └── SZVIDS-*/                 # Camera-specific configurations
├── markers/
│   └── marker_*.png              # Generated marker images
├── main.py                       # Legacy Plotly visualization
└── requirements.txt
```

### Usage

#### Generate Markers
```bash
python scripts/generate_markers.py --id 0 1 2 3 --size-mm 50 --dpi 300
```

#### Single Camera Tracking
```bash
python scripts/run_orbbec_tracker.py --marker-size 0.05 --preview
```

#### Multi-Camera Tracking
```bash
# On remote cameras:
python stream_client.py --server 192.168.1.195 --camera-id cam1

# On server:
python scripts/run_multi_camera_tracker.py \
    --calibration calibration/calibration_result.json \
    --reference 250514 --preview
```

#### Simulated Demo
```bash
python scripts/run_full_system.py --num-markers 5 --motion figure8
```

### Architecture

```
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  Camera 1   │  │  Camera 2   │  │  Camera 3   │
│ (streaming) │  │ (streaming) │  │ (streaming) │
└──────┬──────┘  └──────┬──────┘  └──────┬──────┘
       │                │                │
       └────────────────┼────────────────┘
                        ▼
              ┌─────────────────┐
              │  StreamServer   │
              │ (ZMQ receiver)  │
              └────────┬────────┘
                       ▼
              ┌─────────────────┐
              │MultiCameraTracker│
              │ - ArUco detect  │
              │ - Pose estimate │
              └────────┬────────┘
                       ▼
              ┌─────────────────┐
              │   PoseFusion    │
              │ - Weighted avg  │
              │ - Multi-view    │
              └────────┬────────┘
                       ▼
              ┌─────────────────┐
              │   MessageBus    │
              │/markers/fused   │
              └────────┬────────┘
                       ▼
              ┌─────────────────┐
              │  MarkerViewer   │
              │ - 3D rendering  │
              └─────────────────┘
```

---

## Branch: feature/robot-navigation-ros2

### Overview

The `feature/robot-navigation-ros2` branch extends the master branch with full ROS2 integration and robot navigation capabilities. This branch implements a complete robot navigation system using the Nav2 stack, where the robot has NO onboard sensors and relies entirely on external camera-based localization.

### Commit History (on top of master)

```
924e187 launcher files created
0f408db Add Phase 5: Robot Navigation with Nav2 (FINAL)
c4cb051 Add Phase 4: Point Cloud Obstacle Mapping
6f72638 Add Phase 3: Gazebo Simulation Environment
032cb43 Add Phase 2: ROS2 Integration
0e38113 Add Phase 1 validation infrastructure
c48b9d5 Add robot navigation prompt document with validation gates
```

### Development Phases

The development followed a strict validation-driven methodology with 5 phases:

#### Phase 1: Tracking Validation
- Validated existing tracking code accuracy
- Distance validation: < 5% error
- Orientation validation: < 5 degrees error
- Velocity estimation: < 10% error
- Created validation infrastructure with test scripts

#### Phase 2: ROS2 Integration
- Created ROS2 package `multi_camera_robot_nav`
- Implemented ROS2 nodes for tracking
- Set up TF2 transform tree
- Published marker poses to ROS2 topics

#### Phase 3: Gazebo Simulation Environment
- Created Gazebo world with 4 ceiling cameras
- Built robot model with ArUco marker
- Implemented simulation bridge for testing
- Added obstacles for navigation testing

#### Phase 4: Point Cloud Obstacle Mapping
- Stitched point clouds from multiple cameras
- Generated 2D occupancy grid for Nav2
- Filtered floor and ceiling from point clouds
- Created /map topic for navigation

#### Phase 5: Robot Navigation with Nav2
- Integrated Nav2 navigation stack
- Configured planners and controllers
- Set up launch files for full system
- Implemented navigation goal handling

### New Components

#### ROS2 Nodes

| Node | Description |
|------|-------------|
| `marker_tracker_node` | Bridges internal tracking to ROS2 topics |
| `tf_broadcaster_node` | Publishes TF transforms (world->odom->base_link) |
| `simulation_bridge_node` | Extracts robot pose from Gazebo ground truth |
| `point_cloud_stitcher_node` | Combines point clouds from all cameras |
| `occupancy_grid_mapper_node` | Converts point cloud to 2D occupancy grid |

#### Topics Published

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/robot/pose` | PoseStamped | Robot pose in world frame |
| `/robot/pose_cov` | PoseWithCovarianceStamped | Pose with covariance |
| `/robot/odom` | Odometry | Full odometry with velocity |
| `/robot/velocity` | TwistStamped | Robot velocity |
| `/combined_cloud` | PointCloud2 | Stitched point cloud |
| `/filtered_cloud` | PointCloud2 | Height-filtered point cloud |
| `/map` | OccupancyGrid | 2D navigation map |

#### Launch Files

| Launch File | Description |
|-------------|-------------|
| `full_system.launch.py` | Complete system: Gazebo + tracking + Nav2 + RViz |
| `gz_simulation.launch.py` | Gazebo simulation only |
| `gazebo_simulation.launch.py` | Alternative Gazebo launch |
| `simulation.launch.py` | Simulation bridge launch |
| `marker_tracking.launch.py` | Marker tracking nodes |
| `mapping.launch.py` | Point cloud and occupancy grid |
| `navigation.launch.py` | Nav2 stack |
| `visualization.launch.py` | RViz visualization |

### Additional Project Structure

```
MultiCameraMakerTracking/
├── ros2_ws/
│   ├── src/
│   │   └── multi_camera_robot_nav/
│   │       ├── multi_camera_robot_nav/
│   │       │   ├── __init__.py
│   │       │   ├── marker_tracker_node.py
│   │       │   ├── tf_broadcaster_node.py
│   │       │   ├── simulation_bridge_node.py
│   │       │   ├── point_cloud_stitcher_node.py
│   │       │   └── occupancy_grid_mapper_node.py
│   │       ├── launch/
│   │       │   ├── full_system.launch.py
│   │       │   ├── gz_simulation.launch.py
│   │       │   ├── gazebo_simulation.launch.py
│   │       │   ├── simulation.launch.py
│   │       │   ├── marker_tracking.launch.py
│   │       │   ├── mapping.launch.py
│   │       │   ├── navigation.launch.py
│   │       │   └── visualization.launch.py
│   │       ├── config/
│   │       │   ├── nav2_params.yaml      # Nav2 configuration
│   │       │   └── navigation.rviz       # RViz layout
│   │       ├── worlds/
│   │       │   ├── tracking_room.sdf     # Ignition Gazebo world
│   │       │   └── tracking_room.world   # Classic Gazebo world
│   │       ├── urdf/
│   │       │   └── robot.urdf            # Robot model
│   │       ├── models/
│   │       │   └── marker_robot/
│   │       │       ├── model.config
│   │       │       ├── model.sdf
│   │       │       └── aruco_marker_0.png
│   │       ├── textures/
│   │       │   └── aruco_marker_0.png
│   │       ├── hooks/
│   │       ├── resource/
│   │       ├── test/
│   │       ├── package.xml
│   │       ├── setup.py
│   │       └── setup.cfg
│   ├── build/                            # Colcon build output
│   ├── install/                          # Installed packages
│   └── log/                              # Build logs
├── validation/
│   ├── __init__.py
│   ├── distance_validator.py             # Distance measurement validation
│   ├── orientation_validator.py          # Orientation accuracy validation
│   ├── velocity_estimator.py             # Velocity estimation from pose history
│   ├── fusion_validator.py               # Multi-camera fusion validation
│   ├── run_phase1_validation.py          # Phase 1 test runner
│   ├── run_phase2_validation.py          # Phase 2 test runner
│   ├── run_phase3_validation.py          # Phase 3 test runner
│   ├── run_phase4_validation.py          # Phase 4 test runner
│   └── run_phase5_validation.py          # Phase 5 test runner
└── docs/
    └── ROBOT_NAVIGATION_PROMPT.md        # Full project specification
```

### System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SIMULATION ENVIRONMENT                             │
│                                                                              │
│    ┌─────────────────────────────────────────────────────────────────────┐  │
│    │                      RECTANGULAR ROOM (8m x 8m)                      │  │
│    │                                                                      │  │
│    │   Camera 1 (NE)                                      Camera 2 (NW)  │  │
│    │   ↘ 45° down                                        ↙ 45° down      │  │
│    │                                                                      │  │
│    │          ┌─────┐     ┌─────┐                                        │  │
│    │          │ OBS │     │ OBS │    Random Obstacles                    │  │
│    │          └─────┘     └─────┘                                        │  │
│    │                                                                      │  │
│    │                    ┌───────────┐                                    │  │
│    │                    │   ROBOT   │  ← ArUco Marker on top             │  │
│    │                    └───────────┘                                    │  │
│    │                                                                      │  │
│    │   Camera 3 (SW)                                      Camera 4 (SE)  │  │
│    │   ↗ 45° down                                        ↖ 45° down      │  │
│    │                                                                      │  │
│    └─────────────────────────────────────────────────────────────────────┘  │
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

### Usage

#### Build ROS2 Package
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

#### Launch Full System (Simulation)
```bash
ros2 launch multi_camera_robot_nav full_system.launch.py
```

#### Launch Components Separately
```bash
# Terminal 1: Gazebo
ros2 launch multi_camera_robot_nav gz_simulation.launch.py

# Terminal 2: Marker tracking
ros2 launch multi_camera_robot_nav marker_tracking.launch.py simulation:=true

# Terminal 3: Mapping
ros2 launch multi_camera_robot_nav mapping.launch.py

# Terminal 4: Navigation
ros2 launch multi_camera_robot_nav navigation.launch.py
```

#### Send Navigation Goals
```bash
# Via command line
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}}}}"

# Or use RViz "2D Goal Pose" tool
```

### Key Constraints

- **NO onboard sensors** on the robot (no lidar, no camera, no IMU)
- All localization from external marker tracking
- All obstacle mapping from ceiling camera point clouds
- Robot receives pose updates via ROS2 topics
- Navigation uses Nav2 stack with custom localization

### Dependencies

```xml
<!-- ROS2 Core -->
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>

<!-- Simulation -->
<exec_depend>gazebo_msgs</exec_depend>
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>

<!-- Navigation -->
<exec_depend>nav2_bringup</exec_depend>
<exec_depend>nav2_bt_navigator</exec_depend>
<exec_depend>nav2_controller</exec_depend>
<exec_depend>nav2_planner</exec_depend>
<exec_depend>nav2_behaviors</exec_depend>
<exec_depend>nav2_costmap_2d</exec_depend>
<exec_depend>nav2_map_server</exec_depend>
```

---

## Branch Comparison

### Feature Comparison

| Feature | master | feature/robot-navigation-ros2 |
|---------|--------|------------------------------|
| ArUco Marker Detection | Yes | Yes |
| Multi-Camera Tracking | Yes | Yes |
| Pose Fusion | Yes | Yes |
| 3D Visualization (Open3D) | Yes | Yes |
| Network Streaming | Yes | Yes |
| ROS2 Integration | No | Yes |
| TF2 Transforms | No | Yes |
| Gazebo Simulation | No | Yes |
| Point Cloud Mapping | No | Yes |
| Occupancy Grid | No | Yes |
| Nav2 Navigation | No | Yes |
| RViz Visualization | No | Yes |
| Validation Framework | No | Yes |

### Files Changed Summary

| Category | Files Added/Modified |
|----------|---------------------|
| ROS2 Package | 40+ files (nodes, launch, config) |
| Simulation | 5+ files (world, robot model, URDF) |
| Validation | 10 files (validators, test runners) |
| Documentation | 1 file (ROBOT_NAVIGATION_PROMPT.md) |

### When to Use Each Branch

**Use `master` when:**
- You need standalone marker tracking without ROS2
- You're developing/testing the core tracking algorithms
- You want to use the system with Open3D visualization only
- You're setting up network streaming for cameras

**Use `feature/robot-navigation-ros2` when:**
- You need to integrate with a ROS2 robot system
- You want to use Nav2 for robot navigation
- You need simulation testing in Gazebo
- You require TF2 transforms for robot localization
- You want occupancy grid mapping from ceiling cameras

---

## Conclusion

The repository provides a complete solution for external camera-based robot tracking and navigation:

1. **master branch**: Robust marker tracking foundation with multi-camera support and real-time visualization
2. **feature/robot-navigation-ros2**: Full ROS2/Nav2 integration for autonomous robot navigation

Both branches are actively maintained and can be used independently based on your requirements.
