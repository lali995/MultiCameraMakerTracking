# Multi-Camera Marker Tracking Implementation Plan

## Overview

Expand the current single-camera tracking system to support multiple simultaneous cameras with pose fusion and visualization of which cameras detected each marker.

### Current State
- StreamServer already supports multiple cameras (stores frames by camera_id)
- Tracker processes only one camera (filtered by camera_id_filter)
- Calibration file contains 3 cameras with extrinsic matrices
- MarkerPose doesn't track which camera(s) detected it

### Target State
- Process frames from all 3 cameras simultaneously
- Fuse detections when marker is seen by 2+ cameras
- Visualize which cameras are currently detecting each marker

---

## Implementation Steps

### Step 1: Extend Data Types

**File: `src/core/data_types.py`**

Add new data types for multi-camera tracking:

```python
@dataclass
class CameraDetection:
    """Single camera's detection of a marker."""
    camera_id: str
    marker_id: int
    position_world: np.ndarray  # 3D position in world frame
    position_camera: np.ndarray  # 3D position in camera frame
    orientation: Optional[Tuple[float, float, float, float]]
    confidence: float
    timestamp: float

@dataclass
class FusedMarkerPose:
    """Fused marker pose from multiple camera observations."""
    marker_id: int
    x: float
    y: float
    z: float
    timestamp: float
    orientation: Optional[Tuple[float, float, float, float]] = None
    confidence: float = 1.0
    # NEW: Track which cameras detected this marker
    detecting_cameras: List[str] = field(default_factory=list)
    # Individual camera detections for debugging
    camera_detections: List[CameraDetection] = field(default_factory=list)
```

---

### Step 2: Create Multi-Camera Pose Fusion Module

**New File: `src/tracking/pose_fusion.py`**

Implements fusion algorithms for combining detections from multiple cameras:

```python
class PoseFusion:
    """Fuses marker detections from multiple cameras."""

    def __init__(self, fusion_method: str = "weighted_average"):
        """
        Args:
            fusion_method: "weighted_average", "median", or "least_squares"
        """
        self.fusion_method = fusion_method
        self.detection_window_ms = 50  # Max time difference for fusion

    def fuse_detections(self, detections: List[CameraDetection]) -> FusedMarkerPose:
        """
        Fuse multiple camera detections into a single pose estimate.

        Fusion approaches:
        1. Weighted Average: Weight by detection confidence and distance to camera
        2. Median: Take median position (robust to outliers)
        3. Least Squares: Minimize reprojection error across all cameras
        """
        if len(detections) == 1:
            # Single camera - return as-is
            return self._single_detection_to_pose(detections[0])

        if self.fusion_method == "weighted_average":
            return self._fuse_weighted_average(detections)
        elif self.fusion_method == "median":
            return self._fuse_median(detections)
        else:
            return self._fuse_least_squares(detections)

    def _fuse_weighted_average(self, detections: List[CameraDetection]) -> FusedMarkerPose:
        """
        Weight positions by confidence and inverse distance.
        Closer cameras get higher weight (less depth uncertainty).
        """
        positions = np.array([d.position_world for d in detections])

        # Weight by confidence and inverse depth
        weights = []
        for d in detections:
            depth = np.linalg.norm(d.position_camera)
            weight = d.confidence / (depth + 0.1)  # Avoid division by zero
            weights.append(weight)

        weights = np.array(weights) / sum(weights)  # Normalize

        fused_position = np.average(positions, axis=0, weights=weights)
        fused_confidence = np.mean([d.confidence for d in detections])

        return FusedMarkerPose(
            marker_id=detections[0].marker_id,
            x=fused_position[0],
            y=fused_position[1],
            z=fused_position[2],
            timestamp=max(d.timestamp for d in detections),
            confidence=fused_confidence,
            detecting_cameras=[d.camera_id for d in detections],
            camera_detections=detections
        )
```

---

### Step 3: Create Multi-Camera Tracker

**New File: `src/tracking/multi_camera_tracker.py`**

Processes all camera streams and coordinates fusion:

```python
class MultiCameraTracker:
    """
    Tracks ArUco markers across multiple cameras with pose fusion.
    """

    def __init__(
        self,
        message_bus: MessageBus,
        server: StreamServer,
        camera_configs: Dict[str, CameraConfig],  # camera_id -> config
        marker_size_m: float = 0.05,
        fusion_method: str = "weighted_average"
    ):
        self.server = server
        self.message_bus = message_bus
        self.camera_configs = camera_configs  # Contains intrinsic + extrinsic for each camera

        # Per-camera detectors
        self.detectors: Dict[str, ArucoDetector] = {}
        for cam_id, config in camera_configs.items():
            self.detectors[cam_id] = ArucoDetector(
                camera_matrix=config.intrinsic,
                dist_coeffs=config.dist_coeffs,
                extrinsic=config.extrinsic,
                marker_size_m=marker_size_m
            )

        # Pose fusion
        self.fusion = PoseFusion(fusion_method)

        # Detection buffer for time synchronization
        self.detection_buffer: Dict[int, List[CameraDetection]] = defaultdict(list)
        self.buffer_timeout_ms = 50

        # Publishers
        self.pose_publisher = message_bus.create_publisher('/markers/fused_poses', FusedMarkerPose)
        self.raw_publisher = message_bus.create_publisher('/markers/raw_detections', CameraDetection)

    def _processing_loop(self):
        """Main processing loop - processes all cameras."""
        while self._running:
            current_time = time.time()

            # Collect detections from all cameras
            new_detections = []
            for camera_id in self.server.get_camera_ids():
                if camera_id not in self.detectors:
                    continue

                frame = self.server.get_frame(camera_id)
                if frame is None:
                    continue

                # Detect markers in this camera's frame
                detections = self.detectors[camera_id].detect(frame, camera_id)
                new_detections.extend(detections)

                # Publish raw detections
                for det in detections:
                    self.raw_publisher.publish(det)

            # Group detections by marker ID
            for detection in new_detections:
                self.detection_buffer[detection.marker_id].append(detection)

            # Fuse and publish markers with recent detections
            for marker_id, detections in list(self.detection_buffer.items()):
                # Remove old detections
                detections = [d for d in detections
                             if (current_time - d.timestamp) < self.buffer_timeout_ms / 1000]

                if detections:
                    # Fuse detections
                    fused_pose = self.fusion.fuse_detections(detections)
                    self.pose_publisher.publish(fused_pose)

                self.detection_buffer[marker_id] = detections
```

---

### Step 4: Extend Visualization for Multi-Camera Detection Display

**File: `src/visualization/scene_manager.py`**

Add visual indicators for which cameras are detecting each marker:

```python
class SceneManager:
    def __init__(self, ...):
        # Existing initialization...

        # NEW: Camera detection lines (marker -> camera)
        self.detection_lines: Dict[Tuple[int, str], o3d.geometry.LineSet] = {}

        # Camera colors for detection lines
        self.camera_colors: Dict[str, List[float]] = {}

    def set_camera_colors(self, camera_ids: List[str]):
        """Assign distinct colors to each camera."""
        colors = [
            [1.0, 0.3, 0.3],  # Red
            [0.3, 1.0, 0.3],  # Green
            [0.3, 0.3, 1.0],  # Blue
            [1.0, 1.0, 0.3],  # Yellow
        ]
        for i, cam_id in enumerate(camera_ids):
            self.camera_colors[cam_id] = colors[i % len(colors)]

    def update_detection_lines(
        self,
        marker_id: int,
        marker_position: np.ndarray,
        detecting_cameras: List[str],
        camera_positions: Dict[str, np.ndarray]
    ) -> List[o3d.geometry.LineSet]:
        """
        Create/update lines from marker to detecting cameras.
        Returns new geometries to add.
        """
        new_geometries = []
        active_keys = set()

        for cam_id in detecting_cameras:
            key = (marker_id, cam_id)
            active_keys.add(key)

            if cam_id not in camera_positions:
                continue

            cam_pos = camera_positions[cam_id]
            color = self.camera_colors.get(cam_id, [0.5, 0.5, 0.5])

            if key in self.detection_lines:
                # Update existing line
                line = self.detection_lines[key]
                line.points = o3d.utility.Vector3dVector([marker_position, cam_pos])
            else:
                # Create new line
                line = o3d.geometry.LineSet()
                line.points = o3d.utility.Vector3dVector([marker_position, cam_pos])
                line.lines = o3d.utility.Vector2iVector([[0, 1]])
                line.colors = o3d.utility.Vector3dVector([color])
                self.detection_lines[key] = line
                new_geometries.append(line)

        return new_geometries

    def get_stale_detection_lines(self, marker_id: int, active_cameras: List[str]) -> List:
        """Get detection lines that should be removed."""
        stale = []
        for (mid, cam_id), line in list(self.detection_lines.items()):
            if mid == marker_id and cam_id not in active_cameras:
                stale.append(line)
                del self.detection_lines[(mid, cam_id)]
        return stale
```

---

### Step 5: Update Viewer for Multi-Camera Display

**File: `src/visualization/viewer.py`**

Update to handle FusedMarkerPose and display detection lines:

```python
class MarkerViewer:
    def __init__(self, ...):
        # Existing initialization...

        # NEW: Camera position cache for drawing detection lines
        self._camera_positions: Dict[str, np.ndarray] = {}

    def setup(self, camera_poses: List[CameraPose]) -> None:
        # Existing setup...

        # Cache camera positions
        for pose in camera_poses:
            self._camera_positions[pose.camera_id] = pose.position

        # Set camera colors in scene manager
        self.scene.set_camera_colors([p.camera_id for p in camera_poses])

        # Subscribe to fused poses (in addition to raw)
        self.bus.subscribe('/markers/fused_poses', self._on_fused_pose)

    def _on_fused_pose(self, pose: FusedMarkerPose) -> None:
        """Handle fused pose with camera detection info."""
        with self._pending_lock:
            self._pending_markers[pose.marker_id] = pose

    def _process_marker_updates(self) -> None:
        # Existing position update logic...

        for marker_id, pose in markers_to_process.items():
            # Existing position updates...

            # NEW: Update detection lines
            if hasattr(pose, 'detecting_cameras'):
                # Add new detection lines
                new_lines = self.scene.update_detection_lines(
                    marker_id,
                    np.array([pose.x, pose.y, pose.z]),
                    pose.detecting_cameras,
                    self._camera_positions
                )
                for line in new_lines:
                    self.vis.add_geometry(line, reset_bounding_box=False)

                # Remove stale detection lines
                stale_lines = self.scene.get_stale_detection_lines(
                    marker_id,
                    pose.detecting_cameras
                )
                for line in stale_lines:
                    self.vis.remove_geometry(line, reset_bounding_box=False)
```

---

### Step 6: Create Main Multi-Camera Script

**New File: `scripts/run_multi_camera_tracker.py`**

Main entry point for multi-camera tracking:

```python
def main():
    parser = argparse.ArgumentParser(description="Multi-Camera ArUco Tracker")
    parser.add_argument("--calibration", required=True, help="Path to calibration_result.json")
    parser.add_argument("--reference", default="250514", help="Reference camera ID")
    parser.add_argument("--marker-size", type=float, default=0.05)
    parser.add_argument("--fusion", choices=["weighted_average", "median", "least_squares"],
                       default="weighted_average")
    parser.add_argument("--preview", action="store_true", help="Show video preview grid")
    args = parser.parse_args()

    # Load calibration for all cameras
    config_loader = ConfigLoader('config', reference_camera_id=args.reference)
    camera_poses = config_loader.load_from_calibration_file(args.calibration, args.reference)

    # Build camera configs
    camera_configs = {}
    for pose in camera_poses:
        camera_configs[pose.camera_id] = CameraConfig(
            camera_id=pose.camera_id,
            extrinsic=pose.extrinsic,
            intrinsic=None,  # Use default
            dist_coeffs=None
        )

    # Create message bus
    message_bus = MessageBus()

    # Start stream server
    server = StreamServer(port=5555)
    server.start()

    # Create multi-camera tracker
    tracker = MultiCameraTracker(
        message_bus=message_bus,
        server=server,
        camera_configs=camera_configs,
        marker_size_m=args.marker_size,
        fusion_method=args.fusion
    )
    tracker.start()

    # Create viewer
    viewer = MarkerViewer(message_bus=message_bus, show_trajectory=True)
    viewer.setup(camera_poses)
    viewer.run()
```

---

### Step 7: Multi-Camera Preview Grid (Optional)

**Enhancement to video preview:**

Show a grid of all camera feeds with detection overlays:

```python
class MultiCameraPreview:
    """Shows grid of all camera feeds with detection overlays."""

    def __init__(self, camera_ids: List[str], cols: int = 2):
        self.camera_ids = camera_ids
        self.cols = cols
        self.rows = (len(camera_ids) + cols - 1) // cols

    def create_grid(self, frames: Dict[str, np.ndarray],
                   detections: Dict[str, List[dict]]) -> np.ndarray:
        """Create grid image with all camera feeds."""
        # Resize frames to uniform size
        target_size = (640, 360)
        grid_frames = []

        for cam_id in self.camera_ids:
            if cam_id in frames:
                frame = cv2.resize(frames[cam_id], target_size)
                # Draw camera ID
                cv2.putText(frame, cam_id[-12:], (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                # Draw detections
                if cam_id in detections:
                    for det in detections[cam_id]:
                        cv2.polylines(frame, [det['corners']], True, (0, 255, 0), 2)
                grid_frames.append(frame)
            else:
                # Placeholder for missing camera
                placeholder = np.zeros((*target_size[::-1], 3), dtype=np.uint8)
                cv2.putText(placeholder, f"No signal: {cam_id[-12:]}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                grid_frames.append(placeholder)

        # Arrange in grid
        rows = []
        for r in range(self.rows):
            row_frames = grid_frames[r*self.cols:(r+1)*self.cols]
            while len(row_frames) < self.cols:
                row_frames.append(np.zeros_like(row_frames[0]))
            rows.append(np.hstack(row_frames))

        return np.vstack(rows)
```

---

## Summary of Changes

| Component | File | Changes |
|-----------|------|---------|
| Data Types | `src/core/data_types.py` | Add `CameraDetection`, `FusedMarkerPose` |
| Pose Fusion | `src/tracking/pose_fusion.py` | NEW - Fusion algorithms |
| Multi-Camera Tracker | `src/tracking/multi_camera_tracker.py` | NEW - Process all cameras |
| Scene Manager | `src/visualization/scene_manager.py` | Add detection lines |
| Viewer | `src/visualization/viewer.py` | Handle fused poses, draw detection lines |
| Main Script | `scripts/run_multi_camera_tracker.py` | NEW - Entry point |
| Preview | Enhancement | Optional grid view of all cameras |

---

## Testing Plan

1. **Single Camera Test**: Verify existing single-camera functionality still works
2. **Multi-Camera Sync Test**: Ensure frames from all cameras are processed
3. **Fusion Accuracy Test**: Compare fused position with individual camera estimates
4. **Visualization Test**: Verify detection lines appear/disappear correctly
5. **Performance Test**: Ensure system maintains real-time performance with 3 cameras

---

## Dependencies

No new external dependencies required. Uses existing:
- OpenCV (ArUco detection)
- NumPy (matrix operations)
- Open3D (visualization)
- ZMQ (streaming)
