"""
Shared data types for marker tracking system.
Designed to be compatible with future ROS message types.
"""
from dataclasses import dataclass, field
from typing import Tuple, Optional, List
import numpy as np
import time


@dataclass
class MarkerPose:
    """
    Represents the pose of an ArUco marker in 3D space.
    Maps to geometry_msgs/PoseStamped in ROS.
    """
    marker_id: int
    x: float
    y: float
    z: float
    timestamp: float = field(default_factory=time.time)
    # Optional orientation as quaternion (x, y, z, w)
    orientation: Optional[Tuple[float, float, float, float]] = None
    # Detection confidence 0-1
    confidence: float = 1.0

    def to_array(self) -> np.ndarray:
        """Return position as numpy array."""
        return np.array([self.x, self.y, self.z])

    def __repr__(self) -> str:
        return f"MarkerPose(id={self.marker_id}, pos=({self.x:.3f}, {self.y:.3f}, {self.z:.3f}))"


@dataclass
class CameraPose:
    """
    Represents a camera's pose in world coordinates.
    Extracted from sensor_config.json extrinsic matrix.
    """
    camera_id: str
    position: np.ndarray  # 3D position [x, y, z]
    direction: np.ndarray  # View direction unit vector [u, v, w]
    extrinsic: Optional[np.ndarray] = None  # Full 4x4 transformation matrix

    def __repr__(self) -> str:
        pos = self.position
        return f"CameraPose(id={self.camera_id}, pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}))"


@dataclass
class TrackerStatus:
    """
    Status information from the tracker module.
    """
    is_running: bool
    fps: float
    num_markers_detected: int
    timestamp: float = field(default_factory=time.time)


@dataclass
class CameraDetection:
    """
    Single camera's detection of a marker.
    Used for multi-camera tracking before fusion.
    """
    camera_id: str
    marker_id: int
    position_world: np.ndarray  # 3D position in world frame
    position_camera: np.ndarray  # 3D position in camera frame
    timestamp: float = field(default_factory=time.time)
    orientation: Optional[Tuple[float, float, float, float]] = None
    confidence: float = 1.0
    # Additional detection info
    corners: Optional[np.ndarray] = None  # 2D corner points in image
    rvec: Optional[np.ndarray] = None  # Rotation vector from solvePnP
    tvec: Optional[np.ndarray] = None  # Translation vector from solvePnP

    def __repr__(self) -> str:
        pos = self.position_world
        return f"CameraDetection(cam={self.camera_id[-12:]}, marker={self.marker_id}, pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}))"


@dataclass
class FusedMarkerPose:
    """
    Fused marker pose from multiple camera observations.
    Extends MarkerPose with multi-camera detection info.
    """
    marker_id: int
    x: float
    y: float
    z: float
    timestamp: float = field(default_factory=time.time)
    orientation: Optional[Tuple[float, float, float, float]] = None
    confidence: float = 1.0
    # Multi-camera tracking info
    detecting_cameras: List[str] = field(default_factory=list)
    camera_detections: List[CameraDetection] = field(default_factory=list)
    # Fusion metadata
    fusion_method: str = "single"  # "single", "weighted_average", "median", "least_squares"

    def to_array(self) -> np.ndarray:
        """Return position as numpy array."""
        return np.array([self.x, self.y, self.z])

    def num_cameras(self) -> int:
        """Return number of cameras that detected this marker."""
        return len(self.detecting_cameras)

    def __repr__(self) -> str:
        cams = ", ".join([c[-8:] for c in self.detecting_cameras])
        return f"FusedMarkerPose(id={self.marker_id}, pos=({self.x:.3f}, {self.y:.3f}, {self.z:.3f}), cams=[{cams}])"
