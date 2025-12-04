"""
Shared data types for marker tracking system.
Designed to be compatible with future ROS message types.
"""
from dataclasses import dataclass, field
from typing import Tuple, Optional
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
