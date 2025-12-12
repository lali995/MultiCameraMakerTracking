"""
Multi-camera pose fusion for ArUco marker tracking.

Combines marker detections from multiple cameras to produce
more accurate and robust pose estimates.
"""

import numpy as np
from typing import List, Optional
from collections import defaultdict
import time

from ..core.data_types import CameraDetection, FusedMarkerPose


class PoseFusion:
    """
    Fuses marker detections from multiple cameras.

    Supports multiple fusion methods:
    - weighted_average: Weight by confidence and inverse depth
    - median: Take median position (robust to outliers)
    - least_squares: Minimize position variance (future)
    """

    def __init__(
        self,
        fusion_method: str = "weighted_average",
        detection_window_ms: float = 50.0
    ):
        """
        Initialize pose fusion.

        Args:
            fusion_method: Fusion algorithm to use
            detection_window_ms: Max time difference (ms) to consider detections simultaneous
        """
        self.fusion_method = fusion_method
        self.detection_window_ms = detection_window_ms

        # Detection buffer for time synchronization
        # marker_id -> list of recent detections
        self._detection_buffer: defaultdict = defaultdict(list)

    def add_detection(self, detection: CameraDetection) -> None:
        """
        Add a detection to the buffer.

        Args:
            detection: Camera detection to add
        """
        self._detection_buffer[detection.marker_id].append(detection)

    def get_fused_poses(self, current_time: float = None) -> List[FusedMarkerPose]:
        """
        Get fused poses for all markers with recent detections.

        Args:
            current_time: Current timestamp (default: time.time())

        Returns:
            List of fused marker poses
        """
        if current_time is None:
            current_time = time.time()

        fused_poses = []
        window_sec = self.detection_window_ms / 1000.0

        for marker_id, detections in list(self._detection_buffer.items()):
            # Filter to recent detections within time window
            recent = [d for d in detections
                     if (current_time - d.timestamp) < window_sec]

            # Deduplicate by camera_id - keep only the most recent from each camera
            camera_detections = {}
            for det in recent:
                if det.camera_id not in camera_detections or det.timestamp > camera_detections[det.camera_id].timestamp:
                    camera_detections[det.camera_id] = det

            unique_detections = list(camera_detections.values())

            if unique_detections:
                fused_pose = self.fuse_detections(unique_detections)
                fused_poses.append(fused_pose)

            # Update buffer with only recent detections
            self._detection_buffer[marker_id] = recent

        return fused_poses

    def fuse_detections(self, detections: List[CameraDetection]) -> FusedMarkerPose:
        """
        Fuse multiple camera detections into a single pose estimate.

        Args:
            detections: List of camera detections for same marker

        Returns:
            Fused marker pose
        """
        if len(detections) == 0:
            raise ValueError("Cannot fuse empty detection list")

        if len(detections) == 1:
            return self._single_detection_to_pose(detections[0])

        if self.fusion_method == "weighted_average":
            return self._fuse_weighted_average(detections)
        elif self.fusion_method == "median":
            return self._fuse_median(detections)
        elif self.fusion_method == "least_squares":
            return self._fuse_least_squares(detections)
        else:
            # Default to weighted average
            return self._fuse_weighted_average(detections)

    def _single_detection_to_pose(self, detection: CameraDetection) -> FusedMarkerPose:
        """Convert single detection to fused pose."""
        return FusedMarkerPose(
            marker_id=detection.marker_id,
            x=float(detection.position_world[0]),
            y=float(detection.position_world[1]),
            z=float(detection.position_world[2]),
            timestamp=detection.timestamp,
            orientation=detection.orientation,
            confidence=detection.confidence,
            detecting_cameras=[detection.camera_id],
            camera_detections=[detection],
            fusion_method="single"
        )

    def _fuse_weighted_average(self, detections: List[CameraDetection]) -> FusedMarkerPose:
        """
        Fuse using weighted average.

        Weights are computed based on:
        - Detection confidence
        - Inverse depth (closer = higher weight, less depth uncertainty)
        """
        positions = np.array([d.position_world for d in detections])

        # Compute weights
        weights = []
        for d in detections:
            # Depth from camera (distance)
            depth = np.linalg.norm(d.position_camera)
            # Weight: confidence / depth (closer cameras weighted higher)
            weight = d.confidence / (depth + 0.1)  # +0.1 to avoid div by zero
            weights.append(weight)

        weights = np.array(weights)
        weights = weights / weights.sum()  # Normalize

        # Weighted average position
        fused_position = np.average(positions, axis=0, weights=weights)

        # Average confidence
        fused_confidence = np.mean([d.confidence for d in detections])

        # Fuse orientation (use highest confidence detection's orientation)
        best_detection = max(detections, key=lambda d: d.confidence)
        fused_orientation = best_detection.orientation

        return FusedMarkerPose(
            marker_id=detections[0].marker_id,
            x=float(fused_position[0]),
            y=float(fused_position[1]),
            z=float(fused_position[2]),
            timestamp=max(d.timestamp for d in detections),
            orientation=fused_orientation,
            confidence=fused_confidence,
            detecting_cameras=[d.camera_id for d in detections],
            camera_detections=detections,
            fusion_method="weighted_average"
        )

    def _fuse_median(self, detections: List[CameraDetection]) -> FusedMarkerPose:
        """
        Fuse using median position.

        Robust to outliers - useful when one camera has a bad estimate.
        """
        positions = np.array([d.position_world for d in detections])

        # Median position
        fused_position = np.median(positions, axis=0)

        # Confidence based on consistency
        distances = np.linalg.norm(positions - fused_position, axis=1)
        consistency = 1.0 / (1.0 + np.mean(distances))  # Higher if positions agree

        # Orientation from closest to median
        closest_idx = np.argmin(distances)
        fused_orientation = detections[closest_idx].orientation

        return FusedMarkerPose(
            marker_id=detections[0].marker_id,
            x=float(fused_position[0]),
            y=float(fused_position[1]),
            z=float(fused_position[2]),
            timestamp=max(d.timestamp for d in detections),
            orientation=fused_orientation,
            confidence=consistency,
            detecting_cameras=[d.camera_id for d in detections],
            camera_detections=detections,
            fusion_method="median"
        )

    def _fuse_least_squares(self, detections: List[CameraDetection]) -> FusedMarkerPose:
        """
        Fuse using least squares optimization.

        Minimizes weighted sum of squared distances from each camera's estimate.
        For now, this is equivalent to weighted average.
        Future: Could incorporate reprojection error minimization.
        """
        # For now, use weighted average
        # TODO: Implement proper least squares with reprojection error
        return self._fuse_weighted_average(detections)

    def clear_buffer(self) -> None:
        """Clear the detection buffer."""
        self._detection_buffer.clear()

    def get_buffer_stats(self) -> dict:
        """Get statistics about the detection buffer."""
        stats = {}
        for marker_id, detections in self._detection_buffer.items():
            stats[marker_id] = {
                'num_detections': len(detections),
                'cameras': list(set(d.camera_id for d in detections))
            }
        return stats
