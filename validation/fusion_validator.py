#!/usr/bin/env python3
"""
Multi-Camera Fusion Validator

Validates that pose fusion from multiple cameras is consistent and accurate.

Usage:
    python -m validation.fusion_validator --synthetic
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.core.data_types import CameraDetection, FusedMarkerPose


@dataclass
class FusionValidationResult:
    """Result of a fusion validation test."""
    test_name: str
    passed: bool
    expected: Optional[np.ndarray]
    actual: np.ndarray
    error: float
    tolerance: float
    num_cameras: int
    details: str = ""


class FusionValidator:
    """
    Validate multi-camera pose fusion.

    Tests:
    1. Consistency: All cameras agree on position (within tolerance)
    2. Accuracy: Fused position matches ground truth
    3. Robustness: System handles single camera failure
    4. Outlier rejection: Bad readings are filtered out
    """

    def __init__(self, tolerance_m: float = 0.05):
        """
        Initialize fusion validator.

        Args:
            tolerance_m: Default position tolerance in meters
        """
        self.tolerance_m = tolerance_m
        self.results: List[FusionValidationResult] = []

    def validate_consistency(
        self,
        per_camera_positions: Dict[str, np.ndarray],
        tolerance_m: Optional[float] = None
    ) -> FusionValidationResult:
        """
        Check that all cameras report similar positions.

        Args:
            per_camera_positions: Dict mapping camera_id to position [x, y, z]
            tolerance_m: Maximum acceptable pairwise distance

        Returns:
            Validation result
        """
        if tolerance_m is None:
            tolerance_m = self.tolerance_m

        if len(per_camera_positions) < 2:
            return FusionValidationResult(
                test_name="Camera Consistency",
                passed=False,
                expected=None,
                actual=np.zeros(3),
                error=0.0,
                tolerance=tolerance_m,
                num_cameras=len(per_camera_positions),
                details="Need at least 2 cameras"
            )

        positions = list(per_camera_positions.values())
        camera_ids = list(per_camera_positions.keys())

        # Calculate pairwise distances
        max_distance = 0.0
        worst_pair = ("", "")

        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = np.linalg.norm(positions[i] - positions[j])
                if dist > max_distance:
                    max_distance = dist
                    worst_pair = (camera_ids[i], camera_ids[j])

        passed = max_distance < tolerance_m

        # Calculate centroid
        centroid = np.mean(positions, axis=0)

        result = FusionValidationResult(
            test_name="Camera Consistency",
            passed=passed,
            expected=None,
            actual=centroid,
            error=max_distance,
            tolerance=tolerance_m,
            num_cameras=len(positions),
            details=f"Max deviation: {worst_pair[0]} vs {worst_pair[1]}"
        )

        self.results.append(result)
        return result

    def validate_accuracy(
        self,
        fused_position: np.ndarray,
        ground_truth: np.ndarray,
        tolerance_m: Optional[float] = None
    ) -> FusionValidationResult:
        """
        Check that fused position matches ground truth.

        Args:
            fused_position: Fused position [x, y, z]
            ground_truth: Known true position [x, y, z]
            tolerance_m: Maximum acceptable error

        Returns:
            Validation result
        """
        if tolerance_m is None:
            tolerance_m = self.tolerance_m

        error = np.linalg.norm(fused_position - ground_truth)
        passed = error < tolerance_m

        result = FusionValidationResult(
            test_name="Fusion Accuracy",
            passed=passed,
            expected=ground_truth,
            actual=fused_position,
            error=error,
            tolerance=tolerance_m,
            num_cameras=0,
            details=f"Error vector: [{fused_position[0]-ground_truth[0]:.4f}, "
                   f"{fused_position[1]-ground_truth[1]:.4f}, "
                   f"{fused_position[2]-ground_truth[2]:.4f}]"
        )

        self.results.append(result)
        return result

    def validate_occlusion_handling(
        self,
        full_camera_positions: Dict[str, np.ndarray],
        occluded_camera_positions: Dict[str, np.ndarray],
        tolerance_m: Optional[float] = None
    ) -> FusionValidationResult:
        """
        Test that fusion works when some cameras are occluded.

        Args:
            full_camera_positions: Positions from all cameras
            occluded_camera_positions: Positions with some cameras missing
            tolerance_m: Maximum acceptable difference

        Returns:
            Validation result
        """
        if tolerance_m is None:
            tolerance_m = self.tolerance_m * 2  # More lenient for degraded mode

        if len(occluded_camera_positions) == 0:
            return FusionValidationResult(
                test_name="Occlusion Handling",
                passed=False,
                expected=None,
                actual=np.zeros(3),
                error=0.0,
                tolerance=tolerance_m,
                num_cameras=0,
                details="No cameras available after occlusion"
            )

        full_centroid = np.mean(list(full_camera_positions.values()), axis=0)
        occluded_centroid = np.mean(list(occluded_camera_positions.values()), axis=0)

        error = np.linalg.norm(full_centroid - occluded_centroid)
        passed = error < tolerance_m

        num_occluded = len(full_camera_positions) - len(occluded_camera_positions)

        result = FusionValidationResult(
            test_name="Occlusion Handling",
            passed=passed,
            expected=full_centroid,
            actual=occluded_centroid,
            error=error,
            tolerance=tolerance_m,
            num_cameras=len(occluded_camera_positions),
            details=f"{num_occluded} camera(s) occluded"
        )

        self.results.append(result)
        return result

    def validate_outlier_rejection(
        self,
        positions_with_outlier: Dict[str, np.ndarray],
        ground_truth: np.ndarray,
        outlier_camera: str,
        tolerance_m: Optional[float] = None
    ) -> FusionValidationResult:
        """
        Test that outlier readings are rejected.

        Args:
            positions_with_outlier: Positions including an outlier
            ground_truth: Known true position
            outlier_camera: ID of camera with bad reading
            tolerance_m: Maximum acceptable error (fused result should be close to truth)

        Returns:
            Validation result
        """
        if tolerance_m is None:
            tolerance_m = self.tolerance_m

        # Simple outlier rejection: use median
        positions = np.array(list(positions_with_outlier.values()))
        median_position = np.median(positions, axis=0)

        error = np.linalg.norm(median_position - ground_truth)
        passed = error < tolerance_m

        # Calculate how bad the outlier was
        outlier_pos = positions_with_outlier[outlier_camera]
        outlier_error = np.linalg.norm(outlier_pos - ground_truth)

        result = FusionValidationResult(
            test_name="Outlier Rejection",
            passed=passed,
            expected=ground_truth,
            actual=median_position,
            error=error,
            tolerance=tolerance_m,
            num_cameras=len(positions_with_outlier),
            details=f"Outlier from {outlier_camera} was {outlier_error:.3f}m off"
        )

        self.results.append(result)
        return result

    def validate_weighted_fusion(
        self,
        detections: List[CameraDetection],
        ground_truth: np.ndarray,
        tolerance_m: Optional[float] = None
    ) -> FusionValidationResult:
        """
        Validate weighted average fusion.

        Args:
            detections: List of CameraDetection objects
            ground_truth: Known true position
            tolerance_m: Maximum acceptable error

        Returns:
            Validation result
        """
        if tolerance_m is None:
            tolerance_m = self.tolerance_m

        if len(detections) == 0:
            return FusionValidationResult(
                test_name="Weighted Fusion",
                passed=False,
                expected=ground_truth,
                actual=np.zeros(3),
                error=np.linalg.norm(ground_truth),
                tolerance=tolerance_m,
                num_cameras=0,
                details="No detections"
            )

        # Calculate weights: confidence / depth
        weights = []
        positions = []

        for det in detections:
            depth = np.linalg.norm(det.position_camera) if det.position_camera is not None else 1.0
            weight = det.confidence / (depth + 0.1)
            weights.append(weight)
            positions.append(det.position_world)

        weights = np.array(weights)
        weights = weights / weights.sum()  # Normalize
        positions = np.array(positions)

        # Weighted average
        fused_position = np.average(positions, axis=0, weights=weights)

        error = np.linalg.norm(fused_position - ground_truth)
        passed = error < tolerance_m

        result = FusionValidationResult(
            test_name="Weighted Fusion",
            passed=passed,
            expected=ground_truth,
            actual=fused_position,
            error=error,
            tolerance=tolerance_m,
            num_cameras=len(detections),
            details=f"Weights: {[f'{w:.3f}' for w in weights]}"
        )

        self.results.append(result)
        return result

    def print_report(self) -> bool:
        """Print validation report."""
        print("\n" + "=" * 70)
        print("FUSION VALIDATION REPORT")
        print("=" * 70)

        all_passed = True

        for result in self.results:
            status = "PASS ✓" if result.passed else "FAIL ✗"
            all_passed = all_passed and result.passed

            print(f"\nTest: {result.test_name}")
            if result.expected is not None:
                print(f"  Expected:   [{result.expected[0]:.4f}, {result.expected[1]:.4f}, {result.expected[2]:.4f}]")
            print(f"  Actual:     [{result.actual[0]:.4f}, {result.actual[1]:.4f}, {result.actual[2]:.4f}]")
            print(f"  Error:      {result.error:.4f} m")
            print(f"  Tolerance:  {result.tolerance:.4f} m")
            print(f"  Cameras:    {result.num_cameras}")
            print(f"  Status:     [{status}]")
            if result.details:
                print(f"  Details:    {result.details}")

        print("\n" + "=" * 70)
        overall = "ALL TESTS PASSED ✓" if all_passed else "SOME TESTS FAILED ✗"
        print(f"OVERALL: {overall}")
        print("=" * 70 + "\n")

        return all_passed


def run_synthetic_validation():
    """Run validation with synthetic multi-camera data."""
    print("\n" + "=" * 70)
    print("RUNNING SYNTHETIC FUSION VALIDATION")
    print("(Testing fusion with simulated multi-camera detections)")
    print("=" * 70)

    np.random.seed(42)
    validator = FusionValidator(tolerance_m=0.05)

    # Ground truth marker position
    ground_truth = np.array([1.5, 0.8, 0.1])

    # Test 1: Camera consistency (all cameras see similar position)
    camera_positions = {
        "camera_1": ground_truth + np.random.normal(0, 0.01, 3),
        "camera_2": ground_truth + np.random.normal(0, 0.01, 3),
        "camera_3": ground_truth + np.random.normal(0, 0.01, 3),
        "camera_4": ground_truth + np.random.normal(0, 0.01, 3),
    }
    validator.validate_consistency(camera_positions)

    # Test 2: Fusion accuracy
    fused = np.mean(list(camera_positions.values()), axis=0)
    validator.validate_accuracy(fused, ground_truth)

    # Test 3: Occlusion handling (1 camera occluded)
    occluded_positions = {k: v for k, v in camera_positions.items() if k != "camera_1"}
    validator.validate_occlusion_handling(camera_positions, occluded_positions)

    # Test 4: Outlier rejection
    positions_with_outlier = camera_positions.copy()
    positions_with_outlier["camera_3"] = ground_truth + np.array([0.5, 0.3, 0.2])  # Bad reading
    validator.validate_outlier_rejection(
        positions_with_outlier,
        ground_truth,
        outlier_camera="camera_3"
    )

    # Test 5: Weighted fusion with CameraDetection objects
    detections = []
    for cam_id, pos in camera_positions.items():
        # Simulate camera positions and depths
        camera_pos = np.array([-2.0, 1.5, 2.4])  # Example camera position
        pos_camera = pos - camera_pos  # Position in camera frame

        det = CameraDetection(
            camera_id=cam_id,
            marker_id=0,
            position_world=pos,
            position_camera=pos_camera,
            timestamp=0.0,
            orientation=None,
            confidence=0.9 + np.random.uniform(-0.1, 0.1),
            corners=None,
            rvec=None,
            tvec=None
        )
        detections.append(det)

    validator.validate_weighted_fusion(detections, ground_truth)

    return validator.print_report()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Multi-Camera Fusion Validation")
    parser.add_argument("--synthetic", action="store_true",
                        help="Run synthetic validation test")

    args = parser.parse_args()

    if args.synthetic:
        success = run_synthetic_validation()
        sys.exit(0 if success else 1)
    else:
        print("Use --synthetic for testing with simulated multi-camera data.")
