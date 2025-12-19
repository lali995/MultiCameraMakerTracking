#!/usr/bin/env python3
"""
Orientation Validator for Multi-Camera Marker Tracking

Validates that marker orientation (roll, pitch, yaw) is accurately detected.

Usage:
    python -m validation.orientation_validator --synthetic
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
import sys
import os
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


@dataclass
class OrientationResult:
    """Result of an orientation validation test."""
    test_name: str
    passed: bool
    axis: str  # roll, pitch, yaw
    expected_deg: float
    actual_deg: float
    error_deg: float
    tolerance_deg: float
    std_dev_deg: float
    num_samples: int
    details: str = ""


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

    # Default test cases
    DEFAULT_TEST_CASES = [
        {"axis": "yaw", "angles_deg": [0, 45, 90, 135, 180, -45, -90], "tolerance_deg": 3.0},
        {"axis": "pitch", "angles_deg": [0, 15, 30, 45, -15, -30], "tolerance_deg": 5.0},
        {"axis": "roll", "angles_deg": [0, 15, 30, -15, -30], "tolerance_deg": 5.0},
    ]

    def __init__(self, tracker=None):
        """
        Initialize the orientation validator.

        Args:
            tracker: Optional tracker instance for live validation
        """
        self.tracker = tracker
        self.results: List[OrientationResult] = []

    def rotation_matrix_from_rvec(self, rvec: np.ndarray) -> np.ndarray:
        """Convert OpenCV rvec to 3x3 rotation matrix."""
        R, _ = cv2.Rodrigues(rvec)
        return R

    def rotation_to_euler(self, R: np.ndarray) -> Tuple[float, float, float]:
        """
        Extract Euler angles from rotation matrix (ZYX convention).

        Args:
            R: 3x3 rotation matrix

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0

        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def euler_to_rotation_matrix(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float
    ) -> np.ndarray:
        """
        Create rotation matrix from Euler angles (ZYX convention).

        Args:
            roll_deg: Roll angle in degrees
            pitch_deg: Pitch angle in degrees
            yaw_deg: Yaw angle in degrees

        Returns:
            3x3 rotation matrix
        """
        roll = np.radians(roll_deg)
        pitch = np.radians(pitch_deg)
        yaw = np.radians(yaw_deg)

        # Rotation matrices for each axis
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # ZYX convention: R = Rz * Ry * Rx
        return Rz @ Ry @ Rx

    def quaternion_to_euler(self, q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        """
        Convert quaternion to Euler angles.

        Args:
            q: Quaternion as (x, y, z, w)

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        x, y, z, w = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Gimbal lock
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def normalize_angle(self, angle_deg: float) -> float:
        """Normalize angle to [-180, 180] range."""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg

    def angular_difference(self, angle1_deg: float, angle2_deg: float) -> float:
        """
        Calculate the smallest angular difference between two angles.

        Args:
            angle1_deg: First angle in degrees
            angle2_deg: Second angle in degrees

        Returns:
            Absolute angular difference in degrees [0, 180]
        """
        diff = self.normalize_angle(angle1_deg - angle2_deg)
        return abs(diff)

    def validate_orientation(
        self,
        axis: str,
        ground_truth_deg: float,
        measured_euler_samples: List[Tuple[float, float, float]],
        tolerance_deg: float = 5.0
    ) -> OrientationResult:
        """
        Validate orientation measurements for a specific axis.

        Args:
            axis: 'roll', 'pitch', or 'yaw'
            ground_truth_deg: Expected angle in degrees
            measured_euler_samples: List of (roll, pitch, yaw) tuples
            tolerance_deg: Acceptable error in degrees

        Returns:
            OrientationResult with pass/fail and statistics
        """
        if len(measured_euler_samples) == 0:
            return OrientationResult(
                test_name=f"{axis.capitalize()} {ground_truth_deg}°",
                passed=False,
                axis=axis,
                expected_deg=ground_truth_deg,
                actual_deg=0.0,
                error_deg=abs(ground_truth_deg),
                tolerance_deg=tolerance_deg,
                std_dev_deg=0.0,
                num_samples=0,
                details="No samples provided"
            )

        # Extract the relevant axis
        axis_idx = {'roll': 0, 'pitch': 1, 'yaw': 2}[axis.lower()]
        angles = np.array([sample[axis_idx] for sample in measured_euler_samples])

        # Handle angle wrapping for mean calculation
        # Use circular mean for angles
        sin_mean = np.mean(np.sin(np.radians(angles)))
        cos_mean = np.mean(np.cos(np.radians(angles)))
        mean_angle = np.degrees(np.arctan2(sin_mean, cos_mean))

        # Calculate circular standard deviation
        R = np.sqrt(sin_mean**2 + cos_mean**2)
        std_dev = np.degrees(np.sqrt(-2 * np.log(R))) if R > 0 else 180.0

        # Calculate error
        error = self.angular_difference(mean_angle, ground_truth_deg)
        passed = error < tolerance_deg

        result = OrientationResult(
            test_name=f"{axis.capitalize()} {ground_truth_deg}°",
            passed=passed,
            axis=axis,
            expected_deg=ground_truth_deg,
            actual_deg=mean_angle,
            error_deg=error,
            tolerance_deg=tolerance_deg,
            std_dev_deg=std_dev,
            num_samples=len(angles),
            details=f"Range: [{np.min(angles):.1f}°, {np.max(angles):.1f}°]"
        )

        self.results.append(result)
        return result

    def validate_quaternion_consistency(
        self,
        quaternions: List[Tuple[float, float, float, float]],
        max_variation: float = 0.05
    ) -> OrientationResult:
        """
        Validate that quaternion measurements are consistent.

        Args:
            quaternions: List of (x, y, z, w) quaternions
            max_variation: Maximum acceptable variation in quaternion components

        Returns:
            OrientationResult for consistency test
        """
        if len(quaternions) < 2:
            return OrientationResult(
                test_name="Quaternion Consistency",
                passed=False,
                axis="all",
                expected_deg=0.0,
                actual_deg=0.0,
                error_deg=0.0,
                tolerance_deg=0.0,
                std_dev_deg=0.0,
                num_samples=len(quaternions),
                details="Need at least 2 quaternions"
            )

        quats = np.array(quaternions)

        # Normalize all quaternions
        norms = np.linalg.norm(quats, axis=1, keepdims=True)
        quats = quats / norms

        # Handle quaternion sign ambiguity (q and -q represent same rotation)
        # Flip quaternions that are in opposite hemisphere from first
        for i in range(1, len(quats)):
            if np.dot(quats[0], quats[i]) < 0:
                quats[i] = -quats[i]

        # Calculate standard deviation of each component
        std_devs = np.std(quats, axis=0)
        max_std = np.max(std_devs)

        passed = max_std < max_variation

        result = OrientationResult(
            test_name="Quaternion Consistency",
            passed=passed,
            axis="all",
            expected_deg=0.0,
            actual_deg=max_std * 180,  # Approximate angular equivalent
            error_deg=max_std * 180,
            tolerance_deg=max_variation * 180,
            std_dev_deg=max_std * 180,
            num_samples=len(quaternions),
            details=f"Max component std: {max_std:.4f}"
        )

        self.results.append(result)
        return result

    def print_report(self) -> bool:
        """Print a formatted validation report."""
        print("\n" + "=" * 70)
        print("ORIENTATION VALIDATION REPORT")
        print("=" * 70)

        all_passed = True

        for result in self.results:
            status = "PASS ✓" if result.passed else "FAIL ✗"
            all_passed = all_passed and result.passed

            print(f"\nTest: {result.test_name}")
            print(f"  Axis:       {result.axis}")
            print(f"  Expected:   {result.expected_deg:.1f}°")
            print(f"  Actual:     {result.actual_deg:.1f}°")
            print(f"  Error:      {result.error_deg:.1f}°")
            print(f"  Tolerance:  {result.tolerance_deg:.1f}°")
            print(f"  Std Dev:    {result.std_dev_deg:.2f}°")
            print(f"  Samples:    {result.num_samples}")
            print(f"  Status:     [{status}]")
            if result.details:
                print(f"  Details:    {result.details}")

        print("\n" + "=" * 70)
        overall = "ALL TESTS PASSED ✓" if all_passed else "SOME TESTS FAILED ✗"
        print(f"OVERALL: {overall}")
        print("=" * 70 + "\n")

        return all_passed

    def run_standard_tests(
        self,
        euler_samples_by_test: dict
    ) -> bool:
        """
        Run standard orientation test suite.

        Args:
            euler_samples_by_test: Dict mapping (axis, angle) to list of euler samples

        Returns:
            True if all tests pass
        """
        for test_case in self.DEFAULT_TEST_CASES:
            axis = test_case["axis"]
            tolerance = test_case["tolerance_deg"]

            for angle in test_case["angles_deg"]:
                key = (axis, angle)
                samples = euler_samples_by_test.get(key, [])
                self.validate_orientation(
                    axis=axis,
                    ground_truth_deg=angle,
                    measured_euler_samples=samples,
                    tolerance_deg=tolerance
                )

        return self.print_report()


def run_synthetic_validation():
    """Run validation with synthetic data to test the validator itself."""
    print("\n" + "=" * 70)
    print("RUNNING SYNTHETIC ORIENTATION VALIDATION")
    print("(Testing validator with simulated measurements)")
    print("=" * 70)

    validator = OrientationValidator()
    np.random.seed(42)

    # Generate synthetic samples with realistic noise
    for test_case in OrientationValidator.DEFAULT_TEST_CASES:
        axis = test_case["axis"]
        tolerance = test_case["tolerance_deg"]

        for angle in test_case["angles_deg"]:
            # Simulate measurements with Gaussian noise
            noise_std = 1.5  # 1.5 degree noise

            samples = []
            for _ in range(50):
                if axis == "roll":
                    r, p, y = angle + np.random.normal(0, noise_std), 0, 0
                elif axis == "pitch":
                    r, p, y = 0, angle + np.random.normal(0, noise_std), 0
                else:  # yaw
                    r, p, y = 0, 0, angle + np.random.normal(0, noise_std)
                samples.append((r, p, y))

            validator.validate_orientation(
                axis=axis,
                ground_truth_deg=angle,
                measured_euler_samples=samples,
                tolerance_deg=tolerance
            )

    # Add quaternion consistency test
    base_quat = (0.0, 0.0, 0.383, 0.924)  # ~45° yaw
    quats = []
    for _ in range(50):
        noise = np.random.normal(0, 0.01, 4)
        q = tuple(np.array(base_quat) + noise)
        quats.append(q)
    validator.validate_quaternion_consistency(quats)

    return validator.print_report()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Orientation Validation for Marker Tracking")
    parser.add_argument("--synthetic", action="store_true",
                        help="Run with synthetic data to test validator")

    args = parser.parse_args()

    if args.synthetic:
        success = run_synthetic_validation()
        sys.exit(0 if success else 1)
    else:
        print("Live validation requires tracker integration.")
        print("Use --synthetic for testing with simulated data.")
