#!/usr/bin/env python3
"""
Distance Validator for Multi-Camera Marker Tracking

Validates that measured distances from marker tracking match ground truth
within acceptable tolerances.

Usage:
    python -m validation.distance_validator --samples 100 --distance 2.0
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


@dataclass
class ValidationResult:
    """Result of a validation test."""
    test_name: str
    passed: bool
    expected: float
    actual: float
    error: float
    error_percent: float
    tolerance: float
    std_dev: float
    min_val: float
    max_val: float
    num_samples: int
    details: str = ""


class DistanceValidator:
    """
    Validate distance measurements from marker tracking.

    Test methodology:
    1. Place marker at known distance from camera (measured with tape/laser)
    2. Record tracked distance for N samples (N >= 100)
    3. Calculate: mean, std, min, max, error percentage
    4. Generate validation report
    """

    # Default test cases with tolerances
    DEFAULT_TEST_CASES = [
        {"actual_distance_m": 0.5, "tolerance_m": 0.02},   # 50cm ± 2cm
        {"actual_distance_m": 1.0, "tolerance_m": 0.03},   # 1m ± 3cm
        {"actual_distance_m": 2.0, "tolerance_m": 0.05},   # 2m ± 5cm
        {"actual_distance_m": 3.0, "tolerance_m": 0.08},   # 3m ± 8cm
        {"actual_distance_m": 4.0, "tolerance_m": 0.10},   # 4m ± 10cm
    ]

    def __init__(self, tracker=None):
        """
        Initialize the distance validator.

        Args:
            tracker: Optional tracker instance for live validation.
                    If None, validation uses provided samples.
        """
        self.tracker = tracker
        self.results: List[ValidationResult] = []

    def validate_distance(
        self,
        ground_truth_m: float,
        measured_samples: List[float],
        tolerance_m: Optional[float] = None
    ) -> ValidationResult:
        """
        Validate distance measurements against ground truth.

        Args:
            ground_truth_m: Ground truth distance in meters
            measured_samples: List of measured distances
            tolerance_m: Acceptable error tolerance (default: 2.5% of distance)

        Returns:
            ValidationResult with pass/fail and statistics
        """
        if len(measured_samples) == 0:
            return ValidationResult(
                test_name=f"Distance {ground_truth_m:.2f}m",
                passed=False,
                expected=ground_truth_m,
                actual=0.0,
                error=ground_truth_m,
                error_percent=100.0,
                tolerance=tolerance_m or 0.0,
                std_dev=0.0,
                min_val=0.0,
                max_val=0.0,
                num_samples=0,
                details="No samples provided"
            )

        samples = np.array(measured_samples)

        # Calculate statistics
        mean = np.mean(samples)
        std = np.std(samples)
        min_val = np.min(samples)
        max_val = np.max(samples)
        error = abs(mean - ground_truth_m)
        error_percent = (error / ground_truth_m) * 100 if ground_truth_m > 0 else 0

        # Default tolerance: 2.5% of distance or provided tolerance
        if tolerance_m is None:
            tolerance_m = ground_truth_m * 0.025

        passed = error < tolerance_m

        result = ValidationResult(
            test_name=f"Distance {ground_truth_m:.2f}m",
            passed=passed,
            expected=ground_truth_m,
            actual=mean,
            error=error,
            error_percent=error_percent,
            tolerance=tolerance_m,
            std_dev=std,
            min_val=min_val,
            max_val=max_val,
            num_samples=len(samples),
            details=f"Range: [{min_val:.4f}, {max_val:.4f}]"
        )

        self.results.append(result)
        return result

    def validate_repeatability(
        self,
        samples: List[float],
        max_std_dev: float = 0.01
    ) -> ValidationResult:
        """
        Validate measurement repeatability (consistency).

        Args:
            samples: List of repeated measurements at same position
            max_std_dev: Maximum acceptable standard deviation (default 1cm)

        Returns:
            ValidationResult for repeatability test
        """
        if len(samples) < 10:
            return ValidationResult(
                test_name="Repeatability",
                passed=False,
                expected=0.0,
                actual=0.0,
                error=0.0,
                error_percent=0.0,
                tolerance=max_std_dev,
                std_dev=0.0,
                min_val=0.0,
                max_val=0.0,
                num_samples=len(samples),
                details="Need at least 10 samples for repeatability test"
            )

        samples = np.array(samples)
        std = np.std(samples)
        mean = np.mean(samples)

        passed = std < max_std_dev

        result = ValidationResult(
            test_name="Repeatability",
            passed=passed,
            expected=0.0,  # Ideal std dev
            actual=std,
            error=std,
            error_percent=(std / mean) * 100 if mean > 0 else 0,
            tolerance=max_std_dev,
            std_dev=std,
            min_val=np.min(samples),
            max_val=np.max(samples),
            num_samples=len(samples),
            details=f"Coefficient of variation: {(std/mean)*100:.2f}%"
        )

        self.results.append(result)
        return result

    def calculate_distance_from_pose(
        self,
        marker_position: np.ndarray,
        camera_position: np.ndarray
    ) -> float:
        """
        Calculate Euclidean distance between marker and camera.

        Args:
            marker_position: [x, y, z] marker position in world frame
            camera_position: [x, y, z] camera position in world frame

        Returns:
            Distance in meters
        """
        return float(np.linalg.norm(marker_position - camera_position))

    def collect_samples_from_tracker(
        self,
        camera_id: str,
        marker_id: int,
        num_samples: int = 100,
        timeout_sec: float = 10.0
    ) -> List[float]:
        """
        Collect distance samples from a running tracker.

        Args:
            camera_id: Camera to measure from
            marker_id: Marker to track
            num_samples: Number of samples to collect
            timeout_sec: Maximum time to wait

        Returns:
            List of measured distances
        """
        if self.tracker is None:
            raise ValueError("No tracker provided for live validation")

        samples = []
        start_time = time.time()

        while len(samples) < num_samples:
            if time.time() - start_time > timeout_sec:
                print(f"Warning: Timeout after {len(samples)} samples")
                break

            # Get latest detection from tracker
            # This would integrate with the actual tracker implementation
            # For now, this is a placeholder
            detection = self._get_detection(camera_id, marker_id)

            if detection is not None:
                distance = self.calculate_distance_from_pose(
                    detection.position_world,
                    detection.camera_position
                )
                samples.append(distance)

            time.sleep(0.033)  # ~30 Hz

        return samples

    def _get_detection(self, camera_id: str, marker_id: int):
        """Get detection from tracker (placeholder for integration)."""
        # This would be implemented to interface with the actual tracker
        return None

    def print_report(self):
        """Print a formatted validation report."""
        print("\n" + "=" * 70)
        print("DISTANCE VALIDATION REPORT")
        print("=" * 70)

        all_passed = True

        for result in self.results:
            status = "PASS ✓" if result.passed else "FAIL ✗"
            all_passed = all_passed and result.passed

            print(f"\nTest: {result.test_name}")
            print(f"  Expected:   {result.expected:.4f} m")
            print(f"  Actual:     {result.actual:.4f} m")
            print(f"  Error:      {result.error:.4f} m ({result.error_percent:.2f}%)")
            print(f"  Tolerance:  {result.tolerance:.4f} m")
            print(f"  Std Dev:    {result.std_dev:.4f} m")
            print(f"  Samples:    {result.num_samples}")
            print(f"  Status:     [{status}]")
            if result.details:
                print(f"  Details:    {result.details}")

        print("\n" + "=" * 70)
        overall = "ALL TESTS PASSED ✓" if all_passed else "SOME TESTS FAILED ✗"
        print(f"OVERALL: {overall}")
        print("=" * 70 + "\n")

        return all_passed

    def run_standard_tests(self, samples_per_test: List[List[float]]) -> bool:
        """
        Run standard test suite with provided samples.

        Args:
            samples_per_test: List of sample lists, one per DEFAULT_TEST_CASES entry

        Returns:
            True if all tests pass
        """
        if len(samples_per_test) != len(self.DEFAULT_TEST_CASES):
            raise ValueError(
                f"Expected {len(self.DEFAULT_TEST_CASES)} sample sets, "
                f"got {len(samples_per_test)}"
            )

        for test_case, samples in zip(self.DEFAULT_TEST_CASES, samples_per_test):
            self.validate_distance(
                ground_truth_m=test_case["actual_distance_m"],
                measured_samples=samples,
                tolerance_m=test_case["tolerance_m"]
            )

        return self.print_report()


def run_synthetic_validation():
    """Run validation with synthetic data to test the validator itself."""
    print("\n" + "=" * 70)
    print("RUNNING SYNTHETIC DISTANCE VALIDATION")
    print("(Testing validator with simulated measurements)")
    print("=" * 70)

    validator = DistanceValidator()
    np.random.seed(42)  # For reproducibility

    # Generate synthetic samples with realistic noise
    test_results = []

    for test_case in DistanceValidator.DEFAULT_TEST_CASES:
        ground_truth = test_case["actual_distance_m"]
        tolerance = test_case["tolerance_m"]

        # Simulate measurements with Gaussian noise
        # Noise increases with distance (depth uncertainty)
        noise_std = ground_truth * 0.015  # 1.5% of distance
        samples = np.random.normal(ground_truth, noise_std, 100)

        result = validator.validate_distance(
            ground_truth_m=ground_truth,
            measured_samples=samples.tolist(),
            tolerance_m=tolerance
        )
        test_results.append(result)

    # Add repeatability test
    ground_truth = 2.0
    noise_std = 0.005  # 5mm noise
    samples = np.random.normal(ground_truth, noise_std, 100)
    validator.validate_repeatability(samples.tolist())

    return validator.print_report()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Distance Validation for Marker Tracking")
    parser.add_argument("--synthetic", action="store_true",
                        help="Run with synthetic data to test validator")
    parser.add_argument("--samples", type=int, default=100,
                        help="Number of samples per test")
    parser.add_argument("--distance", type=float,
                        help="Specific distance to validate (meters)")

    args = parser.parse_args()

    if args.synthetic:
        success = run_synthetic_validation()
        sys.exit(0 if success else 1)
    else:
        print("Live validation requires tracker integration.")
        print("Use --synthetic for testing with simulated data.")
        print("\nTo integrate with real tracker:")
        print("  1. Import DistanceValidator")
        print("  2. Pass tracker instance to constructor")
        print("  3. Call collect_samples_from_tracker()")
        print("  4. Call validate_distance() with samples")
