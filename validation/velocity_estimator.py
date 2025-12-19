#!/usr/bin/env python3
"""
Velocity Estimator for Multi-Camera Marker Tracking

Estimates linear and angular velocity from pose history.
This module can be used both for validation and for robot navigation.

Usage:
    python -m validation.velocity_estimator --synthetic
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple, Deque
from collections import deque
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


@dataclass
class PoseStamped:
    """A timestamped pose observation."""
    timestamp: float
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # quaternion [w, x, y, z]


@dataclass
class VelocityEstimate:
    """Estimated velocity."""
    timestamp: float
    linear: np.ndarray  # [vx, vy, vz] in m/s
    angular: np.ndarray  # [wx, wy, wz] in rad/s
    linear_speed: float  # magnitude of linear velocity
    angular_speed: float  # magnitude of angular velocity
    confidence: float  # 0-1, based on sample consistency


@dataclass
class VelocityValidationResult:
    """Result of velocity validation."""
    test_name: str
    passed: bool
    expected_speed: float
    actual_speed: float
    error: float
    error_percent: float
    tolerance: float
    num_samples: int
    details: str = ""


class VelocityEstimator:
    """
    Estimate linear and angular velocity from pose history.

    Methods:
    1. Simple differentiation: v = (p2 - p1) / dt
    2. Linear regression: Fit line to recent positions
    3. Kalman filter: Smooth estimates (optional, for future)
    """

    def __init__(
        self,
        history_size: int = 100,
        smoothing_window: int = 5,
        min_dt: float = 0.001
    ):
        """
        Initialize velocity estimator.

        Args:
            history_size: Maximum poses to store in history
            smoothing_window: Number of recent poses for velocity calculation
            min_dt: Minimum time delta to avoid division by zero
        """
        self.pose_history: Deque[PoseStamped] = deque(maxlen=history_size)
        self.smoothing_window = smoothing_window
        self.min_dt = min_dt
        self._last_velocity: Optional[VelocityEstimate] = None

    def add_pose(
        self,
        timestamp: float,
        position: np.ndarray,
        orientation: np.ndarray
    ):
        """
        Add new pose observation.

        Args:
            timestamp: Time in seconds
            position: [x, y, z] position in meters
            orientation: Quaternion [w, x, y, z]
        """
        pose = PoseStamped(
            timestamp=timestamp,
            position=np.array(position),
            orientation=np.array(orientation)
        )
        self.pose_history.append(pose)

    def get_linear_velocity(self) -> np.ndarray:
        """
        Calculate linear velocity [vx, vy, vz] in m/s.

        Uses linear regression over smoothing window for smoother estimates.

        Returns:
            Velocity vector [vx, vy, vz]
        """
        if len(self.pose_history) < 2:
            return np.zeros(3)

        # Get recent poses
        n = min(self.smoothing_window, len(self.pose_history))
        recent = list(self.pose_history)[-n:]

        if n < 2:
            return np.zeros(3)

        # Extract times and positions
        times = np.array([p.timestamp for p in recent])
        positions = np.array([p.position for p in recent])

        # Normalize times to avoid numerical issues
        t0 = times[0]
        times = times - t0

        dt = times[-1] - times[0]
        if dt < self.min_dt:
            return np.zeros(3)

        # Linear regression for each axis
        velocities = []
        for axis in range(3):
            # Fit line: position = velocity * time + intercept
            coeffs = np.polyfit(times, positions[:, axis], 1)
            velocities.append(coeffs[0])  # Slope = velocity

        return np.array(velocities)

    def get_angular_velocity(self) -> np.ndarray:
        """
        Calculate angular velocity [wx, wy, wz] in rad/s.

        Uses quaternion differentiation.

        Returns:
            Angular velocity vector [wx, wy, wz]
        """
        if len(self.pose_history) < 2:
            return np.zeros(3)

        # Get two most recent poses
        p1 = self.pose_history[-2]
        p2 = self.pose_history[-1]

        dt = p2.timestamp - p1.timestamp
        if dt < self.min_dt:
            return np.zeros(3)

        # Quaternion difference
        q1 = p1.orientation  # [w, x, y, z]
        q2 = p2.orientation

        # Compute relative rotation: q_rel = q2 * q1_conjugate
        q1_conj = np.array([q1[0], -q1[1], -q1[2], -q1[3]])
        q_rel = self._quat_multiply(q2, q1_conj)

        # Angular velocity from quaternion derivative
        # For small rotations: w ≈ 2 * q_rel[1:4] / dt
        # More accurate: convert to axis-angle
        angle, axis = self._quat_to_axis_angle(q_rel)

        angular_velocity = (angle / dt) * axis

        return angular_velocity

    def get_velocity_estimate(self) -> VelocityEstimate:
        """
        Get complete velocity estimate with confidence.

        Returns:
            VelocityEstimate with linear and angular velocities
        """
        linear = self.get_linear_velocity()
        angular = self.get_angular_velocity()

        linear_speed = float(np.linalg.norm(linear))
        angular_speed = float(np.linalg.norm(angular))

        # Calculate confidence based on consistency
        confidence = self._calculate_confidence()

        timestamp = self.pose_history[-1].timestamp if self.pose_history else 0.0

        estimate = VelocityEstimate(
            timestamp=timestamp,
            linear=linear,
            angular=angular,
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            confidence=confidence
        )

        self._last_velocity = estimate
        return estimate

    def _calculate_confidence(self) -> float:
        """
        Calculate confidence in velocity estimate.

        Based on:
        - Number of samples
        - Consistency of measurements
        - Time span of samples
        """
        if len(self.pose_history) < 3:
            return 0.3

        n = min(self.smoothing_window, len(self.pose_history))
        recent = list(self.pose_history)[-n:]

        # Time span factor (longer span = more confident, up to a point)
        dt = recent[-1].timestamp - recent[0].timestamp
        time_factor = min(1.0, dt / 0.2)  # Max confidence at 200ms span

        # Sample count factor
        sample_factor = min(1.0, n / self.smoothing_window)

        # Consistency factor (low residuals = high confidence)
        times = np.array([p.timestamp - recent[0].timestamp for p in recent])
        positions = np.array([p.position for p in recent])

        if len(times) >= 2:
            residuals = []
            for axis in range(3):
                coeffs = np.polyfit(times, positions[:, axis], 1)
                predicted = np.polyval(coeffs, times)
                residual = np.mean((positions[:, axis] - predicted) ** 2)
                residuals.append(residual)

            avg_residual = np.mean(residuals)
            consistency_factor = 1.0 / (1.0 + avg_residual * 100)
        else:
            consistency_factor = 0.5

        confidence = time_factor * sample_factor * consistency_factor
        return float(np.clip(confidence, 0.0, 1.0))

    def _quat_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        Multiply two quaternions.

        Args:
            q1, q2: Quaternions as [w, x, y, z]

        Returns:
            Product quaternion [w, x, y, z]
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def _quat_to_axis_angle(self, q: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Convert quaternion to axis-angle representation.

        Args:
            q: Quaternion [w, x, y, z]

        Returns:
            Tuple of (angle in radians, axis as unit vector)
        """
        # Normalize
        q = q / np.linalg.norm(q)

        # Handle sign ambiguity
        if q[0] < 0:
            q = -q

        w = q[0]
        xyz = q[1:4]

        # Compute angle
        angle = 2 * np.arccos(np.clip(w, -1.0, 1.0))

        # Compute axis
        sin_half = np.sqrt(1 - w*w)
        if sin_half < 1e-6:
            axis = np.array([1.0, 0.0, 0.0])  # Arbitrary for zero rotation
        else:
            axis = xyz / sin_half

        return angle, axis

    def reset(self):
        """Clear pose history."""
        self.pose_history.clear()
        self._last_velocity = None


class VelocityValidator:
    """Validate velocity estimation accuracy."""

    DEFAULT_TEST_CASES = [
        # Linear velocity tests
        {"type": "linear", "direction": "x", "speed_mps": 0.1, "tolerance_mps": 0.02},
        {"type": "linear", "direction": "x", "speed_mps": 0.5, "tolerance_mps": 0.05},
        {"type": "linear", "direction": "y", "speed_mps": 0.3, "tolerance_mps": 0.03},

        # Angular velocity tests (degrees per second)
        {"type": "angular", "axis": "yaw", "speed_dps": 30, "tolerance_dps": 5},
        {"type": "angular", "axis": "yaw", "speed_dps": 90, "tolerance_dps": 10},
    ]

    def __init__(self):
        self.results: List[VelocityValidationResult] = []

    def validate_linear_velocity(
        self,
        direction: str,
        ground_truth_mps: float,
        measured_velocities: List[np.ndarray],
        tolerance_mps: float
    ) -> VelocityValidationResult:
        """
        Validate linear velocity estimation.

        Args:
            direction: 'x', 'y', or 'z'
            ground_truth_mps: Expected speed in m/s
            measured_velocities: List of velocity vectors [vx, vy, vz]
            tolerance_mps: Acceptable error in m/s

        Returns:
            Validation result
        """
        if len(measured_velocities) == 0:
            return VelocityValidationResult(
                test_name=f"Linear velocity {direction} @ {ground_truth_mps} m/s",
                passed=False,
                expected_speed=ground_truth_mps,
                actual_speed=0.0,
                error=ground_truth_mps,
                error_percent=100.0,
                tolerance=tolerance_mps,
                num_samples=0,
                details="No samples"
            )

        axis_idx = {'x': 0, 'y': 1, 'z': 2}[direction.lower()]
        speeds = [abs(v[axis_idx]) for v in measured_velocities]

        mean_speed = np.mean(speeds)
        error = abs(mean_speed - ground_truth_mps)
        error_percent = (error / ground_truth_mps * 100) if ground_truth_mps > 0 else 0

        passed = error < tolerance_mps

        result = VelocityValidationResult(
            test_name=f"Linear velocity {direction} @ {ground_truth_mps} m/s",
            passed=passed,
            expected_speed=ground_truth_mps,
            actual_speed=mean_speed,
            error=error,
            error_percent=error_percent,
            tolerance=tolerance_mps,
            num_samples=len(speeds),
            details=f"Std: {np.std(speeds):.4f} m/s"
        )

        self.results.append(result)
        return result

    def validate_angular_velocity(
        self,
        axis: str,
        ground_truth_dps: float,
        measured_angular_velocities: List[np.ndarray],
        tolerance_dps: float
    ) -> VelocityValidationResult:
        """
        Validate angular velocity estimation.

        Args:
            axis: 'roll', 'pitch', or 'yaw'
            ground_truth_dps: Expected speed in degrees/second
            measured_angular_velocities: List of [wx, wy, wz] in rad/s
            tolerance_dps: Acceptable error in degrees/second

        Returns:
            Validation result
        """
        if len(measured_angular_velocities) == 0:
            return VelocityValidationResult(
                test_name=f"Angular velocity {axis} @ {ground_truth_dps} °/s",
                passed=False,
                expected_speed=ground_truth_dps,
                actual_speed=0.0,
                error=ground_truth_dps,
                error_percent=100.0,
                tolerance=tolerance_dps,
                num_samples=0,
                details="No samples"
            )

        axis_idx = {'roll': 0, 'pitch': 1, 'yaw': 2}[axis.lower()]

        # Convert rad/s to deg/s
        speeds_dps = [abs(np.degrees(w[axis_idx])) for w in measured_angular_velocities]

        mean_speed = np.mean(speeds_dps)
        error = abs(mean_speed - ground_truth_dps)
        error_percent = (error / ground_truth_dps * 100) if ground_truth_dps > 0 else 0

        passed = error < tolerance_dps

        result = VelocityValidationResult(
            test_name=f"Angular velocity {axis} @ {ground_truth_dps} °/s",
            passed=passed,
            expected_speed=ground_truth_dps,
            actual_speed=mean_speed,
            error=error,
            error_percent=error_percent,
            tolerance=tolerance_dps,
            num_samples=len(speeds_dps),
            details=f"Std: {np.std(speeds_dps):.2f} °/s"
        )

        self.results.append(result)
        return result

    def print_report(self) -> bool:
        """Print validation report."""
        print("\n" + "=" * 70)
        print("VELOCITY VALIDATION REPORT")
        print("=" * 70)

        all_passed = True

        for result in self.results:
            status = "PASS ✓" if result.passed else "FAIL ✗"
            all_passed = all_passed and result.passed

            print(f"\nTest: {result.test_name}")
            print(f"  Expected:   {result.expected_speed:.3f}")
            print(f"  Actual:     {result.actual_speed:.3f}")
            print(f"  Error:      {result.error:.3f} ({result.error_percent:.1f}%)")
            print(f"  Tolerance:  {result.tolerance:.3f}")
            print(f"  Samples:    {result.num_samples}")
            print(f"  Status:     [{status}]")
            if result.details:
                print(f"  Details:    {result.details}")

        print("\n" + "=" * 70)
        overall = "ALL TESTS PASSED ✓" if all_passed else "SOME TESTS FAILED ✗"
        print(f"OVERALL: {overall}")
        print("=" * 70 + "\n")

        return all_passed


def run_synthetic_validation():
    """Run validation with synthetic motion data."""
    print("\n" + "=" * 70)
    print("RUNNING SYNTHETIC VELOCITY VALIDATION")
    print("(Testing estimator with simulated motion)")
    print("=" * 70)

    np.random.seed(42)
    validator = VelocityValidator()

    # Test 1: Linear velocity in X direction at 0.5 m/s
    estimator = VelocityEstimator(smoothing_window=10)
    linear_velocities = []

    speed = 0.5  # m/s
    dt = 0.033  # 30 Hz

    for i in range(100):
        t = i * dt
        x = speed * t + np.random.normal(0, 0.002)  # Position with noise
        y = np.random.normal(0, 0.002)
        z = np.random.normal(0, 0.002)

        estimator.add_pose(
            timestamp=t,
            position=np.array([x, y, z]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0])
        )

        if i >= 10:  # Wait for buffer to fill
            vel = estimator.get_linear_velocity()
            linear_velocities.append(vel)

    validator.validate_linear_velocity("x", 0.5, linear_velocities, 0.05)

    # Test 2: Linear velocity in Y direction at 0.3 m/s
    estimator.reset()
    linear_velocities = []

    speed = 0.3

    for i in range(100):
        t = i * dt
        x = np.random.normal(0, 0.002)
        y = speed * t + np.random.normal(0, 0.002)
        z = np.random.normal(0, 0.002)

        estimator.add_pose(
            timestamp=t,
            position=np.array([x, y, z]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0])
        )

        if i >= 10:
            vel = estimator.get_linear_velocity()
            linear_velocities.append(vel)

    validator.validate_linear_velocity("y", 0.3, linear_velocities, 0.03)

    # Test 3: Angular velocity (yaw) at 30 deg/s
    estimator.reset()
    angular_velocities = []

    yaw_rate = np.radians(30)  # 30 deg/s in rad/s

    for i in range(100):
        t = i * dt
        yaw = yaw_rate * t

        # Quaternion for yaw rotation
        w = np.cos(yaw / 2)
        z = np.sin(yaw / 2)

        estimator.add_pose(
            timestamp=t,
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([w, 0.0, 0.0, z])
        )

        if i >= 10:
            ang_vel = estimator.get_angular_velocity()
            angular_velocities.append(ang_vel)

    validator.validate_angular_velocity("yaw", 30, angular_velocities, 5)

    return validator.print_report()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Velocity Estimation and Validation")
    parser.add_argument("--synthetic", action="store_true",
                        help="Run synthetic validation test")

    args = parser.parse_args()

    if args.synthetic:
        success = run_synthetic_validation()
        sys.exit(0 if success else 1)
    else:
        print("Use --synthetic for testing with simulated motion data.")
