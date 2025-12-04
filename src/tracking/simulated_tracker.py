"""
Simulated ArUco marker tracker for testing.
Generates realistic marker movements without actual camera input.
"""
import math
import time
import random
from typing import List, Dict, Any, Tuple

from .tracker_base import TrackerBase
from ..core.message_bus import MessageBus
from ..core.data_types import MarkerPose


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


class SimulatedTracker(TrackerBase):
    """
    Simulates ArUco marker detection with configurable motion patterns.

    Config options:
        num_markers: Number of markers to simulate (default: 3)
        motion_type: 'circular', 'linear', 'random', 'figure8' (default: 'circular')
        bounds: Dict with 'x', 'y', 'z' min/max tuples (default: reasonable room size)
        speed: Movement speed multiplier (default: 1.0)
        detection_noise: Add noise to positions (default: 0.01)
        drop_rate: Probability of "missing" a detection (default: 0.0)
    """

    def __init__(self, message_bus: MessageBus, config: Dict[str, Any]):
        super().__init__(message_bus, config)

        # Simulation settings
        self.num_markers = config.get('num_markers', 3)
        self.motion_type = config.get('motion_type', 'circular')
        self.speed = config.get('speed', 1.0)
        self.detection_noise = config.get('detection_noise', 0.01)
        self.drop_rate = config.get('drop_rate', 0.0)

        # Bounds for marker movement
        default_bounds = {
            'x': (-1.5, 1.5),
            'y': (0.2, 1.8),  # Above ground, below ceiling
            'z': (-1.5, 1.5)
        }
        self.bounds = config.get('bounds', default_bounds)

        # Initialize marker states
        self._marker_states = self._initialize_markers()
        self._start_time = time.time()

    def _initialize_markers(self) -> List[Dict[str, Any]]:
        """Initialize marker states for simulation."""
        markers = []

        for i in range(self.num_markers):
            # Spread markers around the space
            angle_offset = (2 * math.pi * i) / self.num_markers

            marker = {
                'id': i,
                'angle_offset': angle_offset,
                'radius': 0.5 + 0.3 * random.random(),  # Vary radius
                'height': 0.8 + 0.4 * random.random(),  # Vary height
                'phase': random.random() * 2 * math.pi,  # Random phase
                # For random walk
                'position': [0.0, 1.0, 0.0],
                'velocity': [0.0, 0.0, 0.0]
            }
            markers.append(marker)

        return markers

    def _detect_markers(self) -> List[MarkerPose]:
        """Generate simulated marker positions with orientation."""
        poses = []
        current_time = time.time()
        elapsed = current_time - self._start_time

        for marker in self._marker_states:
            # Check for simulated detection drop
            if random.random() < self.drop_rate:
                continue

            # Calculate position and orientation based on motion type
            if self.motion_type == 'circular':
                pos, orientation = self._circular_motion(marker, elapsed)
            elif self.motion_type == 'linear':
                pos, orientation = self._linear_motion(marker, elapsed)
            elif self.motion_type == 'figure8':
                pos, orientation = self._figure8_motion(marker, elapsed)
            elif self.motion_type == 'random':
                pos, orientation = self._random_walk(marker, elapsed)
            else:
                pos, orientation = self._circular_motion(marker, elapsed)

            # Add noise
            if self.detection_noise > 0:
                pos = [
                    p + random.gauss(0, self.detection_noise)
                    for p in pos
                ]

            # Clamp to bounds
            pos[0] = max(self.bounds['x'][0], min(self.bounds['x'][1], pos[0]))
            pos[1] = max(self.bounds['y'][0], min(self.bounds['y'][1], pos[1]))
            pos[2] = max(self.bounds['z'][0], min(self.bounds['z'][1], pos[2]))

            pose = MarkerPose(
                marker_id=marker['id'],
                x=pos[0],
                y=pos[1],
                z=pos[2],
                timestamp=current_time,
                orientation=orientation,
                confidence=0.95 + 0.05 * random.random()
            )
            poses.append(pose)

        return poses

    def _circular_motion(self, marker: Dict, elapsed: float) -> Tuple[List[float], Tuple[float, float, float, float]]:
        """Generate circular motion in XZ plane with orientation facing movement direction."""
        angle = marker['angle_offset'] + elapsed * self.speed * 0.5
        radius = marker['radius']

        x = radius * math.cos(angle)
        y = marker['height']
        z = radius * math.sin(angle)

        # Orientation: face tangent to circle (direction of movement)
        # Yaw angle is perpendicular to radius (tangent direction)
        yaw = angle + math.pi / 2
        # Add some roll/pitch variation
        roll = 0.1 * math.sin(elapsed * 2)
        pitch = 0.1 * math.cos(elapsed * 1.5)
        orientation = euler_to_quaternion(roll, pitch, yaw)

        return [x, y, z], orientation

    def _linear_motion(self, marker: Dict, elapsed: float) -> Tuple[List[float], Tuple[float, float, float, float]]:
        """Generate linear back-and-forth motion."""
        # Oscillate along X axis
        period = 4.0 / self.speed
        t = (elapsed + marker['phase']) % period
        progress = t / period

        # Triangle wave for smooth back-and-forth
        if progress < 0.5:
            x_norm = progress * 2
            facing_positive = True
        else:
            x_norm = 2 - progress * 2
            facing_positive = False

        x_range = self.bounds['x'][1] - self.bounds['x'][0]
        x = self.bounds['x'][0] + x_norm * x_range

        y = marker['height']
        z = marker['radius'] * 0.5  # Offset in Z

        # Orientation: face direction of movement
        yaw = 0 if facing_positive else math.pi
        roll = 0
        pitch = 0
        orientation = euler_to_quaternion(roll, pitch, yaw)

        return [x, y, z], orientation

    def _figure8_motion(self, marker: Dict, elapsed: float) -> Tuple[List[float], Tuple[float, float, float, float]]:
        """Generate figure-8 motion pattern."""
        angle = marker['angle_offset'] + elapsed * self.speed * 0.5
        radius = marker['radius']

        x = radius * math.sin(angle)
        y = marker['height'] + 0.2 * math.sin(angle * 2)
        z = radius * math.sin(angle) * math.cos(angle)

        # Orientation: approximate tangent direction
        # Derivative of figure-8 path
        dx = radius * math.cos(angle)
        dz = radius * (math.cos(angle) * math.cos(angle) - math.sin(angle) * math.sin(angle))
        yaw = math.atan2(dz, dx)
        roll = 0.15 * math.sin(elapsed * 3)
        pitch = 0.1 * math.sin(elapsed * 2)
        orientation = euler_to_quaternion(roll, pitch, yaw)

        return [x, y, z], orientation

    def _random_walk(self, marker: Dict, elapsed: float) -> Tuple[List[float], Tuple[float, float, float, float]]:
        """Generate random walk motion with momentum."""
        pos = marker['position']
        vel = marker['velocity']

        # Add random acceleration
        acc = [
            random.gauss(0, 0.1) * self.speed,
            random.gauss(0, 0.05) * self.speed,
            random.gauss(0, 0.1) * self.speed
        ]

        # Update velocity with damping
        damping = 0.95
        dt = 1.0 / self.rate_hz

        vel[0] = vel[0] * damping + acc[0] * dt
        vel[1] = vel[1] * damping + acc[1] * dt
        vel[2] = vel[2] * damping + acc[2] * dt

        # Clamp velocity
        max_vel = 0.5 * self.speed
        for i in range(3):
            vel[i] = max(-max_vel, min(max_vel, vel[i]))

        # Update position
        pos[0] += vel[0] * dt
        pos[1] += vel[1] * dt
        pos[2] += vel[2] * dt

        # Bounce off bounds
        if pos[0] < self.bounds['x'][0] or pos[0] > self.bounds['x'][1]:
            vel[0] *= -0.8
        if pos[1] < self.bounds['y'][0] or pos[1] > self.bounds['y'][1]:
            vel[1] *= -0.8
        if pos[2] < self.bounds['z'][0] or pos[2] > self.bounds['z'][1]:
            vel[2] *= -0.8

        marker['position'] = pos
        marker['velocity'] = vel

        # Orientation: face velocity direction
        speed = math.sqrt(vel[0]**2 + vel[2]**2)
        if speed > 0.01:
            yaw = math.atan2(vel[2], vel[0])
        else:
            yaw = marker.get('last_yaw', 0)
        marker['last_yaw'] = yaw
        roll = 0
        pitch = 0
        orientation = euler_to_quaternion(roll, pitch, yaw)

        return pos.copy(), orientation

    def set_motion_type(self, motion_type: str) -> None:
        """Change the motion type at runtime."""
        valid_types = ['circular', 'linear', 'figure8', 'random']
        if motion_type in valid_types:
            self.motion_type = motion_type
            # Reset for random walk
            if motion_type == 'random':
                for marker in self._marker_states:
                    marker['position'] = [0.0, 1.0, 0.0]
                    marker['velocity'] = [0.0, 0.0, 0.0]
