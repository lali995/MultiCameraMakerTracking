"""
Timing utilities for rate control and FPS tracking.
"""
import time
from typing import Optional


class RateController:
    """
    Controls loop rate to maintain a target frequency.
    Similar to rospy.Rate in ROS.
    """

    def __init__(self, target_hz: float = 30.0):
        """
        Initialize rate controller.

        Args:
            target_hz: Target frequency in Hz
        """
        self.target_hz = target_hz
        self.period = 1.0 / target_hz
        self._last_time: Optional[float] = None

    def sleep(self) -> None:
        """
        Sleep to maintain the target rate.
        Call this at the end of each loop iteration.
        """
        current_time = time.time()

        if self._last_time is not None:
            elapsed = current_time - self._last_time
            sleep_time = self.period - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

        self._last_time = time.time()

    def reset(self) -> None:
        """Reset the rate controller."""
        self._last_time = None


class FPSCounter:
    """
    Tracks frames per second.
    """

    def __init__(self, window_size: int = 30):
        """
        Initialize FPS counter.

        Args:
            window_size: Number of frames to average over
        """
        self.window_size = window_size
        self._timestamps: list = []
        self._fps: float = 0.0

    def tick(self) -> None:
        """Record a frame. Call this once per frame."""
        current_time = time.time()
        self._timestamps.append(current_time)

        # Keep only recent timestamps
        if len(self._timestamps) > self.window_size:
            self._timestamps = self._timestamps[-self.window_size:]

        # Calculate FPS
        if len(self._timestamps) >= 2:
            time_span = self._timestamps[-1] - self._timestamps[0]
            if time_span > 0:
                self._fps = (len(self._timestamps) - 1) / time_span

    def get_fps(self) -> float:
        """Return current FPS estimate."""
        return self._fps

    def reset(self) -> None:
        """Reset the FPS counter."""
        self._timestamps.clear()
        self._fps = 0.0


class Timer:
    """Simple timer for measuring elapsed time."""

    def __init__(self):
        self._start_time: Optional[float] = None

    def start(self) -> None:
        """Start the timer."""
        self._start_time = time.time()

    def elapsed(self) -> float:
        """Return elapsed time in seconds since start."""
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time

    def elapsed_ms(self) -> float:
        """Return elapsed time in milliseconds since start."""
        return self.elapsed() * 1000.0
