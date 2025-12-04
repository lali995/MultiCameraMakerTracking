"""
Abstract base class for marker trackers.
Defines the interface that all tracker implementations must follow.
"""
from abc import ABC, abstractmethod
from typing import List, Dict, Any
import threading

from ..core.message_bus import MessageBus, Publisher
from ..core.data_types import MarkerPose, TrackerStatus
from ..core.timing import RateController, FPSCounter


class TrackerBase(ABC):
    """
    Abstract base class for ArUco marker trackers.

    Subclasses must implement:
        - _detect_markers(): Perform marker detection (simulated or real)

    Usage:
        tracker = SimulatedTracker(bus, config)
        tracker.start()  # Starts tracking in background thread
        ...
        tracker.stop()   # Stops tracking
    """

    def __init__(self, message_bus: MessageBus, config: Dict[str, Any]):
        """
        Initialize the tracker.

        Args:
            message_bus: Message bus for publishing marker poses
            config: Configuration dictionary
        """
        self.bus = message_bus
        self.config = config

        # Default settings
        self.rate_hz = config.get('rate_hz', 30.0)
        self.topic_poses = config.get('topic_poses', '/markers/poses')
        self.topic_status = config.get('topic_status', '/tracker/status')

        # Create publishers
        self.pose_publisher = message_bus.create_publisher(self.topic_poses, MarkerPose)
        self.status_publisher = message_bus.create_publisher(self.topic_status, TrackerStatus)

        # Internal state
        self._running = False
        self._paused = False
        self._thread: threading.Thread = None
        self._fps_counter = FPSCounter()

    @abstractmethod
    def _detect_markers(self) -> List[MarkerPose]:
        """
        Perform marker detection.
        Must be implemented by subclasses.

        Returns:
            List of detected MarkerPose objects
        """
        pass

    def start(self) -> None:
        """Start the tracking loop in a background thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the tracking loop and wait for thread to finish."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def pause(self) -> None:
        """Pause marker detection (still running, but not publishing)."""
        self._paused = True

    def resume(self) -> None:
        """Resume marker detection."""
        self._paused = False

    def is_running(self) -> bool:
        """Return True if tracker is running."""
        return self._running

    def get_fps(self) -> float:
        """Return current detection FPS."""
        return self._fps_counter.get_fps()

    def _run_loop(self) -> None:
        """Main tracking loop running in background thread."""
        rate = RateController(self.rate_hz)

        while self._running:
            if not self._paused:
                # Detect markers
                poses = self._detect_markers()

                # Publish each detected marker
                for pose in poses:
                    self.pose_publisher.publish(pose)

                # Update FPS counter
                self._fps_counter.tick()

                # Publish status periodically
                status = TrackerStatus(
                    is_running=self._running,
                    fps=self._fps_counter.get_fps(),
                    num_markers_detected=len(poses)
                )
                self.status_publisher.publish(status)

            rate.sleep()

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        return False
