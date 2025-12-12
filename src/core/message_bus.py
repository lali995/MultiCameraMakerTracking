"""
Thread-safe pub/sub message bus.
Designed to be compatible with ROS topic patterns for future migration.
"""
import threading
from queue import Queue, Empty
from typing import Any, Callable, Dict, List, Optional
from dataclasses import dataclass
import time


@dataclass
class Message:
    """Wrapper for messages with metadata."""
    topic: str
    data: Any
    timestamp: float


class Publisher:
    """Publishes messages to a topic."""

    def __init__(self, bus: 'MessageBus', topic: str, msg_type: type):
        self._bus = bus
        self._topic = topic
        self._msg_type = msg_type

    def publish(self, message: Any) -> None:
        """Publish a message to the topic."""
        self._bus._publish(self._topic, message)


class Subscription:
    """Represents an active subscription."""

    def __init__(self, topic: str, callback: Callable[[Any], None], subscription_id: int):
        self.topic = topic
        self.callback = callback
        self.id = subscription_id
        self.active = True

    def unsubscribe(self) -> None:
        """Deactivate this subscription."""
        self.active = False


class MessageBus:
    """
    Thread-safe pub/sub message broker.

    Usage mirrors ROS topics:
        bus = MessageBus()
        pub = bus.create_publisher('/markers/poses', MarkerPose)
        bus.subscribe('/markers/poses', callback_fn)
        pub.publish(MarkerPose(...))
        bus.spin_once()  # Process callbacks in main thread
    """

    _instance: Optional['MessageBus'] = None
    _lock = threading.Lock()

    def __new__(cls) -> 'MessageBus':
        """Singleton pattern for global message bus."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True

        self._topics: Dict[str, type] = {}
        self._subscriptions: Dict[str, List[Subscription]] = {}
        self._pending_callbacks: Queue = Queue()
        self._subscription_counter = 0
        self._lock = threading.Lock()

    def create_publisher(self, topic: str, msg_type: type) -> Publisher:
        """
        Create a publisher for a topic.

        Args:
            topic: Topic name (e.g., '/markers/poses')
            msg_type: Message type class

        Returns:
            Publisher instance
        """
        with self._lock:
            if topic in self._topics:
                if self._topics[topic] != msg_type:
                    raise ValueError(f"Topic {topic} already exists with different type")
            else:
                self._topics[topic] = msg_type
                self._subscriptions[topic] = []

        return Publisher(self, topic, msg_type)

    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> Subscription:
        """
        Subscribe to a topic.

        Args:
            topic: Topic name to subscribe to
            callback: Function to call when message received

        Returns:
            Subscription object (can be used to unsubscribe)
        """
        with self._lock:
            if topic not in self._subscriptions:
                self._subscriptions[topic] = []

            self._subscription_counter += 1
            sub = Subscription(topic, callback, self._subscription_counter)
            self._subscriptions[topic].append(sub)

        return sub

    def _publish(self, topic: str, message: Any) -> None:
        """Internal publish method called by Publisher."""
        with self._lock:
            subscribers = self._subscriptions.get(topic, [])
            for sub in subscribers:
                if sub.active:
                    # Queue callback for main thread processing
                    self._pending_callbacks.put((sub.callback, message))

    def spin_once(self, timeout: float = 0.0) -> int:
        """
        Process pending callbacks.
        Call this from the main thread (required for Open3D).

        Args:
            timeout: Max time to wait for messages (0 = non-blocking)

        Returns:
            Number of callbacks processed
        """
        processed = 0
        deadline = time.time() + timeout if timeout > 0 else 0

        while True:
            try:
                if timeout == 0:
                    # Non-blocking mode
                    callback, message = self._pending_callbacks.get_nowait()
                else:
                    wait_time = max(0, deadline - time.time())
                    if wait_time <= 0:
                        break
                    callback, message = self._pending_callbacks.get(timeout=wait_time)
                callback(message)
                processed += 1
                self._pending_callbacks.task_done()
            except Empty:
                break
            except Exception:
                break

        return processed

    def get_topics(self) -> List[str]:
        """Return list of registered topics."""
        with self._lock:
            return list(self._topics.keys())

    def reset(self) -> None:
        """Reset the message bus (for testing)."""
        with self._lock:
            self._topics.clear()
            self._subscriptions.clear()
            while not self._pending_callbacks.empty():
                try:
                    self._pending_callbacks.get_nowait()
                except Empty:
                    break


def get_message_bus() -> MessageBus:
    """Get the global message bus instance."""
    return MessageBus()
