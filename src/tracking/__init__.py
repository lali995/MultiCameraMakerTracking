from .tracker_base import TrackerBase
from .simulated_tracker import SimulatedTracker
from .pose_fusion import PoseFusion
from .multi_camera_tracker import MultiCameraTracker, CameraConfig

# Orbbec tracker is optional (requires pyorbbecsdk)
try:
    from .orbbec_tracker import OrbbecArucoTracker, create_orbbec_tracker
    ORBBEC_AVAILABLE = True
except ImportError:
    ORBBEC_AVAILABLE = False
    OrbbecArucoTracker = None
    create_orbbec_tracker = None

# Network stream tracker (requires pyzmq)
try:
    from .network_stream_tracker import NetworkStreamTracker
    NETWORK_STREAMING_AVAILABLE = True
except ImportError:
    NETWORK_STREAMING_AVAILABLE = False
    NetworkStreamTracker = None

__all__ = ['TrackerBase', 'SimulatedTracker', 'OrbbecArucoTracker',
           'create_orbbec_tracker', 'ORBBEC_AVAILABLE',
           'NetworkStreamTracker', 'NETWORK_STREAMING_AVAILABLE',
           'PoseFusion', 'MultiCameraTracker', 'CameraConfig']
