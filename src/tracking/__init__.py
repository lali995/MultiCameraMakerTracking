from .tracker_base import TrackerBase
from .simulated_tracker import SimulatedTracker

# Orbbec tracker is optional (requires pyorbbecsdk)
try:
    from .orbbec_tracker import OrbbecArucoTracker, create_orbbec_tracker
    ORBBEC_AVAILABLE = True
except ImportError:
    ORBBEC_AVAILABLE = False
    OrbbecArucoTracker = None
    create_orbbec_tracker = None

__all__ = ['TrackerBase', 'SimulatedTracker', 'OrbbecArucoTracker',
           'create_orbbec_tracker', 'ORBBEC_AVAILABLE']
