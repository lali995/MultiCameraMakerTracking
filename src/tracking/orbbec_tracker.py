"""
Orbbec Camera ArUco Marker Tracker

Real ArUco marker tracking using Orbbec RGB-D camera.
Uses pyorbbecsdk for camera access and OpenCV for marker detection.
"""

import time
import threading
from typing import Optional, Dict, List, Tuple, Callable
import numpy as np

try:
    import cv2
    from cv2 import aruco
except ImportError:
    raise ImportError("OpenCV with ArUco support required. Install with: pip install opencv-contrib-python")

try:
    from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat, OBAlignMode
    from pyorbbecsdk import FrameSet, VideoStreamProfile
    ORBBEC_AVAILABLE = True
except ImportError:
    ORBBEC_AVAILABLE = False
    print("Warning: pyorbbecsdk not available. Install with: pip install pyorbbecsdk")

from .tracker_base import TrackerBase
from ..core.data_types import MarkerPose
from ..core.message_bus import MessageBus


# ArUco dictionary mapping
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
}


class OrbbecArucoTracker(TrackerBase):
    """
    ArUco marker tracker using Orbbec RGB-D camera.

    Detects ArUco markers in the color stream and uses depth
    to get 3D world coordinates.
    """

    def __init__(self,
                 message_bus: MessageBus,
                 marker_size_m: float = 0.05,
                 dictionary_type: str = "DICT_4X4_50",
                 rate_hz: float = 30.0,
                 color_width: int = 1280,
                 color_height: int = 720,
                 depth_width: int = 640,
                 depth_height: int = 480,
                 enable_depth_align: bool = True,
                 camera_matrix: Optional[np.ndarray] = None,
                 dist_coeffs: Optional[np.ndarray] = None,
                 extrinsic_matrix: Optional[np.ndarray] = None):
        """
        Initialize Orbbec ArUco tracker.

        Args:
            message_bus: Message bus for publishing marker poses
            marker_size_m: Physical size of markers in meters
            dictionary_type: ArUco dictionary to use
            rate_hz: Target tracking rate in Hz
            color_width: Color stream width
            color_height: Color stream height
            depth_width: Depth stream width
            depth_height: Depth stream height
            enable_depth_align: Align depth to color frame
            camera_matrix: Camera intrinsic matrix (3x3). If None, uses camera defaults.
            dist_coeffs: Distortion coefficients. If None, uses camera defaults.
            extrinsic_matrix: Camera to world transform (4x4). If None, uses identity.
        """
        super().__init__(message_bus, rate_hz)

        if not ORBBEC_AVAILABLE:
            raise RuntimeError("pyorbbecsdk is not available. Please install it.")

        self.marker_size_m = marker_size_m
        self.dictionary_type = dictionary_type
        self.color_width = color_width
        self.color_height = color_height
        self.depth_width = depth_width
        self.depth_height = depth_height
        self.enable_depth_align = enable_depth_align

        # Camera parameters
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        self.extrinsic_matrix = extrinsic_matrix if extrinsic_matrix is not None else np.eye(4)

        # ArUco detector
        if dictionary_type not in ARUCO_DICT:
            raise ValueError(f"Unknown dictionary: {dictionary_type}")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dictionary_type])

        # API differs between OpenCV versions
        if hasattr(cv2.aruco, 'ArucoDetector'):
            # OpenCV 4.7+
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self._use_new_api = True
        else:
            # OpenCV 4.5.x and earlier
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_detector = None
            self._use_new_api = False

        # Orbbec pipeline
        self.pipeline: Optional[Pipeline] = None
        self.config: Optional[Config] = None

        # Depth scale (mm to meters)
        self.depth_scale = 0.001

        # Frame callback
        self.frame_callback: Optional[Callable] = None

        # Latest frames for external access
        self._latest_color_frame: Optional[np.ndarray] = None
        self._latest_depth_frame: Optional[np.ndarray] = None
        self._latest_detections: List[Dict] = []
        self._frame_lock = threading.Lock()

    def _initialize_camera(self) -> bool:
        """Initialize Orbbec camera pipeline."""
        try:
            self.pipeline = Pipeline()
            self.config = Config()

            # Configure color stream
            try:
                color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                color_profile = None
                for i in range(color_profiles.get_count()):
                    profile = color_profiles.get_video_stream_profile(i)
                    if (profile.get_width() == self.color_width and
                        profile.get_height() == self.color_height and
                        profile.get_format() == OBFormat.RGB):
                        color_profile = profile
                        break

                if color_profile is None:
                    # Try to find any RGB profile
                    for i in range(color_profiles.get_count()):
                        profile = color_profiles.get_video_stream_profile(i)
                        if profile.get_format() == OBFormat.RGB:
                            color_profile = profile
                            self.color_width = profile.get_width()
                            self.color_height = profile.get_height()
                            print(f"Using color profile: {self.color_width}x{self.color_height}")
                            break

                if color_profile:
                    self.config.enable_stream(color_profile)
            except Exception as e:
                print(f"Warning: Could not configure color stream: {e}")
                return False

            # Configure depth stream
            try:
                depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
                depth_profile = None
                for i in range(depth_profiles.get_count()):
                    profile = depth_profiles.get_video_stream_profile(i)
                    if (profile.get_width() == self.depth_width and
                        profile.get_height() == self.depth_height and
                        profile.get_format() == OBFormat.Y16):
                        depth_profile = profile
                        break

                if depth_profile is None:
                    # Try to find any Y16 depth profile
                    for i in range(depth_profiles.get_count()):
                        profile = depth_profiles.get_video_stream_profile(i)
                        if profile.get_format() == OBFormat.Y16:
                            depth_profile = profile
                            self.depth_width = profile.get_width()
                            self.depth_height = profile.get_height()
                            print(f"Using depth profile: {self.depth_width}x{self.depth_height}")
                            break

                if depth_profile:
                    self.config.enable_stream(depth_profile)
            except Exception as e:
                print(f"Warning: Could not configure depth stream: {e}")

            # Enable depth to color alignment
            if self.enable_depth_align:
                self.config.set_align_mode(OBAlignMode.HW_MODE)

            # Start pipeline
            self.pipeline.start(self.config)

            # Get camera intrinsics if not provided
            if self.camera_matrix is None:
                self._get_camera_intrinsics()

            print("Orbbec camera initialized successfully")
            return True

        except Exception as e:
            print(f"Failed to initialize Orbbec camera: {e}")
            return False

    def _get_camera_intrinsics(self):
        """Get camera intrinsic parameters from the device."""
        try:
            camera_param = self.pipeline.get_camera_param()

            # Use color intrinsics for ArUco detection
            fx = camera_param.rgb_intrinsic.fx
            fy = camera_param.rgb_intrinsic.fy
            cx = camera_param.rgb_intrinsic.cx
            cy = camera_param.rgb_intrinsic.cy

            self.camera_matrix = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ], dtype=np.float64)

            # Get distortion coefficients
            dist = camera_param.rgb_distortion
            self.dist_coeffs = np.array([
                dist.k1, dist.k2, dist.p1, dist.p2, dist.k3
            ], dtype=np.float64)

            print(f"Camera intrinsics loaded: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

        except Exception as e:
            print(f"Warning: Could not get camera intrinsics: {e}")
            # Use default intrinsics
            self.camera_matrix = np.array([
                [self.color_width, 0, self.color_width / 2],
                [0, self.color_width, self.color_height / 2],
                [0, 0, 1]
            ], dtype=np.float64)

    def start(self):
        """Start the tracker."""
        if not self._initialize_camera():
            raise RuntimeError("Failed to initialize camera")
        super().start()

    def stop(self):
        """Stop the tracker and release camera."""
        super().stop()
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
            self.pipeline = None

    def _tracking_loop(self):
        """Main tracking loop - captures frames and detects markers."""
        while self._running:
            try:
                # Wait for frames
                frames: FrameSet = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                # Get color frame
                color_frame = frames.get_color_frame()
                if color_frame is None:
                    continue

                # Get depth frame
                depth_frame = frames.get_depth_frame()

                # Convert color frame to numpy
                color_data = np.asarray(color_frame.get_data())
                color_image = color_data.reshape((color_frame.get_height(),
                                                  color_frame.get_width(), 3))
                # Convert RGB to BGR for OpenCV
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                # Convert depth frame to numpy if available
                depth_image = None
                if depth_frame:
                    depth_data = np.asarray(depth_frame.get_data())
                    depth_image = depth_data.reshape((depth_frame.get_height(),
                                                      depth_frame.get_width())).astype(np.uint16)

                # Detect markers
                detections = self._detect_markers(color_image, depth_image)

                # Store latest frames
                with self._frame_lock:
                    self._latest_color_frame = color_image.copy()
                    self._latest_depth_frame = depth_image.copy() if depth_image is not None else None
                    self._latest_detections = detections

                # Publish detected markers
                for detection in detections:
                    marker_pose = MarkerPose(
                        marker_id=detection['id'],
                        x=detection['position'][0],
                        y=detection['position'][1],
                        z=detection['position'][2],
                        timestamp=time.time(),
                        orientation=detection.get('orientation'),
                        confidence=detection.get('confidence', 1.0)
                    )
                    self._message_bus.publish('/markers/poses', marker_pose)

                # Call frame callback if registered
                if self.frame_callback:
                    self.frame_callback(color_image, depth_image, detections)

                # Rate limiting
                self._rate_limiter.sleep()

            except Exception as e:
                print(f"Tracking error: {e}")
                time.sleep(0.1)

    def _detect_markers(self, color_image: np.ndarray,
                       depth_image: Optional[np.ndarray]) -> List[Dict]:
        """
        Detect ArUco markers in the color image.

        Args:
            color_image: BGR color image
            depth_image: Depth image (optional, for 3D position)

        Returns:
            List of detection dictionaries with id, position, orientation
        """
        detections = []

        # Convert to grayscale for detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect markers (API differs between OpenCV versions)
        if self._use_new_api:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if ids is None:
            return detections

        # Estimate pose for each marker
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]

            # Get 3D position
            if self.camera_matrix is not None:
                # Use solvePnP for pose estimation
                obj_points = np.array([
                    [-self.marker_size_m / 2, self.marker_size_m / 2, 0],
                    [self.marker_size_m / 2, self.marker_size_m / 2, 0],
                    [self.marker_size_m / 2, -self.marker_size_m / 2, 0],
                    [-self.marker_size_m / 2, -self.marker_size_m / 2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, marker_corners,
                    self.camera_matrix, self.dist_coeffs
                )

                if success:
                    # Position in camera frame
                    position_camera = tvec.flatten()

                    # Transform to world frame
                    position_world = self._camera_to_world(position_camera)

                    # Get orientation quaternion from rotation vector
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

                    # Transform orientation to world frame
                    quaternion_world = self._transform_orientation(quaternion)

                    detections.append({
                        'id': int(marker_id),
                        'position': position_world,
                        'orientation': quaternion_world,
                        'corners': marker_corners,
                        'rvec': rvec,
                        'tvec': tvec,
                        'confidence': 1.0
                    })

            else:
                # Fallback: use depth image center point
                center = marker_corners.mean(axis=0).astype(int)

                if depth_image is not None:
                    # Get depth at marker center
                    depth_value = depth_image[
                        min(center[1], depth_image.shape[0] - 1),
                        min(center[0], depth_image.shape[1] - 1)
                    ]
                    z = depth_value * self.depth_scale
                else:
                    z = 1.0  # Default distance

                # Simple 3D estimation
                x = (center[0] - self.color_width / 2) * z / self.color_width
                y = (center[1] - self.color_height / 2) * z / self.color_height

                position_world = self._camera_to_world(np.array([x, y, z]))

                detections.append({
                    'id': int(marker_id),
                    'position': position_world,
                    'orientation': (0, 0, 0, 1),  # Identity quaternion
                    'corners': marker_corners,
                    'confidence': 0.5  # Lower confidence without proper pose estimation
                })

        return detections

    def _camera_to_world(self, position_camera: np.ndarray) -> Tuple[float, float, float]:
        """Transform position from camera frame to world frame."""
        # Create homogeneous coordinate
        pos_homogeneous = np.append(position_camera, 1)

        # Apply extrinsic transformation
        pos_world = self.extrinsic_matrix @ pos_homogeneous

        return (pos_world[0], pos_world[1], pos_world[2])

    def _transform_orientation(self, quaternion: Tuple[float, float, float, float]
                               ) -> Tuple[float, float, float, float]:
        """Transform orientation quaternion from camera to world frame."""
        # Extract rotation from extrinsic matrix
        R_world = self.extrinsic_matrix[:3, :3]

        # Convert quaternion to rotation matrix
        q = quaternion
        R_camera = self._quaternion_to_rotation_matrix(q)

        # Combine rotations
        R_combined = R_world @ R_camera

        # Convert back to quaternion
        return self._rotation_matrix_to_quaternion(R_combined)

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
        """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return (x, y, z, w)

    @staticmethod
    def _quaternion_to_rotation_matrix(q: Tuple[float, float, float, float]) -> np.ndarray:
        """Convert quaternion (x, y, z, w) to 3x3 rotation matrix."""
        x, y, z, w = q

        R = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

        return R

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], List[Dict]]:
        """
        Get the latest captured frame and detections.

        Returns:
            Tuple of (color_image, depth_image, detections)
        """
        with self._frame_lock:
            return (
                self._latest_color_frame.copy() if self._latest_color_frame is not None else None,
                self._latest_depth_frame.copy() if self._latest_depth_frame is not None else None,
                self._latest_detections.copy()
            )

    def set_frame_callback(self, callback: Callable):
        """
        Set callback for each processed frame.

        Args:
            callback: Function(color_image, depth_image, detections)
        """
        self.frame_callback = callback

    def draw_detections(self, image: np.ndarray,
                       detections: List[Dict],
                       draw_axes: bool = True) -> np.ndarray:
        """
        Draw detected markers on image.

        Args:
            image: BGR image to draw on
            detections: List of detection dictionaries
            draw_axes: Whether to draw coordinate axes

        Returns:
            Image with detections drawn
        """
        result = image.copy()

        for detection in detections:
            corners = detection['corners'].reshape((4, 2)).astype(int)

            # Draw marker boundary
            cv2.polylines(result, [corners], True, (0, 255, 0), 2)

            # Draw marker ID
            center = corners.mean(axis=0).astype(int)
            cv2.putText(result, f"ID: {detection['id']}",
                       (center[0] - 30, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Draw position
            pos = detection['position']
            cv2.putText(result, f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                       (center[0] - 60, center[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            # Draw coordinate axes
            if draw_axes and 'rvec' in detection and 'tvec' in detection:
                cv2.drawFrameAxes(result, self.camera_matrix, self.dist_coeffs,
                                 detection['rvec'], detection['tvec'],
                                 self.marker_size_m * 0.5)

        return result


def create_orbbec_tracker(message_bus: MessageBus,
                         marker_size_m: float = 0.05,
                         dictionary_type: str = "DICT_4X4_50",
                         **kwargs) -> OrbbecArucoTracker:
    """
    Factory function to create Orbbec ArUco tracker.

    Args:
        message_bus: Message bus for publishing poses
        marker_size_m: Physical marker size in meters
        dictionary_type: ArUco dictionary type
        **kwargs: Additional tracker options

    Returns:
        Configured OrbbecArucoTracker instance
    """
    return OrbbecArucoTracker(
        message_bus=message_bus,
        marker_size_m=marker_size_m,
        dictionary_type=dictionary_type,
        **kwargs
    )
