"""
Camera configuration loader.
Refactored from original main.py to load camera poses from sensor_config.json files.
Supports both legacy sensor_config.json and new calibration_result.json formats.
"""
import json
import os
import glob
from typing import List, Dict, Any, Optional
import numpy as np

from .data_types import CameraPose


class ConfigLoader:
    """
    Loads camera configurations from sensor_config.json files or calibration_result.json.
    """

    def __init__(self, config_path: str = 'config', reference_camera_id: str = None):
        """
        Initialize the config loader.

        Args:
            config_path: Path to the config directory containing SZVIDS-* folders
            reference_camera_id: Camera ID substring to use as reference (origin).
                                 All other cameras will be positioned relative to this camera.
        """
        self.config_path = config_path
        self.reference_camera_id = reference_camera_id

    def get_camera_poses(self) -> List[CameraPose]:
        """
        Load all camera poses from sensor_config.json files.

        Returns:
            List of CameraPose objects
        """
        camera_poses = []
        config_folders = glob.glob(os.path.join(self.config_path, 'SZVIDS-*'))

        for folder in config_folders:
            sensor_config_path = os.path.join(folder, 'sensor_config.json')
            if os.path.exists(sensor_config_path):
                pose = self._load_camera_pose(sensor_config_path, folder)
                if pose is not None:
                    camera_poses.append(pose)

        return camera_poses

    def _load_camera_pose(self, config_path: str, folder: str) -> Optional[CameraPose]:
        """
        Load a single camera pose from a sensor_config.json file.

        Args:
            config_path: Path to sensor_config.json
            folder: Parent folder path

        Returns:
            CameraPose object or None if loading fails
        """
        try:
            with open(config_path, 'r') as f:
                config_data = json.load(f)

            extrinsic = config_data.get('Sensor to World Extrinsic')
            if extrinsic:
                extrinsic_matrix = np.array(extrinsic).reshape(4, 4)

                # Position is the translation vector (last column)
                position = extrinsic_matrix[:3, 3]

                # Direction: the 3rd column of rotation matrix is the camera's Z-axis in world coords
                # This sensor uses OpenCV convention where camera looks down +Z
                direction = extrinsic_matrix[:3, 2]

                # Get folder name as camera identifier
                folder_name = os.path.basename(folder)

                return CameraPose(
                    camera_id=folder_name,
                    position=position,
                    direction=direction,
                    extrinsic=extrinsic_matrix
                )

        except json.JSONDecodeError:
            print(f"Error decoding JSON from {config_path}")
        except Exception as e:
            print(f"Error loading camera pose from {config_path}: {e}")

        return None

    def get_camera_intrinsics(self, camera_id: str) -> Optional[Dict[str, Any]]:
        """
        Load camera intrinsic parameters for a specific camera.

        Args:
            camera_id: Camera folder name (e.g., 'SZVIDS-250515-DB77B5-8F194B-9AF224')

        Returns:
            Dictionary with intrinsic parameters or None if not found
        """
        sensor_config_path = os.path.join(self.config_path, camera_id, 'sensor_config.json')

        if not os.path.exists(sensor_config_path):
            return None

        try:
            with open(sensor_config_path, 'r') as f:
                config_data = json.load(f)

            return {
                'color_intrinsic': config_data.get('Color Intrinsic'),
                'depth_intrinsic': config_data.get('Depth Intrinsic'),
                'depth_to_color_extrinsic': config_data.get('Depth to Color Extrinsic')
            }

        except Exception as e:
            print(f"Error loading intrinsics for {camera_id}: {e}")
            return None

    def load_from_calibration_file(self, calibration_path: str,
                                   reference_camera_substring: str = None) -> List[CameraPose]:
        """
        Load camera poses from a calibration_result.json file.

        The calibration file format contains camera extrinsics as comma-separated
        values representing 4x4 transformation matrices.

        Args:
            calibration_path: Path to calibration_result.json
            reference_camera_substring: Substring to identify reference camera.
                                        All cameras will be transformed relative to this one.

        Returns:
            List of CameraPose objects with positions relative to reference camera
        """
        if not os.path.exists(calibration_path):
            print(f"Calibration file not found: {calibration_path}")
            return []

        try:
            with open(calibration_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            print(f"Error loading calibration file: {e}")
            return []

        cameras = data.get('CalibratedCameras', [])
        if not cameras:
            print("No cameras found in calibration file")
            return []

        # Parse all camera extrinsics
        camera_data = []
        reference_idx = None

        for i, cam in enumerate(cameras):
            camera_id = cam.get('CameraID', f'Camera_{i}')
            extrinsic_str = cam.get('CameraExtrinsic', '')

            if not extrinsic_str:
                continue

            # Parse comma-separated extrinsic matrix values
            try:
                values = [float(v.strip()) for v in extrinsic_str.split(',')]
                if len(values) != 16:
                    print(f"Invalid extrinsic matrix for {camera_id}: expected 16 values, got {len(values)}")
                    continue
                extrinsic_matrix = np.array(values).reshape(4, 4)
            except ValueError as e:
                print(f"Error parsing extrinsic for {camera_id}: {e}")
                continue

            camera_data.append({
                'id': camera_id,
                'extrinsic': extrinsic_matrix
            })

            # Check if this is the reference camera
            ref_substr = reference_camera_substring or self.reference_camera_id
            if ref_substr and ref_substr in camera_id:
                reference_idx = len(camera_data) - 1
                print(f"Using {camera_id} as reference camera (origin)")

        if not camera_data:
            print("No valid cameras found in calibration file")
            return []

        # If reference camera specified but not found, warn and use identity
        reference_transform = np.eye(4)
        if reference_idx is not None:
            # Get inverse of reference camera's transform
            # This will make the reference camera be at origin
            reference_transform = np.linalg.inv(camera_data[reference_idx]['extrinsic'])

        # Create camera poses transformed relative to reference
        camera_poses = []
        for cam in camera_data:
            # Transform extrinsic relative to reference camera
            transformed_extrinsic = reference_transform @ cam['extrinsic']

            # Position is the translation vector (last column)
            position = transformed_extrinsic[:3, 3]

            # Direction: camera's Z-axis in world coords (3rd column of rotation)
            direction = transformed_extrinsic[:3, 2]

            camera_poses.append(CameraPose(
                camera_id=cam['id'],
                position=position,
                direction=direction,
                extrinsic=transformed_extrinsic
            ))

        return camera_poses
