"""
Camera configuration loader.
Refactored from original main.py to load camera poses from sensor_config.json files.
"""
import json
import os
import glob
from typing import List, Dict, Any, Optional
import numpy as np

from .data_types import CameraPose


class ConfigLoader:
    """
    Loads camera configurations from sensor_config.json files.
    """

    def __init__(self, config_path: str = 'config'):
        """
        Initialize the config loader.

        Args:
            config_path: Path to the config directory containing SZVIDS-* folders
        """
        self.config_path = config_path

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
