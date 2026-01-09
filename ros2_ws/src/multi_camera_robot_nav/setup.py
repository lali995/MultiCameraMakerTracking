from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_camera_robot_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Hook files for AMENT_PREFIX_PATH
        (os.path.join('share', package_name, 'hook'),
            glob(os.path.join('hooks', '*.dsv.in'))),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # RViz config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.rviz'))),
        # URDF/mesh files
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
        # Robot model files
        (os.path.join('share', package_name, 'models', 'marker_robot'),
            glob(os.path.join('models', 'marker_robot', '*'))),
        # Textures
        (os.path.join('share', package_name, 'textures'),
            glob(os.path.join('textures', '*.png'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='determ',
    maintainer_email='determ@local',
    description='Multi-camera marker tracking for robot navigation using external cameras',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'marker_tracker_node = multi_camera_robot_nav.marker_tracker_node:main',
            'tf_broadcaster_node = multi_camera_robot_nav.tf_broadcaster_node:main',
            'simulation_bridge_node = multi_camera_robot_nav.simulation_bridge_node:main',
            'point_cloud_stitcher_node = multi_camera_robot_nav.point_cloud_stitcher_node:main',
            'occupancy_grid_mapper_node = multi_camera_robot_nav.occupancy_grid_mapper_node:main',
        ],
    },
)
