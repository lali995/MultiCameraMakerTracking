#!/usr/bin/env python3
"""
Phase 3 Validation Runner

Validates Gazebo simulation environment for the multi-camera robot navigation system.

Validation checks:
1. Gazebo world file exists and is valid SDF
2. Launch file exists and is valid Python
3. URDF has Gazebo plugins
4. World has 4 depth cameras configured
5. World has obstacles for navigation testing

Usage:
    python -m validation.run_phase3_validation
"""

import sys
import os
import subprocess
import xml.etree.ElementTree as ET
from datetime import datetime
from typing import Dict, Tuple

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def print_header():
    """Print validation header."""
    print("\n")
    print("+" + "=" * 78 + "+")
    print("|" + " " * 20 + "PHASE 3 VALIDATION SUITE" + " " * 34 + "|")
    print("|" + " " * 15 + "Gazebo Simulation Environment" + " " * 34 + "|")
    print("+" + "=" * 78 + "+")
    print(f"|  Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}" + " " * 51 + "|")
    print("+" + "=" * 78 + "+")


def check_gazebo_installation() -> Tuple[bool, str]:
    """Check if Gazebo is installed."""
    try:
        result = subprocess.run(
            ['which', 'gzserver'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0 and result.stdout.strip():
            return True, f"Gazebo found at {result.stdout.strip()}"
        else:
            # Check if it's installed in ROS but not in PATH
            ros_gazebo = '/opt/ros/humble/lib/gazebo_ros'
            if os.path.isdir(ros_gazebo):
                return True, "Gazebo ROS package found (source ROS setup)"
            return False, "Install: sudo apt install ros-humble-gazebo-ros-pkgs"
    except subprocess.TimeoutExpired:
        return False, "Command timed out"
    except FileNotFoundError:
        return False, "which command not found"


def check_world_file() -> Tuple[bool, str]:
    """Check if world file exists and is valid SDF."""
    world_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'worlds', 'tracking_room.world'
    )

    if not os.path.exists(world_path):
        return False, "tracking_room.world not found"

    try:
        tree = ET.parse(world_path)
        root = tree.getroot()

        # Check for SDF format
        if root.tag != 'sdf':
            return False, "Not a valid SDF file (root is not <sdf>)"

        # Check for world element
        world = root.find('.//world')
        if world is None:
            return False, "No <world> element found"

        world_name = world.get('name')
        return True, f"Valid SDF world file ({world_name})"

    except ET.ParseError as e:
        return False, f"XML parse error: {e}"


def check_cameras_in_world() -> Tuple[bool, str]:
    """Check that world has 4 depth cameras."""
    world_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'worlds', 'tracking_room.world'
    )

    if not os.path.exists(world_path):
        return False, "World file not found"

    try:
        tree = ET.parse(world_path)
        root = tree.getroot()

        # Find all depth camera sensors
        sensors = root.findall('.//sensor[@type="depth"]')
        camera_count = len(sensors)

        if camera_count < 4:
            return False, f"Only {camera_count} depth cameras (need 4)"

        # Check camera names
        camera_names = [s.get('name', 'unnamed') for s in sensors]

        return True, f"{camera_count} depth cameras: {', '.join(camera_names)}"

    except ET.ParseError as e:
        return False, f"XML parse error: {e}"


def check_obstacles_in_world() -> Tuple[bool, str]:
    """Check that world has obstacles for navigation testing."""
    world_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'worlds', 'tracking_room.world'
    )

    if not os.path.exists(world_path):
        return False, "World file not found"

    try:
        tree = ET.parse(world_path)
        root = tree.getroot()

        # Find models that look like obstacles
        models = root.findall('.//model')
        obstacle_names = [m.get('name', '') for m in models if 'obstacle' in m.get('name', '').lower()]

        if len(obstacle_names) < 1:
            return False, "No obstacles found in world"

        return True, f"{len(obstacle_names)} obstacles: {', '.join(obstacle_names)}"

    except ET.ParseError as e:
        return False, f"XML parse error: {e}"


def check_room_structure() -> Tuple[bool, str]:
    """Check that world has room walls/boundaries."""
    world_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'worlds', 'tracking_room.world'
    )

    if not os.path.exists(world_path):
        return False, "World file not found"

    try:
        tree = ET.parse(world_path)
        root = tree.getroot()

        # Find models for walls, floor, ceiling
        models = root.findall('.//model')
        model_names = [m.get('name', '').lower() for m in models]

        required = ['floor', 'wall', 'ceiling']
        found = []
        missing = []

        for req in required:
            if any(req in name for name in model_names):
                found.append(req)
            else:
                missing.append(req)

        if missing:
            return False, f"Missing room elements: {missing}"

        wall_count = sum(1 for name in model_names if 'wall' in name)
        return True, f"Room structure: floor, ceiling, {wall_count} walls"

    except ET.ParseError as e:
        return False, f"XML parse error: {e}"


def check_gazebo_launch_file() -> Tuple[bool, str]:
    """Check if Gazebo launch file exists and is valid."""
    launch_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'launch', 'gazebo_simulation.launch.py'
    )

    if not os.path.exists(launch_path):
        return False, "gazebo_simulation.launch.py not found"

    try:
        with open(launch_path, 'r') as f:
            content = f.read()

        # Check for key components
        required = [
            'gzserver',
            'spawn_entity.py',
            'simulation_bridge',
            'tracking_room.world'
        ]

        missing = [r for r in required if r not in content]

        if missing:
            return False, f"Launch file missing: {missing}"

        # Check syntax
        compile(content, launch_path, 'exec')

        return True, "Gazebo launch file valid with all required components"

    except SyntaxError as e:
        return False, f"Syntax error: {e}"


def check_urdf_gazebo_plugins() -> Tuple[bool, str]:
    """Check if URDF has Gazebo plugins."""
    urdf_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'urdf', 'robot.urdf'
    )

    if not os.path.exists(urdf_path):
        return False, "robot.urdf not found"

    with open(urdf_path, 'r') as f:
        content = f.read()

    # Check for Gazebo elements
    required = [
        '<gazebo',
        'plugin',
        'diff_drive'
    ]

    found = [r for r in required if r in content]
    missing = [r for r in required if r not in content]

    if missing:
        return False, f"URDF missing Gazebo elements: {missing}"

    return True, f"URDF has Gazebo plugins: {', '.join(found)}"


def check_camera_ros_topics() -> Tuple[bool, str]:
    """Verify camera ROS topic configuration in world file."""
    world_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'worlds', 'tracking_room.world'
    )

    if not os.path.exists(world_path):
        return False, "World file not found"

    with open(world_path, 'r') as f:
        content = f.read()

    # Check for ROS camera plugin configuration
    required_patterns = [
        'libgazebo_ros_camera',
        'image_raw',
        'depth/image_raw',
        'points',
    ]

    found = [p for p in required_patterns if p in content]
    missing = [p for p in required_patterns if p not in content]

    if missing:
        return False, f"Missing camera config: {missing}"

    return True, "Cameras configured with RGB, depth, and point cloud topics"


def print_gate_checklist(results: Dict[str, Tuple[bool, str]]) -> bool:
    """Print the Phase 3 gate checklist."""
    print("\n")
    print("+" + "-" * 78 + "+")
    print("|" + " " * 25 + "PHASE 3 GATE CHECKLIST" + " " * 31 + "|")
    print("+" + "-" * 78 + "+")

    all_passed = True
    gazebo_missing = False
    config_passed = True

    for name, (passed, details) in results.items():
        status = "[PASS]" if passed else "[FAIL]"

        # Gazebo installation is a soft requirement - config files are what matter
        if name == "Gazebo Installation" and not passed:
            status = "[WARN]"
            gazebo_missing = True
        else:
            all_passed = all_passed and passed
            config_passed = config_passed and passed

        # Truncate details if too long
        if len(details) > 40:
            details = details[:37] + "..."

        line = f"|  {name}: {details}"
        padding = 78 - len(line) - len(status) - 2
        print(f"{line}" + " " * padding + f"{status}  |")

    print("+" + "-" * 78 + "+")

    if config_passed:
        if gazebo_missing:
            print("|  " + "CONFIG VALID - Install Gazebo to run simulation" + " " * 30 + "|")
            print("+" + "-" * 78 + "+")
            print("\n  ! Install: sudo apt install ros-humble-gazebo-ros-pkgs")
            print("  -> Configuration is valid, proceeding to Phase 4\n")
            return True
        else:
            print("|  " + "ALL TESTS PASSED - GATE OPEN" + " " * 48 + "|")
            print("+" + "-" * 78 + "+")
            print("\n  -> You may proceed to Phase 4: Point Cloud Obstacle Mapping\n")
    else:
        print("|  " + "SOME TESTS FAILED - GATE BLOCKED" + " " * 44 + "|")
        print("+" + "-" * 78 + "+")
        print("\n  ! Fix failing tests before proceeding to Phase 4\n")

    return config_passed


def main():
    """Run Phase 3 validation."""
    print_header()

    results = {}

    print("\nRunning validation checks...")
    print("-" * 80)

    # Check 1: Gazebo installation
    print("\n[1/7] Checking Gazebo installation...")
    results["Gazebo Installation"] = check_gazebo_installation()
    print(f"      {results['Gazebo Installation'][1]}")

    # Check 2: World file
    print("\n[2/7] Checking world file...")
    results["World File"] = check_world_file()
    print(f"      {results['World File'][1]}")

    # Check 3: Depth cameras
    print("\n[3/7] Checking depth cameras in world...")
    results["Depth Cameras"] = check_cameras_in_world()
    print(f"      {results['Depth Cameras'][1]}")

    # Check 4: Room structure
    print("\n[4/7] Checking room structure...")
    results["Room Structure"] = check_room_structure()
    print(f"      {results['Room Structure'][1]}")

    # Check 5: Obstacles
    print("\n[5/7] Checking obstacles...")
    results["Obstacles"] = check_obstacles_in_world()
    print(f"      {results['Obstacles'][1]}")

    # Check 6: Launch file
    print("\n[6/7] Checking Gazebo launch file...")
    results["Launch File"] = check_gazebo_launch_file()
    print(f"      {results['Launch File'][1]}")

    # Check 7: URDF Gazebo plugins
    print("\n[7/7] Checking URDF Gazebo plugins...")
    results["URDF Plugins"] = check_urdf_gazebo_plugins()
    print(f"      {results['URDF Plugins'][1]}")

    # Additional: Camera ROS topics
    print("\n[+] Checking camera ROS topic configuration...")
    results["Camera Topics"] = check_camera_ros_topics()
    print(f"      {results['Camera Topics'][1]}")

    # Print final checklist
    success = print_gate_checklist(results)

    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
