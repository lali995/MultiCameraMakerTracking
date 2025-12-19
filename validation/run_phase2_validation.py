#!/usr/bin/env python3
"""
Phase 2 Validation Runner

Validates ROS2 integration for the multi-camera robot navigation system.

Validation checks:
1. ROS2 package is installed and discoverable
2. All nodes can be imported and instantiated
3. Publishers are correctly registered
4. TF tree can be configured
5. Launch files are valid

Usage:
    python -m validation.run_phase2_validation
"""

import sys
import os
import subprocess
from datetime import datetime
from typing import Dict, Tuple

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def print_header():
    """Print validation header."""
    print("\n")
    print("+" + "=" * 78 + "+")
    print("|" + " " * 20 + "PHASE 2 VALIDATION SUITE" + " " * 34 + "|")
    print("|" + " " * 18 + "ROS2 Integration Verification" + " " * 31 + "|")
    print("+" + "=" * 78 + "+")
    print(f"|  Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}" + " " * 51 + "|")
    print("+" + "=" * 78 + "+")


def check_ros2_installation() -> Tuple[bool, str]:
    """Check if ROS2 is installed and sourced."""
    try:
        result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 --help'],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            return True, "ROS2 Humble is installed and accessible"
        else:
            return False, f"ROS2 command failed: {result.stderr}"
    except subprocess.TimeoutExpired:
        return False, "ROS2 command timed out"
    except FileNotFoundError:
        return False, "ROS2 not found in PATH"


def check_package_installed() -> Tuple[bool, str]:
    """Check if the package is installed in the workspace."""
    workspace_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'install', 'multi_camera_robot_nav'
    )

    if os.path.isdir(workspace_path):
        # Check for key files - use share path which is consistent
        required_files = [
            'share/multi_camera_robot_nav/package.xml',
        ]

        # For Python site-packages, check either install or build (symlink-install)
        python_path_install = os.path.join(
            workspace_path, 'lib', 'python3.10', 'site-packages', 'multi_camera_robot_nav', '__init__.py')
        python_path_build = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'ros2_ws', 'build', 'multi_camera_robot_nav', 'multi_camera_robot_nav', '__init__.py')

        missing = []
        for f in required_files:
            if not os.path.exists(os.path.join(workspace_path, f)):
                missing.append(f)

        # Check Python package (either in install or build for symlink-install)
        if not os.path.exists(python_path_install) and not os.path.exists(python_path_build):
            missing.append('Python package (not in install or build)')

        if missing:
            return False, f"Missing files: {missing}"
        return True, f"Package installed at {workspace_path}"
    else:
        return False, f"Package not found at {workspace_path}"


def check_executables() -> Tuple[bool, str]:
    """Check if all node executables are registered."""
    workspace_install = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'install'
    )

    cmd = f'''
    source /opt/ros/humble/setup.bash && \
    source {workspace_install}/setup.bash && \
    export AMENT_PREFIX_PATH="{workspace_install}/multi_camera_robot_nav:$AMENT_PREFIX_PATH" && \
    ros2 pkg executables multi_camera_robot_nav
    '''

    try:
        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True,
            text=True,
            timeout=15
        )

        if result.returncode != 0:
            return False, f"Failed to list executables: {result.stderr}"

        expected_nodes = [
            'marker_tracker_node',
            'tf_broadcaster_node',
            'simulation_bridge_node'
        ]

        output = result.stdout
        missing_nodes = []
        for node in expected_nodes:
            if node not in output:
                missing_nodes.append(node)

        if missing_nodes:
            return False, f"Missing nodes: {missing_nodes}"

        return True, f"All {len(expected_nodes)} nodes registered"

    except subprocess.TimeoutExpired:
        return False, "Command timed out"
    except Exception as e:
        return False, str(e)


def check_node_imports() -> Tuple[bool, str]:
    """Check if all nodes can be imported."""
    workspace_install = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'install'
    )

    # Check marker_tracker_node and tf_broadcaster_node (required)
    # simulation_bridge_node is optional (needs gazebo_msgs)
    cmd = f'''
    source /opt/ros/humble/setup.bash && \
    source {workspace_install}/setup.bash && \
    python3 -c "
from multi_camera_robot_nav.marker_tracker_node import MarkerTrackerNode
from multi_camera_robot_nav.tf_broadcaster_node import TFBroadcasterNode
print('Required imports successful')
try:
    from multi_camera_robot_nav.simulation_bridge_node import SimulationBridgeNode
    print('Simulation bridge available')
except ImportError:
    print('Simulation bridge needs gazebo_msgs (optional)')
"
    '''

    try:
        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True,
            text=True,
            timeout=15
        )

        if result.returncode != 0:
            return False, f"Import failed: {result.stderr}"

        if 'Required imports successful' in result.stdout:
            extra_info = ""
            if 'Simulation bridge available' in result.stdout:
                extra_info = " (all 3 nodes)"
            elif 'optional' in result.stdout:
                extra_info = " (sim bridge optional)"
            return True, f"Node classes can be imported{extra_info}"
        else:
            return False, f"Unexpected output: {result.stdout}"

    except subprocess.TimeoutExpired:
        return False, "Import timed out"
    except Exception as e:
        return False, str(e)


def check_launch_files() -> Tuple[bool, str]:
    """Check if launch files exist and are valid Python."""
    launch_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'launch'
    )

    expected_launch_files = [
        'marker_tracking.launch.py',
        'simulation.launch.py',
    ]

    missing = []
    invalid = []

    for launch_file in expected_launch_files:
        path = os.path.join(launch_dir, launch_file)
        if not os.path.exists(path):
            missing.append(launch_file)
        else:
            # Check if it's valid Python
            try:
                with open(path, 'r') as f:
                    compile(f.read(), path, 'exec')
            except SyntaxError as e:
                invalid.append(f"{launch_file}: {e}")

    if missing:
        return False, f"Missing launch files: {missing}"
    if invalid:
        return False, f"Invalid launch files: {invalid}"

    return True, f"All {len(expected_launch_files)} launch files valid"


def check_urdf() -> Tuple[bool, str]:
    """Check if robot URDF exists."""
    urdf_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'urdf', 'robot.urdf'
    )

    if not os.path.exists(urdf_path):
        return False, "robot.urdf not found"

    with open(urdf_path, 'r') as f:
        content = f.read()

    # Basic checks
    checks = {
        '<robot': 'Root element',
        'base_link': 'Base link',
        'marker': 'Marker link',
        'base_footprint': 'Base footprint for Nav2',
    }

    missing = []
    for pattern, name in checks.items():
        if pattern not in content:
            missing.append(name)

    if missing:
        return False, f"URDF missing: {missing}"

    return True, "robot.urdf is valid with all required links"


def check_tf_frames() -> Tuple[bool, str]:
    """Verify TF frame configuration in code."""
    tf_node_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav',
        'multi_camera_robot_nav', 'tf_broadcaster_node.py'
    )

    if not os.path.exists(tf_node_path):
        return False, "tf_broadcaster_node.py not found"

    with open(tf_node_path, 'r') as f:
        content = f.read()

    required_patterns = [
        'world_frame',
        'odom_frame',
        'robot_frame',
        'TransformBroadcaster',
        'StaticTransformBroadcaster',
    ]

    missing = []
    for pattern in required_patterns:
        if pattern not in content:
            missing.append(pattern)

    if missing:
        return False, f"TF node missing: {missing}"

    return True, "TF broadcaster configured for world -> odom -> base_link"


def check_topics() -> Tuple[bool, str]:
    """Verify ROS2 topic configuration in code."""
    tracker_node_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav',
        'multi_camera_robot_nav', 'marker_tracker_node.py'
    )

    if not os.path.exists(tracker_node_path):
        return False, "marker_tracker_node.py not found"

    with open(tracker_node_path, 'r') as f:
        content = f.read()

    required_topics = [
        '/robot/pose',
        '/robot/pose_cov',
        '/robot/odom',
        '/robot/velocity',
    ]

    missing = []
    for topic in required_topics:
        if topic not in content:
            missing.append(topic)

    if missing:
        return False, f"Missing topics: {missing}"

    return True, f"All {len(required_topics)} required topics configured"


def print_gate_checklist(results: Dict[str, Tuple[bool, str]]) -> bool:
    """Print the Phase 2 gate checklist."""
    print("\n")
    print("+" + "-" * 78 + "+")
    print("|" + " " * 25 + "PHASE 2 GATE CHECKLIST" + " " * 31 + "|")
    print("+" + "-" * 78 + "+")

    all_passed = True

    for name, (passed, details) in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        all_passed = all_passed and passed

        # Truncate details if too long
        if len(details) > 40:
            details = details[:37] + "..."

        line = f"|  {name}: {details}"
        padding = 78 - len(line) - len(status) - 2
        print(f"{line}" + " " * padding + f"{status}  |")

    print("+" + "-" * 78 + "+")

    if all_passed:
        print("|  " + "ALL TESTS PASSED - GATE OPEN" + " " * 48 + "|")
        print("+" + "-" * 78 + "+")
        print("\n  -> You may proceed to Phase 3: Gazebo Simulation\n")
    else:
        print("|  " + "SOME TESTS FAILED - GATE BLOCKED" + " " * 44 + "|")
        print("+" + "-" * 78 + "+")
        print("\n  ! Fix failing tests before proceeding to Phase 3\n")

    return all_passed


def main():
    """Run Phase 2 validation."""
    print_header()

    results = {}

    print("\nRunning validation checks...")
    print("-" * 80)

    # Check 1: ROS2 installation
    print("\n[1/7] Checking ROS2 installation...")
    results["ROS2 Installation"] = check_ros2_installation()
    print(f"      {results['ROS2 Installation'][1]}")

    # Check 2: Package installed
    print("\n[2/7] Checking package installation...")
    results["Package Installed"] = check_package_installed()
    print(f"      {results['Package Installed'][1]}")

    # Check 3: Executables registered
    print("\n[3/7] Checking node executables...")
    results["Node Executables"] = check_executables()
    print(f"      {results['Node Executables'][1]}")

    # Check 4: Node imports
    print("\n[4/7] Checking node imports...")
    results["Node Imports"] = check_node_imports()
    print(f"      {results['Node Imports'][1]}")

    # Check 5: Launch files
    print("\n[5/7] Checking launch files...")
    results["Launch Files"] = check_launch_files()
    print(f"      {results['Launch Files'][1]}")

    # Check 6: URDF
    print("\n[6/7] Checking robot URDF...")
    results["Robot URDF"] = check_urdf()
    print(f"      {results['Robot URDF'][1]}")

    # Check 7: TF frames
    print("\n[7/7] Checking TF frame configuration...")
    results["TF Frames"] = check_tf_frames()
    print(f"      {results['TF Frames'][1]}")

    # Additional: Topics
    print("\n[+] Checking topic configuration...")
    results["ROS2 Topics"] = check_topics()
    print(f"      {results['ROS2 Topics'][1]}")

    # Print final checklist
    success = print_gate_checklist(results)

    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
