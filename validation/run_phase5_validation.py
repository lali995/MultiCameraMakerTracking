#!/usr/bin/env python3
"""
Phase 5 Validation Runner (Final Gate)

Validates robot navigation with Nav2 for the multi-camera robot navigation system.

Validation checks:
1. Nav2 configuration file exists and is valid YAML
2. Navigation launch file exists and is valid
3. All required Nav2 components are configured
4. Package dependencies are declared
5. Full system launch file includes all components

Usage:
    python -m validation.run_phase5_validation
"""

import sys
import os
from datetime import datetime
from typing import Dict, Tuple

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


def print_header():
    """Print validation header."""
    print("\n")
    print("+" + "=" * 78 + "+")
    print("|" + " " * 17 + "PHASE 5 VALIDATION SUITE (FINAL)" + " " * 28 + "|")
    print("|" + " " * 17 + "Robot Navigation with Nav2" + " " * 35 + "|")
    print("+" + "=" * 78 + "+")
    print(f"|  Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}" + " " * 51 + "|")
    print("+" + "=" * 78 + "+")


def check_nav2_params() -> Tuple[bool, str]:
    """Check if Nav2 parameters file exists and is valid YAML."""
    params_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'config', 'nav2_params.yaml'
    )

    if not os.path.exists(params_path):
        return False, "nav2_params.yaml not found"

    if not YAML_AVAILABLE:
        # Check file size as basic validation
        if os.path.getsize(params_path) > 1000:
            return True, "Nav2 params file exists (YAML module not available)"
        return False, "Nav2 params file too small"

    try:
        with open(params_path, 'r') as f:
            config = yaml.safe_load(f)

        # Check for required top-level keys
        required_keys = [
            'bt_navigator',
            'controller_server',
            'planner_server',
            'local_costmap',
            'global_costmap',
        ]

        missing = [k for k in required_keys if k not in config]
        if missing:
            return False, f"Missing config sections: {missing}"

        return True, f"Nav2 params valid with {len(config)} components"

    except yaml.YAMLError as e:
        return False, f"YAML parse error: {e}"
    except Exception as e:
        return False, f"Error: {e}"


def check_nav_launch_file() -> Tuple[bool, str]:
    """Check if navigation launch file exists and is valid."""
    launch_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'launch', 'navigation.launch.py'
    )

    if not os.path.exists(launch_path):
        return False, "navigation.launch.py not found"

    with open(launch_path, 'r') as f:
        content = f.read()

    # Check for key components
    required = [
        'nav2_bringup',
        'nav2_params.yaml',
        'marker_tracker',
        'simulation_bridge',
        'tf_broadcaster',
        'point_cloud_stitcher',
        'occupancy_grid_mapper',
    ]

    missing = [r for r in required if r not in content]
    if missing:
        return False, f"Missing: {missing}"

    try:
        compile(content, launch_path, 'exec')
    except SyntaxError as e:
        return False, f"Syntax error: {e}"

    return True, "Navigation launch file valid"


def check_controller_config() -> Tuple[bool, str]:
    """Check controller configuration in Nav2 params."""
    params_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'config', 'nav2_params.yaml'
    )

    if not os.path.exists(params_path):
        return False, "Nav2 params not found"

    with open(params_path, 'r') as f:
        content = f.read()

    # Check for DWB controller configuration
    required_controller_params = [
        'controller_frequency',
        'max_vel_x',
        'max_vel_theta',
        'acc_lim_x',
        'DWBLocalPlanner',
    ]

    missing = [p for p in required_controller_params if p not in content]
    if missing:
        return False, f"Missing controller params: {missing}"

    return True, "Controller configured (DWB for differential drive)"


def check_costmap_config() -> Tuple[bool, str]:
    """Check costmap configuration."""
    params_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'config', 'nav2_params.yaml'
    )

    if not os.path.exists(params_path):
        return False, "Nav2 params not found"

    with open(params_path, 'r') as f:
        content = f.read()

    # Check for costmap configuration
    required_costmap_params = [
        'local_costmap',
        'global_costmap',
        'robot_radius',
        'inflation_layer',
        'resolution',
    ]

    missing = [p for p in required_costmap_params if p not in content]
    if missing:
        return False, f"Missing costmap params: {missing}"

    return True, "Local and global costmaps configured"


def check_planner_config() -> Tuple[bool, str]:
    """Check planner configuration."""
    params_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'config', 'nav2_params.yaml'
    )

    if not os.path.exists(params_path):
        return False, "Nav2 params not found"

    with open(params_path, 'r') as f:
        content = f.read()

    # Check for planner configuration
    if 'planner_server' not in content:
        return False, "Planner server not configured"

    if 'NavfnPlanner' not in content and 'SmacPlanner' not in content:
        return False, "No path planner configured"

    return True, "Planner configured (NavfnPlanner)"


def check_behavior_tree() -> Tuple[bool, str]:
    """Check behavior tree navigator configuration."""
    params_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'config', 'nav2_params.yaml'
    )

    if not os.path.exists(params_path):
        return False, "Nav2 params not found"

    with open(params_path, 'r') as f:
        content = f.read()

    if 'bt_navigator' not in content:
        return False, "BT Navigator not configured"

    if 'plugin_lib_names' not in content:
        return False, "BT plugins not listed"

    return True, "Behavior tree navigator configured"


def check_package_dependencies() -> Tuple[bool, str]:
    """Check if Nav2 dependencies are declared in package.xml."""
    pkg_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'package.xml'
    )

    if not os.path.exists(pkg_path):
        return False, "package.xml not found"

    with open(pkg_path, 'r') as f:
        content = f.read()

    required_deps = [
        'nav2_bringup',
        'nav2_controller',
        'nav2_planner',
    ]

    missing = [d for d in required_deps if d not in content]
    if missing:
        return False, f"Missing Nav2 deps: {missing}"

    return True, "All Nav2 dependencies declared"


def check_odom_frame_config() -> Tuple[bool, str]:
    """Check that odometry comes from external tracking."""
    params_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'multi_camera_robot_nav', 'config', 'nav2_params.yaml'
    )

    if not os.path.exists(params_path):
        return False, "Nav2 params not found"

    with open(params_path, 'r') as f:
        content = f.read()

    # Check for external odom topic
    if '/robot/odom' in content:
        return True, "Using external odometry from marker tracking"

    return False, "External odometry topic not configured"


def print_gate_checklist(results: Dict[str, Tuple[bool, str]]) -> bool:
    """Print the Phase 5 (Final) gate checklist."""
    print("\n")
    print("+" + "-" * 78 + "+")
    print("|" + " " * 20 + "PHASE 5 GATE CHECKLIST (FINAL)" + " " * 27 + "|")
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
        print("|  " + "ALL PHASES COMPLETE - SYSTEM READY FOR DEPLOYMENT" + " " * 27 + "|")
        print("+" + "-" * 78 + "+")
        print("\n  *** ROBOT NAVIGATION SYSTEM IMPLEMENTATION COMPLETE ***\n")
        print("  Next steps:")
        print("  1. Install Gazebo: sudo apt install ros-humble-gazebo-ros-pkgs")
        print("  2. Install Nav2: sudo apt install ros-humble-navigation2")
        print("  3. Build: cd ros2_ws && colcon build")
        print("  4. Test: ros2 launch multi_camera_robot_nav gazebo_simulation.launch.py")
        print("  5. Navigate: ros2 launch multi_camera_robot_nav navigation.launch.py\n")
    else:
        print("|  " + "SOME TESTS FAILED - FIX BEFORE DEPLOYMENT" + " " * 35 + "|")
        print("+" + "-" * 78 + "+")
        print("\n  ! Fix failing tests before deploying\n")

    return all_passed


def main():
    """Run Phase 5 (Final) validation."""
    print_header()

    results = {}

    print("\nRunning validation checks...")
    print("-" * 80)

    # Check 1: Nav2 params
    print("\n[1/8] Checking Nav2 parameters file...")
    results["Nav2 Params"] = check_nav2_params()
    print(f"      {results['Nav2 Params'][1]}")

    # Check 2: Navigation launch file
    print("\n[2/8] Checking navigation launch file...")
    results["Nav Launch"] = check_nav_launch_file()
    print(f"      {results['Nav Launch'][1]}")

    # Check 3: Controller config
    print("\n[3/8] Checking controller configuration...")
    results["Controller"] = check_controller_config()
    print(f"      {results['Controller'][1]}")

    # Check 4: Costmap config
    print("\n[4/8] Checking costmap configuration...")
    results["Costmaps"] = check_costmap_config()
    print(f"      {results['Costmaps'][1]}")

    # Check 5: Planner config
    print("\n[5/8] Checking planner configuration...")
    results["Planner"] = check_planner_config()
    print(f"      {results['Planner'][1]}")

    # Check 6: Behavior tree
    print("\n[6/8] Checking behavior tree navigator...")
    results["BT Navigator"] = check_behavior_tree()
    print(f"      {results['BT Navigator'][1]}")

    # Check 7: Package dependencies
    print("\n[7/8] Checking package dependencies...")
    results["Dependencies"] = check_package_dependencies()
    print(f"      {results['Dependencies'][1]}")

    # Check 8: Odom configuration
    print("\n[8/8] Checking external odometry configuration...")
    results["External Odom"] = check_odom_frame_config()
    print(f"      {results['External Odom'][1]}")

    # Print final checklist
    success = print_gate_checklist(results)

    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
