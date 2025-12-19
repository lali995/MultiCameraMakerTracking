#!/usr/bin/env python3
"""
Phase 1 Validation Runner

Runs all Phase 1 validation tests:
1. Distance validation
2. Orientation validation
3. Velocity estimation validation
4. Multi-camera fusion validation

Usage:
    python -m validation.run_phase1_validation --synthetic
    python -m validation.run_phase1_validation --live (requires running tracker)
"""

import sys
import os
import argparse
from datetime import datetime

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from validation.distance_validator import DistanceValidator, run_synthetic_validation as run_distance_synthetic
from validation.orientation_validator import OrientationValidator, run_synthetic_validation as run_orientation_synthetic
from validation.velocity_estimator import VelocityValidator, run_synthetic_validation as run_velocity_synthetic
from validation.fusion_validator import FusionValidator, run_synthetic_validation as run_fusion_synthetic


def print_header():
    """Print validation header."""
    print("\n")
    print("╔" + "═" * 78 + "╗")
    print("║" + " " * 20 + "PHASE 1 VALIDATION SUITE" + " " * 34 + "║")
    print("║" + " " * 15 + "Multi-Camera Marker Tracking System" + " " * 27 + "║")
    print("╠" + "═" * 78 + "╣")
    print(f"║  Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}" + " " * 51 + "║")
    print(f"║  Mode: {'Synthetic (simulated data)' if '--synthetic' in sys.argv else 'Live (real tracker)'}" + " " * 40 + "║")
    print("╚" + "═" * 78 + "╝")


def print_gate_checklist(results: dict):
    """Print the Phase 1 gate checklist."""
    print("\n")
    print("┏" + "━" * 78 + "┓")
    print("┃" + " " * 25 + "PHASE 1 GATE CHECKLIST" + " " * 31 + "┃")
    print("┣" + "━" * 78 + "┫")

    checks = [
        ("Distance Validation", results.get("distance", False)),
        ("Orientation Validation", results.get("orientation", False)),
        ("Velocity Estimation", results.get("velocity", False)),
        ("Multi-Camera Fusion", results.get("fusion", False)),
    ]

    for name, passed in checks:
        status = "✓ PASS" if passed else "✗ FAIL"
        color_status = f"[{status}]"
        padding = 78 - len(name) - len(color_status) - 6
        print(f"┃  {name}" + " " * padding + f"{color_status}  ┃")

    print("┣" + "━" * 78 + "┫")

    all_passed = all(p for _, p in checks)
    overall = "ALL TESTS PASSED - GATE OPEN ✓" if all_passed else "SOME TESTS FAILED - GATE BLOCKED ✗"
    padding = 78 - len(overall) - 4
    print(f"┃  {overall}" + " " * padding + "┃")

    print("┗" + "━" * 78 + "┛")

    if all_passed:
        print("\n  ➜ You may proceed to Phase 2: ROS2 Integration\n")
    else:
        print("\n  ⚠ Fix failing tests before proceeding to Phase 2\n")

    return all_passed


def run_synthetic_validation():
    """Run all validation tests with synthetic data."""
    print_header()

    results = {}

    # 1. Distance Validation
    print("\n" + "─" * 80)
    print("RUNNING: Distance Validation")
    print("─" * 80)
    results["distance"] = run_distance_synthetic()

    # 2. Orientation Validation
    print("\n" + "─" * 80)
    print("RUNNING: Orientation Validation")
    print("─" * 80)
    results["orientation"] = run_orientation_synthetic()

    # 3. Velocity Estimation Validation
    print("\n" + "─" * 80)
    print("RUNNING: Velocity Estimation Validation")
    print("─" * 80)
    results["velocity"] = run_velocity_synthetic()

    # 4. Multi-Camera Fusion Validation
    print("\n" + "─" * 80)
    print("RUNNING: Multi-Camera Fusion Validation")
    print("─" * 80)
    results["fusion"] = run_fusion_synthetic()

    # Print final gate checklist
    return print_gate_checklist(results)


def run_live_validation():
    """Run validation with live tracker data."""
    print_header()

    print("\n⚠ Live validation requires:")
    print("  1. Running tracker with cameras connected")
    print("  2. Marker placed at known positions")
    print("  3. Manual measurement of ground truth distances/angles")
    print("\nThis mode is not yet fully implemented.")
    print("Use --synthetic for testing with simulated data.")

    return False


def main():
    parser = argparse.ArgumentParser(
        description="Phase 1 Validation Suite for Multi-Camera Marker Tracking"
    )
    parser.add_argument(
        "--synthetic", action="store_true",
        help="Run with synthetic data (for testing without hardware)"
    )
    parser.add_argument(
        "--live", action="store_true",
        help="Run with live tracker data (requires running system)"
    )
    parser.add_argument(
        "--gazebo", action="store_true",
        help="Run with Gazebo simulation (future implementation)"
    )

    args = parser.parse_args()

    if args.synthetic:
        success = run_synthetic_validation()
    elif args.live:
        success = run_live_validation()
    elif args.gazebo:
        print("Gazebo validation will be implemented in Phase 3")
        success = False
    else:
        print("Please specify a validation mode:")
        print("  --synthetic  : Run with simulated data")
        print("  --live       : Run with live tracker")
        print("  --gazebo     : Run with Gazebo simulation (Phase 3)")
        success = False

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
