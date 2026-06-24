#!/usr/bin/env python3
"""Demo / test workflow for diff_robots, built on the multi_arm_control API.

Run WHILE the package is up (Gazebo + move_group):

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/diff_robots/scripts/workflow_demo.py

Runs labelled tests and prints a PASS/FAIL summary:
  Part 1  smoke test       - bring all arms to home (is the pipeline alive?)
  Part 2  joint positions  - simultaneous (coordinated) + sequential
  Part 3  IK (Cartesian)   - sequential + simultaneous
"""

import rclpy

from multi_arm_control import ArmFleet, HOME_JOINTS, UPRIGHT_JOINTS

PACKAGE = "diff_robots"

# Modest base-frame Cartesian target [x, y, z, roll, pitch, yaw] (metres, rad),
# kept small/high to stay reachable and clear of the floor. Tune per package
# (use move_pose_cli.py) if an arm reports a planning failure.
DEMO_POSE = [0.25, 0.0, 0.30, 0.0, 1.5708, 0.0]


def main():
    rclpy.init()
    fleet = ArmFleet(PACKAGE)
    names = fleet.arm_names
    multi = len(names) >= 2
    results = {}
    try:
        # Part 1: smoke test - bring every arm to a known home.
        results["1. smoke: home all arms"] = all(
            [fleet.move_joints(n, HOME_JOINTS) for n in names])

        # Part 2a: simultaneous, coordinated (one collision-checked plan).
        if multi:
            up = fleet.move_coordinated({n: UPRIGHT_JOINTS for n in names})
            down = fleet.move_coordinated({n: HOME_JOINTS for n in names})
            results["2a. joints simultaneous (coordinated)"] = up and down

        # Part 2b: sequential - each arm up then down, one at a time.
        seq = True
        for n in names:
            seq = fleet.move_joints(n, UPRIGHT_JOINTS) and seq
            seq = fleet.move_joints(n, HOME_JOINTS) and seq
        results["2b. joints sequential"] = seq

        # Part 3a: sequential IK - each arm to a pose, then home.
        seq_ik = True
        for n in names:
            seq_ik = fleet.move_pose(n, DEMO_POSE) and seq_ik
            seq_ik = fleet.move_joints(n, HOME_JOINTS) and seq_ik
        results["3a. IK sequential"] = seq_ik

        # Part 3b: simultaneous IK - all arms to poses as ONE coordinated,
        #          collision-checked motion (IK -> move_coordinated), then home.
        if multi:
            poses_ok = fleet.move_coordinated_poses({n: DEMO_POSE for n in names})
            home_ok = fleet.move_coordinated({n: HOME_JOINTS for n in names})
            results["3b. IK simultaneous (coordinated)"] = poses_ok and home_ok
    finally:
        log = fleet.get_logger()
        log.info("==================== demo results ====================")
        for label, ok in results.items():
            log.info(f"  [{'PASS' if ok else 'FAIL'}] {label}")
        log.info("======================================================")
        fleet.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
