"""multi_arm_control - reusable control API for generated multi-UR-arm packages.

Typical use from a per-package workflow script:

    import rclpy
    from multi_arm_control import ArmFleet, HOME_JOINTS, UPRIGHT_JOINTS

    rclpy.init()
    fleet = ArmFleet("diff_robots")          # discovers arms from robots.json
    fleet.move_coordinated({n: UPRIGHT_JOINTS for n in fleet.arm_names})
    fleet.arm("robot1").move_pose([0.4, 0.1, 0.4, 0.0, 1.5708, 0.0])
"""

from .conventions import (
    COMBINED_GROUP,
    HOME_JOINTS,
    JOINT_SUFFIXES,
    UPRIGHT_JOINTS,
)
from .fleet import Arm, ArmFleet, rpy_to_quat

__all__ = [
    "ArmFleet",
    "Arm",
    "rpy_to_quat",
    "HOME_JOINTS",
    "UPRIGHT_JOINTS",
    "JOINT_SUFFIXES",
    "COMBINED_GROUP",
]
