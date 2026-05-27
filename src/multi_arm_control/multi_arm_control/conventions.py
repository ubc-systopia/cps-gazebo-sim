"""Naming conventions baked into create-ros-pkg generated packages.

This is the single source of truth for how arm keys (e.g. "robot1") map to ROS
names. The package generator (scripts/create-ros-pkg + scripts/helpers/) and this
library MUST agree on these patterns; if the generator's naming changes, update
it here too or generated packages will desync from the API.
"""

# UR joints, in order. Full joint name = f"{arm}_{suffix}_joint".
JOINT_SUFFIXES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
)

# Composite SRDF group spanning all arms (emitted by generate_srdf.sh for >=2 arms).
COMBINED_GROUP = "all_arms"

# Named joint-space configurations [pan, lift, elbow, wrist_1, wrist_2, wrist_3] (rad).
# All-zeros = arm stretched straight OUT (horizontal) -> sweeps near the floor.
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# shoulder_lift = -pi/2 lifts the upper arm vertical -> arm stands upright, floor-clear.
UPRIGHT_JOINTS = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]


def joint_names(arm):
    """Full ordered joint names for an arm key."""
    return [f"{arm}_{suffix}_joint" for suffix in JOINT_SUFFIXES]


def planning_group(arm):
    """MoveIt planning group for a single arm (== the arm key)."""
    return arm


def tip_link(arm):
    """End-effector / goal link for pose targets."""
    return f"{arm}_tool0"


def base_frame(arm):
    """Frame that per-arm Cartesian goals are expressed in (base-relative)."""
    return f"{arm}_base_link"


def controller_action(arm):
    """FollowJointTrajectory action for an arm's joint_trajectory_controller."""
    return f"/{arm}_joint_trajectory_controller/follow_joint_trajectory"
