# tests/test_collision.py
import pytest
from collision_detection import RealisticCollisionTester

SCENARIOS = [
    (
        "Starting position (safe)",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        False, 0, 0,
    ),
    (
        "arm1wrist_3_link - arm2_wrist_3_link (collision)",
        [0.0, -1.60, 0.0, -1.57, 1.57, 0.0],
        [0.0, -1.57, 0.0, -1.57, -1.57, 0.0],
        True,  1, 0,
    ),
    (
        "Starting position (safe)",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        False, 0, 0,
    ),
    (
        "arm1wrist_2_link - arm2_wrist_2_link",
        [0.0, -1.70, 0.0, -1.57, -1.57, 0.0],
        [0.0, -1.49, 0.0, -1.57, 1.57, 0.0],
        True, 1, 0,
    ),
    (
        "Starting position (safe)",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        False, 0, 0,
    ),
    (
        "arm1wrist_1_link - arm2_wrist_1_link",
        [0.0, -1.70, 0.0, 0.0, 0.0, 0.0],
        [0.0, -1.45, 0.0, -3.14, 0.0, 0.0],
        True, 1, 0,
    ),
    (
        "Starting position (safe)",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], # 0.0, -1.57, 0.0, -1.57, 0.0, 0.0
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], # 0.0, -1.57, 0.0, -1.57, 0.0, 0.0
        False, 0, 0,
    ),
    (
        "arm1elbow_joint_link - arm2_elbow_joint_link",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], # -0.35 -1.4 -0.7 -1.57 0.2 0.0
        [5.0, -0.5, 0.0, -1.57, 0.0, 0.0], # 4.5 -1.75 1.2 -1.57 0.0 0.0
        True, 1, 0,
    ),
]

@pytest.fixture
def tester():
    return RealisticCollisionTester()

@pytest.mark.asyncio
@pytest.mark.parametrize(
    "name, arm1, arm2, expect_collision, expect_inter, expect_intra",
    SCENARIOS,
    ids=[s[0] for s in SCENARIOS]
)
async def test_realistic_collision(
    tester, name, arm1, arm2, expect_collision, expect_inter, expect_intra
):
    has_collision, inter_arm, intra_arm, collision_details = await tester.check_collision(arm1, arm2)

    print("Collision Details: -------------------------------------------------------------------")
    print(f"{collision_details}")

    assert has_collision == expect_collision, (
        f"{name}: expected collision={expect_collision}, got {has_collision}"
    )
    assert inter_arm == expect_inter, (
        f"{name}: expected inter_arm={expect_inter}, got {inter_arm}"
    )
    assert intra_arm == expect_intra, (
        f"{name}: expected intra_arm={expect_intra}, got {intra_arm}"
    )
