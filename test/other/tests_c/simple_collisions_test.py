# tests/test_collision.py
import pytest
from tests_b.collision_detection import RealisticCollisionTester

# Define your three scenarios:
# name, arm1_positions, arm2_positions, expected_collision, expected_inter, expected_intra
SCENARIOS = [
    (
        "Starting position (safe)",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        False, 0, 0,
    ),
    (
        "Arm 1 neutral (safe)",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        False, 0, 0,
    ),
    (
        "Arm 2 neutral (collision)",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        True,  1, 0,
    ),
]

@pytest.fixture
def tester():
    # Assumes rclpy.init() / shutdown happen in conftest.py,
    # and that MoveIt's state_validity service is up & running.
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
    has_collision, inter_arm, intra_arm, _ = await tester.check_collision(arm1, arm2)
    assert has_collision == expect_collision, (
        f"{name}: expected collision={expect_collision}, got {has_collision}"
    )
    assert inter_arm == expect_inter, (
        f"{name}: expected inter_arm={expect_inter}, got {inter_arm}"
    )
    assert intra_arm == expect_intra, (
        f"{name}: expected intra_arm={expect_intra}, got {intra_arm}"
    )
