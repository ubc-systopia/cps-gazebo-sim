# tests/test_collision.py
import asyncio
import pytest
from collision_detection import RealisticCollisionTester

class DummyContact:
    def __init__(self, body1, body2):
        self.body_name_1 = body1
        self.body_name_2 = body2

class DummyResponse:
    def __init__(self, valid, contacts):
        self.valid = valid
        self.contacts = contacts

class DummyFuture(asyncio.Future):
    def __init__(self, response):
        super().__init__()
        self.set_result(response)

@pytest.fixture
def tester(monkeypatch):
    # 1) Instantiate your tester (rclpy has already been init/shutdown via conftest.py)
    t = RealisticCollisionTester()

    # 2) Define your safe configurations
    safe1_arm1 = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    safe1_arm2 = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    safe2_arm1 = [0.0,  0.0,  0.0,  0.0, 0.0, 0.0]
    safe2_arm2 = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

    SAFE_CONFIGS = {
        (tuple(safe1_arm1), tuple(safe1_arm2)),
        (tuple(safe2_arm1), tuple(safe2_arm2)),
    }

    # 3) Monkey-patch call_async to return appropriate DummyResponse
    def fake_call_async(request):
        arm1 = tuple(request.robot_state.joint_state.position[:6])
        arm2 = tuple(request.robot_state.joint_state.position[6:])
        if (arm1, arm2) in SAFE_CONFIGS:
            resp = DummyResponse(valid=True, contacts=[])
        else:
            resp = DummyResponse(
                valid=False,
                contacts=[DummyContact('arm1_link','arm2_link')]
            )
        return DummyFuture(resp)

    monkeypatch.setattr(t.state_validity_client, 'call_async', fake_call_async)
    return t

# Define your three scenarios:
scenarios = [
    # name, arm1_positions, arm2_positions, expected_collision
    (
        "Starting position (safe)",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        False
    ),
    (
        "Arm 1 neutral (safe)",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        False
    ),
    (
        "Arm 2 neutral (collision)",
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        True
    ),
]

@pytest.mark.asyncio
@pytest.mark.parametrize("name,arm1,arm2,expect", scenarios)
async def test_realistic_collision(tester, name, arm1, arm2, expect):
    has_collision, inter_arm, intra_arm = await tester.check_collision(
        arm1, arm2, description=name
    )
    assert has_collision is expect, f"{name}: expected collision={expect}, got {has_collision}"
    if expect:
        assert inter_arm == 1 and intra_arm == 0
    else:
        assert inter_arm == 0 and intra_arm == 0
