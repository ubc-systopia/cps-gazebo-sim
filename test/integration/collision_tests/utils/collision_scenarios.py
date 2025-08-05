"""
Collision test scenarios for the dual-arm robot system.
Contains predefined robot configurations for testing various inter-arm collisions.
"""

DEFAULT_COLLISION_SCENARIOS = [
    {
        'name': 'Home Positions (Safe)',
        'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': False,
        'expected_collision_count': 0,
        'duration': 1.5
    },
    {
        'name': 'Wrist3 Collision Test',
        'arm1': [0.192, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [1.343, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': True,
        'expected_collision_count': 1,
        'expected_colliding_links': [('arm1wrist_3_link', 'arm2_wrist_3_link')],
        'duration': 1.5
    },
    {
        'name': 'Home Positions (Safe)',
        'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': False,
        'expected_collision_count': 0,
        'duration': 1.5
    },
    {
        'name': 'Wrist2 Collision Test',
        'arm1': [0.401, -1.623, 0.349, -1.623, 2.234, 0.0],
        'arm2': [1.047, -1.676, 0.0, -1.571, -1.588, 0.0],
        'expected_collision': True,
        'expected_collision_count': 1,
        'expected_colliding_links': [('arm1wrist_2_link', 'arm2_wrist_2_link')],
        'duration': 1.5
    },
    {
        'name': 'Home Positions (Safe)',
        'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': False,
        'expected_collision_count': 0,
        'duration': 1.5
    },
    {
        'name': 'Wrist1 Collision Test',
        'arm1': [0.401, -1.623, 0.349, -2.566, 2.234, 0.0],
        'arm2': [1.012, -1.833, 0.0, -0.140, -1.588, 0.0],
        'expected_collision': True,
        'expected_collision_count': 1,
        'expected_colliding_links': [('arm1wrist_1_link', 'arm2_wrist_1_link')],
        'duration': 1.5
    },
    {
        'name': 'Home Positions (Safe)',
        'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': False,
        'expected_collision_count': 0,
        'duration': 1.5
    },
    {
        'name': 'Forearm Collision Test',
        'arm1': [-5.044, -1.099, 0.471, -1.571, 0.0, 0.0],
        'arm2': [-3.264, -1.222, 0.017, -1.571, 0.0, 0.0],
        'expected_collision': True,
        'expected_collision_count': 1,
        'expected_colliding_links': [('arm1forearm_link', 'arm2_forearm_link')],
        'duration': 1.5
    },
    {
        'name': 'Home Positions (Safe)',
        'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': False,
        'expected_collision_count': 0,
        'duration': 1.5
    },
    {
        'name': 'Upper arm collision test',
        'arm1': [5.131, 0.017, -0.244, -0.349, 0.0, 0.0],
        'arm2': [1.833, -2.374, 1.152, -1.343, 0.0, 0.0],
        'expected_collision': True,
        'expected_collision_count': 1,
        'expected_colliding_links': [('arm1upper_arm_link', 'arm2_upper_arm_link')],
        'duration': 1.5
    },
    {
        'name': 'Home Positions (Safe)',
        'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': False,
        'expected_collision_count': 0,
        'duration': 1.5
    },
    {
        'name': 'Upper arm collision test',
        'arm1': [5.131, 0.017, -0.244, -0.349, 0.0, 0.0],
        'arm2': [1.833, -2.374, 1.152, -1.343, 0.0, 0.0],
        'expected_collision': True,
        'expected_collision_count': 1,
        'expected_colliding_links': [('arm1upper_arm_link', 'arm2_upper_arm_link')],
        'duration': 1.5
    },
    {
        'name': 'Home Positions (Safe)',
        'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': False,
        'expected_collision_count': 0,
        'duration': 1.5
    },
    {   
        'name': 'Shoulder to Forearm Collision Test',
        'arm1': [3.805, -3.142, 0.0, -1.571, 0.0, 0.0],
        'arm2': [0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
        'expected_collision': True,
        'expected_collision_count': 1,
        'expected_colliding_links': [('arm1forearm_link', 'arm2_shoulder_link')],
        'duration': 1.5
    },
    {
        'name': 'Home Positions (Safe)',
        'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        'expected_collision': False,
        'expected_collision_count': 0,
        'duration': 1.5
    },
]