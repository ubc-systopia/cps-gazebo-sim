# tests/conftest.py

import os
import sys

# 1) Make your project root importable
sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)

import pytest
import rclpy

# 2) Initialize rclpy once for the entire test session
@pytest.fixture(scope="session", autouse=True)
def ros_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()
