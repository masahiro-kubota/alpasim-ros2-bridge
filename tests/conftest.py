"""Common test fixtures for ROS2 tests."""

import uuid

import pytest

try:
    import rclpy
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


@pytest.fixture(scope="session")
def ros2_context():
    """Initialize rclpy once for the entire test session."""
    if not HAS_RCLPY:
        pytest.skip("rclpy not available")
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ros2_node(ros2_context):
    """Create a disposable ROS2 node for each test with a unique name."""
    name = f"test_node_{uuid.uuid4().hex[:8]}"
    node = rclpy.create_node(name)
    yield node
    node.destroy_node()
