"""Tests for TrajectoryListener (Layer 2: requires rclpy).

Verifies /planning/trajectory subscription and asyncio.Event-based waiting.
"""

import asyncio

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_planning_msgs")

from autoware_planning_msgs.msg import Trajectory as AwTrajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from builtin_interfaces.msg import Duration

from alpasim_ros2_bridge.trajectory_listener import TrajectoryListener


def make_autoware_trajectory(n_points=5, start_sec=1, velocity_mps=10.0):
    """Create a test autoware_planning_msgs/Trajectory."""
    traj = AwTrajectory()
    traj.header.stamp.sec = start_sec
    traj.header.frame_id = "map"
    for i in range(n_points):
        pt = TrajectoryPoint()
        pt.time_from_start = Duration(sec=0, nanosec=i * 200_000_000)
        pt.pose.position.x = float(i) * 2.0
        pt.pose.position.y = 0.0
        pt.pose.position.z = 0.0
        pt.pose.orientation.w = 1.0
        pt.longitudinal_velocity_mps = velocity_mps
        pt.lateral_velocity_mps = 0.0
        pt.acceleration_mps2 = 0.0
        pt.heading_rate_rps = 0.0
        pt.front_wheel_angle_rad = 0.0
        traj.points.append(pt)
    return traj


class TestTrajectoryListener:

    def test_initial_state_has_no_trajectory(self, ros2_node):
        """No trajectory should be available before any message is received."""
        listener = TrajectoryListener(ros2_node)
        assert listener._trajectory is None

    def test_receives_trajectory(self, ros2_node):
        """Subscribing to an autoware Trajectory should update internal state."""
        listener = TrajectoryListener(ros2_node)

        pub = ros2_node.create_publisher(
            AwTrajectory, "/planning/trajectory", 10
        )
        pub.publish(make_autoware_trajectory(n_points=3))

        rclpy.spin_once(ros2_node, timeout_sec=0.1)

        assert listener._trajectory is not None

    def test_trajectory_has_correct_waypoints(self, ros2_node):
        """Converted trajectory should have the correct number of waypoints."""
        listener = TrajectoryListener(ros2_node)
        pub = ros2_node.create_publisher(
            AwTrajectory, "/planning/trajectory", 10
        )

        pub.publish(make_autoware_trajectory(n_points=5))
        rclpy.spin_once(ros2_node, timeout_sec=0.1)

        assert len(listener._trajectory.poses) == 5

    def test_latest_wins(self, ros2_node):
        """When multiple trajectories are published, the latest should be kept."""
        listener = TrajectoryListener(ros2_node)
        pub = ros2_node.create_publisher(
            AwTrajectory, "/planning/trajectory", 10
        )

        pub.publish(make_autoware_trajectory(n_points=3))
        rclpy.spin_once(ros2_node, timeout_sec=0.1)

        pub.publish(make_autoware_trajectory(n_points=7))
        rclpy.spin_once(ros2_node, timeout_sec=0.1)

        assert len(listener._trajectory.poses) == 7

    @pytest.mark.asyncio
    async def test_wait_timeout_raises(self, ros2_node):
        """Waiting for a trajectory should raise TimeoutError on timeout."""
        listener = TrajectoryListener(ros2_node)
        with pytest.raises(asyncio.TimeoutError):
            await listener.wait_for_trajectory(timeout_sec=0.1)
