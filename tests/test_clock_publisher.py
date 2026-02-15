"""Tests for ClockPublisher (Layer 2: requires rclpy).

Verifies that /clock is published with the correct timestamp.
"""

rclpy = __import__("pytest").importorskip("rclpy")

from rosgraph_msgs.msg import Clock

from alpasim_ros2_bridge.clock_publisher import ClockPublisher


class TestClockPublisher:

    def test_publish_updates_clock(self, ros2_node):
        """Published timestamp_us should appear on /clock."""
        received = []
        ros2_node.create_subscription(
            Clock, "/clock", lambda msg: received.append(msg), 10
        )

        clock_pub = ClockPublisher(ros2_node)
        clock_pub.publish(1_500_000)  # 1.5 sec

        rclpy.spin_once(ros2_node, timeout_sec=0.1)

        assert len(received) == 1
        assert received[0].clock.sec == 1
        assert received[0].clock.nanosec == 500_000_000

    def test_monotonic_clock(self, ros2_node):
        """Consecutive publishes should produce monotonically increasing times."""
        received = []
        ros2_node.create_subscription(
            Clock, "/clock", lambda msg: received.append(msg), 10
        )

        clock_pub = ClockPublisher(ros2_node)
        for t in [100_000, 200_000, 300_000]:
            clock_pub.publish(t)
            rclpy.spin_once(ros2_node, timeout_sec=0.1)

        times = [m.clock.sec * 1_000_000_000 + m.clock.nanosec for m in received]
        assert times == sorted(times)
        assert len(set(times)) == 3  # no duplicates
