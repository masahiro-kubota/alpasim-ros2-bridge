"""Publish simulation time to the /clock topic.

ROS2 nodes with use_sim_time:=true consume this clock.
"""

from __future__ import annotations

from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from alpasim_ros2_bridge.conversions import timestamp_us_to_sec_nanosec


class ClockPublisher:
    """Publish AlpaSim simulation time to /clock."""

    def __init__(self, node: Node) -> None:
        self._pub = node.create_publisher(Clock, "/clock", 10)

    def publish(self, timestamp_us: int) -> None:
        """Publish timestamp_us to /clock."""
        sec, nanosec = timestamp_us_to_sec_nanosec(timestamp_us)
        msg = Clock()
        msg.clock.sec = sec
        msg.clock.nanosec = nanosec
        self._pub.publish(msg)
