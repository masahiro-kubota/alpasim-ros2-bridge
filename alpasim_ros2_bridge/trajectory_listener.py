"""Wait for planner trajectory from ROS2.

Subscribes to /planning/trajectory (autoware_planning_msgs/Trajectory) and
uses asyncio.Event for barrier synchronization. Called from the gRPC step()
handler to block until the planner responds.
"""

from __future__ import annotations

import asyncio
from typing import Optional

from rclpy.node import Node

from alpasim_grpc.v0 import common_pb2
from alpasim_ros2_bridge.conversions import (
    autoware_trajectory_to_alpasim,
    ros_quat_to_alpasim,
)

from autoware_planning_msgs.msg import Trajectory as AwTrajectory


class TrajectoryListener:
    """Receive and wait for planner trajectory from ROS2."""

    def __init__(self, node: Node) -> None:
        self._trajectory: Optional[common_pb2.Trajectory] = None
        self._event = asyncio.Event()
        self._sub = node.create_subscription(
            AwTrajectory, "/planning/trajectory", self._on_trajectory, 10
        )

    def _on_trajectory(self, msg: AwTrajectory) -> None:
        """Receive an autoware Trajectory and convert to AlpaSim format."""
        points = []
        for pt in msg.points:
            points.append({
                "pose": {
                    "position": {
                        "x": pt.pose.position.x,
                        "y": pt.pose.position.y,
                        "z": pt.pose.position.z,
                    },
                    "orientation": {
                        "x": pt.pose.orientation.x,
                        "y": pt.pose.orientation.y,
                        "z": pt.pose.orientation.z,
                        "w": pt.pose.orientation.w,
                    },
                },
                "time_from_start": {
                    "sec": pt.time_from_start.sec,
                    "nanosec": pt.time_from_start.nanosec,
                },
                "longitudinal_velocity_mps": pt.longitudinal_velocity_mps,
                "heading_rate_rps": pt.heading_rate_rps,
            })

        self._trajectory = autoware_trajectory_to_alpasim(
            points=points,
            header_stamp_sec=msg.header.stamp.sec,
            header_stamp_nanosec=msg.header.stamp.nanosec,
        )
        self._event.set()

    async def wait_for_trajectory(self, timeout_sec: float = 30.0) -> common_pb2.Trajectory:
        """Wait for a planner trajectory. Raises asyncio.TimeoutError on timeout."""
        self._event.clear()
        await asyncio.wait_for(self._event.wait(), timeout=timeout_sec)
        return self._trajectory
