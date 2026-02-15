"""AlpaSim ROS2 Bridge gRPC server.

Runs a gRPC server and ROS2 node in the same process, publishing AlpaSim
data to ROS2 topics and returning the planner trajectory as a gRPC response.
"""

from __future__ import annotations

import asyncio
import logging
from concurrent import futures

import grpc
import rclpy
from rclpy.node import Node

from alpasim_grpc.v0 import bridge_pb2, bridge_pb2_grpc, common_pb2
from alpasim_ros2_bridge.clock_publisher import ClockPublisher
from alpasim_ros2_bridge.sensor_publisher import SensorPublisher
from alpasim_ros2_bridge.tf_broadcaster import TFBroadcaster
from alpasim_ros2_bridge.traffic_publisher import TrafficPublisher
from alpasim_ros2_bridge.trajectory_listener import TrajectoryListener

logger = logging.getLogger(__name__)

VERSION = "0.1.0"


class BridgeServicer(bridge_pb2_grpc.ROS2BridgeServiceServicer):
    """gRPC implementation of ROS2BridgeService."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._clock_pub = ClockPublisher(node)
        self._tf_broadcaster = TFBroadcaster(node)
        self._sensor_pub: SensorPublisher | None = None
        self._traffic_pub = TrafficPublisher(node)
        self._traj_listener = TrajectoryListener(node)
        self._camera_names: list[str] = []

    async def start_session(self, request, context):
        self._camera_names = list(request.camera_names)
        self._sensor_pub = SensorPublisher(self._node, self._camera_names)
        logger.info(
            "Session started: %s, cameras: %s",
            request.session_uuid, self._camera_names,
        )
        return common_pb2.SessionRequestStatus()

    async def close_session(self, request, context):
        logger.info("Session closed: %s", request.session_uuid)
        return common_pb2.Empty()

    async def step(self, request, context):
        # 1. /clock
        self._clock_pub.publish(request.timestamp_us)

        # 2. /tf (ego)
        self._tf_broadcaster.send_ego_transform(request.ego_pose, request.timestamp_us)

        # 3. /tf (traffic actors)
        for obj in request.traffic_objects:
            self._tf_broadcaster.send_actor_transform(
                obj.object_id, obj.pose, request.timestamp_us
            )

        # 4. Camera images
        if self._sensor_pub is not None:
            for img in request.camera_images:
                self._sensor_pub.publish_image(
                    img.camera_id, img.image_bytes,
                    img.width, img.height, request.timestamp_us,
                )

        # 5. Traffic objects (TrackedObjects)
        if request.traffic_objects:
            traffic_dicts = []
            for obj in request.traffic_objects:
                traffic_dicts.append({
                    "object_id": obj.object_id,
                    "pose": obj.pose,
                    "aabb": obj.aabb,
                    "is_static": obj.is_static,
                })
            self._traffic_pub.publish_objects(traffic_dicts, request.timestamp_us)

        # 6. Ego velocity (VelocityReport)
        if request.HasField("ego_dynamic_state"):
            self._traffic_pub.publish_velocity_report(
                request.ego_dynamic_state, request.timestamp_us
            )

        # 7. Wait for planner response (skip when force_gt)
        if not request.force_gt:
            try:
                trajectory = await self._traj_listener.wait_for_trajectory()
                return bridge_pb2.BridgeStepResponse(
                    trajectory=trajectory, has_trajectory=True
                )
            except asyncio.TimeoutError:
                logger.warning("Trajectory timeout, returning empty response")
                return bridge_pb2.BridgeStepResponse(has_trajectory=False)

        return bridge_pb2.BridgeStepResponse(has_trajectory=False)

    async def get_version(self, request, context):
        return common_pb2.VersionId(version_id=VERSION)

    async def shut_down(self, request, context):
        logger.info("Shutting down bridge")
        return common_pb2.Empty()


async def serve(port: int = 50060) -> None:
    """Start the gRPC server and ROS2 node."""
    rclpy.init()
    node = rclpy.create_node("alpasim_bridge")

    server = grpc.aio.server()
    servicer = BridgeServicer(node)
    bridge_pb2_grpc.add_ROS2BridgeServiceServicer_to_server(servicer, server)
    server.add_insecure_port(f"[::]:{port}")
    await server.start()
    logger.info("Bridge gRPC server listening on port %d", port)

    # Run ROS2 spin in the background
    spin_task = asyncio.create_task(_spin_ros2(node))

    try:
        await server.wait_for_termination()
    finally:
        spin_task.cancel()
        node.destroy_node()
        rclpy.shutdown()


async def _spin_ros2(node: Node) -> None:
    """Process ROS2 callbacks inside the asyncio event loop."""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)
