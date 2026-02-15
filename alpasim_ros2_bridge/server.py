"""AlpaSim ROS2 Bridge gRPC server.

Implements AlpaSim's EgodriverService, caching data from submit_*() calls
and publishing to ROS2 when drive() is called.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass, field

import grpc
import rclpy
from rclpy.node import Node

from alpasim_grpc.v0 import egodriver_pb2, egodriver_pb2_grpc, common_pb2
from alpasim_ros2_bridge.clock_publisher import ClockPublisher
from alpasim_ros2_bridge.sensor_publisher import SensorPublisher
from alpasim_ros2_bridge.tf_broadcaster import TFBroadcaster
from alpasim_ros2_bridge.traffic_publisher import TrafficPublisher
from alpasim_ros2_bridge.trajectory_listener import TrajectoryListener

logger = logging.getLogger(__name__)

VERSION = "0.1.0-egodriver"


@dataclass
class SessionState:
    """セッションごとの状態を管理"""
    camera_specs: dict  # logical_id -> CameraSpec
    latest_images: dict = field(default_factory=dict)  # logical_id -> (bytes, width, height, timestamp)
    latest_ego_trajectory: common_pb2.Trajectory | None = None
    latest_dynamic_state: common_pb2.DynamicState | None = None
    latest_route: list | None = None
    route_timestamp_us: int = 0


class EgodriverServicer(egodriver_pb2_grpc.EgodriverServiceServicer):
    """gRPC implementation of EgodriverService."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._sessions: dict[str, SessionState] = {}
        self._clock_pub = ClockPublisher(node)
        self._tf_broadcaster = TFBroadcaster(node)
        self._traffic_pub = TrafficPublisher(node)
        self._traj_listener = TrajectoryListener(node)
        self._sensor_pubs: dict[str, SensorPublisher] = {}

    async def start_session(self, request, context):
        """セッション開始: カメラ情報を保存"""
        session_uuid = request.session_uuid
        camera_specs_list = [
            cam.intrinsics
            for cam in request.rollout_spec.vehicle.available_cameras
        ]
        camera_specs = {spec.logical_id: spec for spec in camera_specs_list}

        self._sessions[session_uuid] = SessionState(camera_specs=camera_specs)

        camera_ids = list(camera_specs.keys())
        self._sensor_pubs[session_uuid] = SensorPublisher(self._node, camera_ids)

        logger.info(
            "Session started: %s, cameras: %s",
            session_uuid, camera_ids,
        )
        return common_pb2.SessionRequestStatus()

    async def close_session(self, request, context):
        """セッション終了"""
        session_uuid = request.session_uuid
        if session_uuid in self._sessions:
            del self._sessions[session_uuid]
            del self._sensor_pubs[session_uuid]
        logger.info("Session closed: %s", session_uuid)
        return common_pb2.Empty()

    async def submit_image_observation(self, request, context):
        """画像データをキャッシュ"""
        session_uuid = request.session_uuid
        if session_uuid not in self._sessions:
            return common_pb2.Empty()

        state = self._sessions[session_uuid]
        cam_img = request.camera_image

        # CameraSpec から width, height を取得
        spec = state.camera_specs.get(cam_img.logical_id)
        if spec is None:
            logger.warning("Unknown camera: %s", cam_img.logical_id)
            return common_pb2.Empty()

        state.latest_images[cam_img.logical_id] = (
            cam_img.image_bytes,
            spec.resolution_w,
            spec.resolution_h,
            cam_img.frame_start_us,
        )
        return common_pb2.Empty()

    async def submit_egomotion_observation(self, request, context):
        """ego 姿勢・速度をキャッシュ"""
        session_uuid = request.session_uuid
        if session_uuid not in self._sessions:
            return common_pb2.Empty()

        state = self._sessions[session_uuid]
        state.latest_ego_trajectory = request.trajectory
        state.latest_dynamic_state = request.dynamic_state
        return common_pb2.Empty()

    async def submit_route(self, request, context):
        """ルートをキャッシュ"""
        session_uuid = request.session_uuid
        if session_uuid not in self._sessions:
            return common_pb2.Empty()

        state = self._sessions[session_uuid]
        state.latest_route = list(request.route.waypoints)
        state.route_timestamp_us = request.route.timestamp_us
        return common_pb2.Empty()

    async def submit_recording_ground_truth(self, request, context):
        """Ground truth データは今回の実装では使用しない（無視）"""
        return common_pb2.Empty()

    async def drive(self, request, context):
        """キャッシュしたデータを ROS2 に publish し、trajectory を待つ"""
        session_uuid = request.session_uuid
        if session_uuid not in self._sessions:
            logger.warning("Unknown session: %s", session_uuid)
            return egodriver_pb2.DriveResponse()

        state = self._sessions[session_uuid]
        time_now_us = request.time_now_us

        # 1. Publish /clock
        self._clock_pub.publish(time_now_us)

        # 2. Publish /tf (ego pose)
        if state.latest_ego_trajectory and state.latest_ego_trajectory.poses:
            latest_pose = state.latest_ego_trajectory.poses[-1].pose
            self._tf_broadcaster.send_ego_transform(latest_pose, time_now_us)

        # 3. Publish camera images
        sensor_pub = self._sensor_pubs.get(session_uuid)
        if sensor_pub:
            for logical_id, (img_bytes, w, h, timestamp_us) in state.latest_images.items():
                sensor_pub.publish_image(logical_id, img_bytes, w, h, time_now_us)

        # 4. Publish ego velocity
        if state.latest_dynamic_state:
            self._traffic_pub.publish_velocity_report(
                state.latest_dynamic_state, time_now_us
            )

        # 5. Wait for planner trajectory
        try:
            trajectory = await self._traj_listener.wait_for_trajectory(timeout_sec=30.0)
            return egodriver_pb2.DriveResponse(trajectory=trajectory)
        except asyncio.TimeoutError:
            logger.warning("Trajectory timeout")
            return egodriver_pb2.DriveResponse()

    async def get_version(self, request, context):
        return common_pb2.VersionId(version_id=VERSION)

    async def shut_down(self, request, context):
        logger.info("Shutting down egodriver")
        return common_pb2.Empty()


async def serve(port: int = 50060) -> None:
    """Start the gRPC server and ROS2 node."""
    rclpy.init()
    node = rclpy.create_node("alpasim_egodriver")

    server = grpc.aio.server()
    servicer = EgodriverServicer(node)
    egodriver_pb2_grpc.add_EgodriverServiceServicer_to_server(servicer, server)
    server.add_insecure_port(f"[::]:{port}")
    await server.start()
    logger.info("Egodriver gRPC server listening on port %d", port)

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
