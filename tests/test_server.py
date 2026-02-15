"""gRPC + ROS2 統合テスト（EgodriverService）。

submit_*() でデータを送信し、drive() で ROS2 publish + trajectory 待機が
正しく動作することを検証する。
"""

import asyncio

import pytest
import pytest_asyncio

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_planning_msgs")

import grpc
from rosgraph_msgs.msg import Clock
from autoware_planning_msgs.msg import Trajectory as AwTrajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from builtin_interfaces.msg import Duration

from alpasim_grpc.v0 import egodriver_pb2, egodriver_pb2_grpc, common_pb2
from alpasim_ros2_bridge.server import EgodriverServicer
from tests.mock_data import (
    make_rollout_camera_image,
    make_rollout_ego_trajectory,
    make_route_request,
    make_drive_request,
    make_drive_session_request,
    make_trajectory,
)


def make_autoware_trajectory(n_points=5, start_sec=1, velocity_mps=10.0):
    """テスト用 autoware Trajectory を生成"""
    traj = AwTrajectory()
    traj.header.stamp.sec = start_sec
    traj.header.frame_id = "map"
    for i in range(n_points):
        pt = TrajectoryPoint()
        pt.time_from_start = Duration(sec=0, nanosec=i * 200_000_000)
        pt.pose.position.x = float(i) * 2.0
        pt.pose.orientation.w = 1.0
        pt.longitudinal_velocity_mps = velocity_mps
        traj.points.append(pt)
    return traj


@pytest_asyncio.fixture
async def egodriver_server_port(ros2_node):
    """テスト用 gRPC サーバーを起動"""
    server = grpc.aio.server()
    servicer = EgodriverServicer(ros2_node)
    egodriver_pb2_grpc.add_EgodriverServiceServicer_to_server(servicer, server)
    port = server.add_insecure_port("[::]:0")  # ランダムポート
    await server.start()
    yield port
    await server.stop(grace=0)


@pytest_asyncio.fixture
async def egodriver_stub(egodriver_server_port):
    """テスト用 gRPC クライアント"""
    channel = grpc.aio.insecure_channel(f"localhost:{egodriver_server_port}")
    stub = egodriver_pb2_grpc.EgodriverServiceStub(channel)
    yield stub
    await channel.close()


@pytest.mark.integration
class TestEgodriverSession:
    """セッション管理のテスト"""

    @pytest.mark.asyncio
    async def test_start_session_with_cameras(self, egodriver_stub):
        """start_session で camera リストを受け取る"""
        request = make_drive_session_request("test-session", ["front", "rear"])
        response = await egodriver_stub.start_session(request)
        assert response is not None  # SessionRequestStatus

    @pytest.mark.asyncio
    async def test_close_session(self, egodriver_stub):
        """close_session が成功する"""
        await egodriver_stub.start_session(
            make_drive_session_request("test-session", [])
        )
        response = await egodriver_stub.close_session(
            egodriver_pb2.DriveSessionCloseRequest(session_uuid="test-session")
        )
        assert response is not None  # Empty


@pytest.mark.integration
class TestEgodriverSubmit:
    """submit_* メソッドのテスト（データ蓄積の確認）"""

    @pytest.mark.asyncio
    async def test_submit_image_observation(self, egodriver_stub):
        """submit_image_observation が画像を受け取る"""
        await egodriver_stub.start_session(
            make_drive_session_request("test", ["front"])
        )
        request = make_rollout_camera_image("test", "front")
        response = await egodriver_stub.submit_image_observation(request)
        assert response is not None  # Empty

    @pytest.mark.asyncio
    async def test_submit_egomotion_observation(self, egodriver_stub):
        """submit_egomotion_observation が trajectory + dynamic_state を受け取る"""
        await egodriver_stub.start_session(
            make_drive_session_request("test", [])
        )
        request = make_rollout_ego_trajectory("test", n_points=5)
        response = await egodriver_stub.submit_egomotion_observation(request)
        assert response is not None  # Empty

    @pytest.mark.asyncio
    async def test_submit_route(self, egodriver_stub):
        """submit_route がルート waypoints を受け取る"""
        await egodriver_stub.start_session(
            make_drive_session_request("test", [])
        )
        waypoints = [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
        request = make_route_request("test", waypoints, timestamp_us=1_000_000)
        response = await egodriver_stub.submit_route(request)
        assert response is not None  # Empty

    @pytest.mark.asyncio
    async def test_submit_recording_ground_truth(self, egodriver_stub):
        """submit_recording_ground_truth を受け取る（実装では無視）"""
        await egodriver_stub.start_session(
            make_drive_session_request("test", [])
        )
        # Ground truth データは今回の実装では使用しないため、空のリクエストで十分
        request = egodriver_pb2.GroundTruthRequest(
            session_uuid="test",
            ground_truth=egodriver_pb2.GroundTruth(
                timestamp_us=1_000_000,
                trajectory=make_trajectory(n_points=3),
            )
        )
        response = await egodriver_stub.submit_recording_ground_truth(request)
        assert response is not None  # Empty


@pytest.mark.integration
class TestEgodriverDrive:
    """drive() の統合テスト - ROS2 publish と trajectory 待機"""

    @pytest.mark.asyncio
    async def test_drive_publishes_clock(self, egodriver_stub, ros2_node):
        """drive() が /clock を publish する"""
        received = []
        ros2_node.create_subscription(
            Clock, "/clock", lambda m: received.append(m), 10
        )

        await egodriver_stub.start_session(
            make_drive_session_request("test", [])
        )

        # Planner を準備（タイムアウトしないように）
        planner_pub = ros2_node.create_publisher(
            AwTrajectory, "/planning/trajectory", 10
        )

        async def mock_planner():
            await asyncio.sleep(0.05)
            planner_pub.publish(make_autoware_trajectory(n_points=3))
            rclpy.spin_once(ros2_node, timeout_sec=0.1)

        task = asyncio.create_task(mock_planner())

        await egodriver_stub.drive(
            make_drive_request("test", time_now_us=1_000_000, time_query_us=1_100_000)
        )
        await task

        rclpy.spin_once(ros2_node, timeout_sec=0.1)
        assert len(received) >= 1
        assert received[0].clock.sec == 1

    @pytest.mark.asyncio
    async def test_drive_returns_trajectory(self, egodriver_stub, ros2_node):
        """drive() が /planning/trajectory からの応答を返す"""
        planner_pub = ros2_node.create_publisher(
            AwTrajectory, "/planning/trajectory", 10
        )

        await egodriver_stub.start_session(
            make_drive_session_request("test", [])
        )
        await egodriver_stub.submit_egomotion_observation(
            make_rollout_ego_trajectory("test", n_points=3)
        )

        async def mock_planner():
            await asyncio.sleep(0.05)
            planner_pub.publish(make_autoware_trajectory(n_points=5))
            rclpy.spin_once(ros2_node, timeout_sec=0.1)

        task = asyncio.create_task(mock_planner())

        response = await egodriver_stub.drive(
            make_drive_request("test", time_now_us=1_000_000, time_query_us=1_100_000)
        )
        await task

        assert response.trajectory is not None
        assert len(response.trajectory.poses) == 5

    @pytest.mark.asyncio
    async def test_drive_full_flow(self, egodriver_stub, ros2_node):
        """統合フロー: start_session -> submit_* -> drive -> close_session"""
        planner_pub = ros2_node.create_publisher(
            AwTrajectory, "/planning/trajectory", 10
        )

        # Session 開始
        await egodriver_stub.start_session(
            make_drive_session_request("full-test", ["front"])
        )

        # データ送信
        await egodriver_stub.submit_image_observation(
            make_rollout_camera_image("full-test", "front")
        )
        await egodriver_stub.submit_egomotion_observation(
            make_rollout_ego_trajectory("full-test", n_points=5)
        )
        await egodriver_stub.submit_route(
            make_route_request("full-test", [(1.0, 0.0, 0.0)])
        )

        # Planner 準備
        async def mock_planner():
            await asyncio.sleep(0.05)
            planner_pub.publish(make_autoware_trajectory(n_points=10))
            rclpy.spin_once(ros2_node, timeout_sec=0.1)

        task = asyncio.create_task(mock_planner())

        # Drive 要求
        response = await egodriver_stub.drive(
            make_drive_request("full-test", time_now_us=1_000_000, time_query_us=1_100_000)
        )
        await task

        assert len(response.trajectory.poses) == 10

        # Session 終了
        await egodriver_stub.close_session(
            egodriver_pb2.DriveSessionCloseRequest(session_uuid="full-test")
        )


@pytest.mark.integration
class TestEgodriverUtility:
    """その他の RPC のテスト"""

    @pytest.mark.asyncio
    async def test_get_version(self, egodriver_stub):
        """get_version が version を返す"""
        response = await egodriver_stub.get_version(common_pb2.Empty())
        assert response.version_id != ""

    @pytest.mark.asyncio
    async def test_shut_down(self, egodriver_stub):
        """shut_down が成功する"""
        response = await egodriver_stub.shut_down(common_pb2.Empty())
        assert response is not None  # Empty
