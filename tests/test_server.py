"""gRPC + ROS2 integration tests (Layer 3).

Verifies that the bridge server's step() RPC publishes all topics and
returns the planner trajectory as a gRPC response.
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

from alpasim_grpc.v0 import bridge_pb2, bridge_pb2_grpc
from alpasim_ros2_bridge.server import BridgeServicer
from tests.mock_data import make_pose


def make_autoware_trajectory(n_points=5, start_sec=1, velocity_mps=10.0):
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
async def bridge_server_port(ros2_node):
    """Start a test gRPC server and return its port number."""
    server = grpc.aio.server()
    servicer = BridgeServicer(ros2_node)
    bridge_pb2_grpc.add_ROS2BridgeServiceServicer_to_server(servicer, server)
    port = server.add_insecure_port("[::]:0")
    await server.start()
    yield port
    await server.stop(grace=0)


@pytest_asyncio.fixture
async def bridge_stub(bridge_server_port):
    """Create a test gRPC client."""
    channel = grpc.aio.insecure_channel(f"localhost:{bridge_server_port}")
    stub = bridge_pb2_grpc.ROS2BridgeServiceStub(channel)
    yield stub
    await channel.close()


@pytest.mark.integration
class TestBridgeStep:

    @pytest.mark.asyncio
    async def test_step_force_gt_returns_immediately(self, bridge_stub):
        """force_gt=True should return immediately without waiting for trajectory."""
        response = await bridge_stub.step(bridge_pb2.BridgeStepRequest(
            session_uuid="test",
            timestamp_us=1_000_000,
            ego_pose=make_pose(x=0.0),
            force_gt=True,
        ))
        assert response.has_trajectory is False

    @pytest.mark.asyncio
    async def test_step_publishes_clock(self, bridge_stub, ros2_node):
        """step() should publish to /clock."""
        received = []
        ros2_node.create_subscription(
            Clock, "/clock", lambda m: received.append(m), 10
        )

        await bridge_stub.step(bridge_pb2.BridgeStepRequest(
            session_uuid="test",
            timestamp_us=1_000_000,
            ego_pose=make_pose(x=10.0),
            force_gt=True,
        ))

        rclpy.spin_once(ros2_node, timeout_sec=0.1)
        assert len(received) >= 1
        assert received[0].clock.sec == 1

    @pytest.mark.asyncio
    async def test_multi_step_flow(self, bridge_stub, ros2_node):
        """3 consecutive steps should publish /clock 3 times."""
        received_clocks = []
        ros2_node.create_subscription(
            Clock, "/clock", lambda m: received_clocks.append(m), 10
        )

        for i in range(3):
            await bridge_stub.step(bridge_pb2.BridgeStepRequest(
                session_uuid="test",
                timestamp_us=(i + 1) * 100_000,
                ego_pose=make_pose(x=float(i)),
                force_gt=True,
            ))
            rclpy.spin_once(ros2_node, timeout_sec=0.05)

        assert len(received_clocks) == 3

    @pytest.mark.asyncio
    async def test_step_returns_trajectory(self, bridge_stub, ros2_node):
        """step(force_gt=False) should return the planner trajectory.

        Full flow: gRPC step() -> ROS2 publish -> planner response -> gRPC response.
        """
        planner_pub = ros2_node.create_publisher(
            AwTrajectory, "/planning/trajectory", 10
        )

        async def mock_planner():
            """Simulate a planner response while the bridge waits for trajectory."""
            await asyncio.sleep(0.05)
            planner_pub.publish(make_autoware_trajectory(n_points=5))
            rclpy.spin_once(ros2_node, timeout_sec=0.1)

        task = asyncio.create_task(mock_planner())

        response = await bridge_stub.step(bridge_pb2.BridgeStepRequest(
            session_uuid="test",
            timestamp_us=1_000_000,
            ego_pose=make_pose(x=10.0),
            force_gt=False,
        ))
        await task

        assert response.has_trajectory is True
        assert len(response.trajectory.poses) == 5


@pytest.mark.integration
class TestBridgeSession:

    @pytest.mark.asyncio
    async def test_start_and_close_session(self, bridge_stub):
        """Session start and close should succeed."""
        await bridge_stub.start_session(bridge_pb2.BridgeSessionRequest(
            session_uuid="test-session",
            camera_names=["front"],
        ))
        await bridge_stub.close_session(bridge_pb2.BridgeSessionCloseRequest(
            session_uuid="test-session",
        ))

    @pytest.mark.asyncio
    async def test_get_version(self, bridge_stub):
        """get_version should return a non-empty version string."""
        from alpasim_grpc.v0 import common_pb2
        response = await bridge_stub.get_version(common_pb2.Empty())
        assert response.version_id != ""
