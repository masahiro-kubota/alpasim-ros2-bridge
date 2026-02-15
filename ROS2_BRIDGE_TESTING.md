# AlpaSim ROS2 Bridge テスト戦略

## 基本方針

- Bridge は **EgodriverService** (egodriver.proto) を実装する gRPC サービス
- 別リポジトリ・別コンテナで動作し、AlpaSim 本体は不要
- AlpaSim の protobuf 定義（`alpasim_grpc`）のみ軽量依存として使用
- テストは Docker コンテナ内で `pytest` を直接実行する（colcon test は使わない）
- **AlpaSim 側のコード変更は不要**（設定で driver アドレスを Bridge に向けるだけ）

---

## テストレイヤー

```
┌─────────────────────────────────────────────────────────┐
│  Layer 3: gRPC Integration Tests                        │
│  「EgodriverService の submit_*() + drive() 呼び出し   │
│   → ROS2 publish → trajectory 受信 → gRPC レスポンス   │
│   の全フローが正しく流れるか」                          │
│  gRPC client + ROS2 ノード起動 + mock プランナー        │
├─────────────────────────────────────────────────────────┤
│  Layer 2: Node Tests                                    │
│  「各 ROS2 publisher/subscriber が単体で動くか」        │
│  rclpy 使用、AlpaSim 不要                               │
├─────────────────────────────────────────────────────────┤
│  Layer 1: Unit Tests                                    │
│  「型変換・データ変換が正しいか」                        │
│  Pure Python、ROS2 不要、pytest のみ                    │
└─────────────────────────────────────────────────────────┘
```

---

## テストディレクトリ構成

```
alpasim-ros2-bridge/
├── alpasim_ros2_bridge/
│   ├── __init__.py
│   ├── conversions.py
│   ├── clock_publisher.py
│   ├── sensor_publisher.py
│   ├── tf_broadcaster.py
│   ├── traffic_publisher.py
│   ├── trajectory_listener.py
│   └── server.py              # EgodriverServicer 実装
└── tests/
    ├── conftest.py             # 共通フィクスチャ
    ├── mock_data.py            # AlpaSim データの mock 生成
    │
    │  # Layer 1: Unit Tests (ROS2 不要)
    ├── test_conversions.py
    │
    │  # Layer 2: Node Tests (rclpy 使用)
    ├── test_clock_publisher.py
    ├── test_sensor_publisher.py
    ├── test_tf_broadcaster.py
    ├── test_traffic_publisher.py
    ├── test_trajectory_listener.py
    │
    │  # Layer 3: gRPC Integration Tests
    └── test_server.py          # EgodriverService 統合テスト
```

---

## テスト実行方法

### Docker コンテナ内（全レイヤー）

```bash
# ビルド
docker build -t alpasim-bridge:latest .

# 全テスト実行（デフォルト CMD）
docker run --rm alpasim-bridge:latest

# レイヤー別
docker run --rm alpasim-bridge:latest python3 -m pytest tests/test_conversions.py -v
docker run --rm alpasim-bridge:latest python3 -m pytest tests/test_clock_publisher.py -v
docker run --rm alpasim-bridge:latest python3 -m pytest tests/test_server.py -v
```

### ホストマシン（Layer 1 のみ）

Layer 1 は ROS2 不要なのでホストでも実行可能。

```bash
cd alpasim-ros2-bridge
python3 -m pytest tests/test_conversions.py -v
```

Layer 2/3 はコンテナ内でのみ実行（rclpy + autoware_msgs が必要）。

---

## Layer 1: Unit Tests（Pure Python、ROS2 不要）

### test_conversions.py

ROS2 や rclpy を一切 import せずにテスト可能。
`conversions.py` の関数が正しく動くことを確認する。

```python
"""型変換のユニットテスト。ROS2 不要。"""

import numpy as np
import pytest
from alpasim_ros2_bridge.conversions import (
    timestamp_us_to_sec_nanosec,
    sec_nanosec_to_timestamp_us,
    alpasim_quat_to_ros,
    ros_quat_to_alpasim,
    alpasim_pose_to_ros_pose,
    autoware_trajectory_to_alpasim,
)


class TestTimestampConversion:
    """AlpaSim timestamp_us ↔ ROS2 Time の変換"""

    def test_zero(self):
        sec, nanosec = timestamp_us_to_sec_nanosec(0)
        assert sec == 0
        assert nanosec == 0

    def test_round_trip(self):
        original_us = 1_700_000_123_456
        sec, nanosec = timestamp_us_to_sec_nanosec(original_us)
        restored_us = sec_nanosec_to_timestamp_us(sec, nanosec)
        assert restored_us == original_us

    def test_microsecond_precision(self):
        sec, nanosec = timestamp_us_to_sec_nanosec(1_500_000)
        assert sec == 1
        assert nanosec == 500_000_000

    @pytest.mark.parametrize("us", [0, 1, 999_999, 1_000_000, 123_456_789_012])
    def test_non_negative(self, us):
        sec, nanosec = timestamp_us_to_sec_nanosec(us)
        assert sec >= 0
        assert 0 <= nanosec < 1_000_000_000


class TestQuaternionConversion:
    """AlpaSim Quat(w,x,y,z) ↔ ROS2 Quaternion(x,y,z,w) の並び順変換"""

    def test_identity(self):
        ros_q = alpasim_quat_to_ros(w=1.0, x=0.0, y=0.0, z=0.0)
        assert ros_q == (0.0, 0.0, 0.0, 1.0)

    def test_round_trip(self):
        w, x, y, z = 0.707, 0.0, 0.707, 0.0
        ros_q = alpasim_quat_to_ros(w, x, y, z)
        restored = ros_quat_to_alpasim(*ros_q)
        assert restored == pytest.approx((w, x, y, z))


class TestTrajectoryConversion:
    """autoware_planning_msgs/Trajectory → AlpaSim common_pb2.Trajectory の変換"""

    def test_empty_trajectory(self):
        traj = autoware_trajectory_to_alpasim(points=[], header_stamp_sec=0, header_stamp_nanosec=0)
        assert len(traj.poses) == 0

    def test_multi_point_ordering(self):
        """waypoint は時系列順であること"""
        points = [
            {
                "pose": {"position": {"x": float(i), "y": 0.0, "z": 0.0},
                         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
                "time_from_start": {"sec": i, "nanosec": 0},
                "longitudinal_velocity_mps": 10.0,
                "heading_rate_rps": 0.0,
            }
            for i in range(5)
        ]
        traj = autoware_trajectory_to_alpasim(points=points, header_stamp_sec=0, header_stamp_nanosec=0)
        timestamps = [p.timestamp_us for p in traj.poses]
        assert timestamps == sorted(timestamps)
        assert len(set(timestamps)) == 5
```

---

## Layer 2: Node Tests（rclpy 使用、AlpaSim 不要）

rclpy を使って ROS2 ノードの機能を個別にテストする。
AlpaSim の実サービスは不要。mock データを注入する。

### conftest.py（共通フィクスチャ）

```python
"""ROS2 テスト用共通フィクスチャ"""

import uuid
import pytest

try:
    import rclpy
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


@pytest.fixture(scope="session")
def ros2_context():
    """セッション全体で1回だけ rclpy を初期化"""
    if not HAS_RCLPY:
        pytest.skip("rclpy not available")
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ros2_node(ros2_context):
    """テストごとに使い捨て ROS2 ノードを作成（ユニーク名）"""
    node = rclpy.create_node(f"test_node_{uuid.uuid4().hex[:8]}")
    yield node
    node.destroy_node()
```

### mock_data.py（AlpaSim データの mock）

```python
"""AlpaSim サービス出力の mock データ生成。
alpasim_grpc の protobuf 型を直接構築する。"""

import numpy as np
from alpasim_grpc.v0 import common_pb2, egodriver_pb2


def make_pose(x=0.0, y=0.0, z=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    """Create AlpaSim Pose"""
    return common_pb2.Pose(
        vec=common_pb2.Vec3(x=x, y=y, z=z),
        quat=common_pb2.Quat(w=qw, x=qx, y=qy, z=qz),
    )


def make_pose_at_time(timestamp_us, x=0.0, y=0.0, z=0.0):
    """Create AlpaSim PoseAtTime"""
    return common_pb2.PoseAtTime(
        pose=make_pose(x=x, y=y, z=z),
        timestamp_us=timestamp_us,
    )


def make_dynamic_state(vx=10.0, vy=0.0, vz=0.0):
    """Create AlpaSim DynamicState"""
    return common_pb2.DynamicState(
        linear_velocity=common_pb2.Vec3(x=vx, y=vy, z=vz),
        angular_velocity=common_pb2.Vec3(x=0.0, y=0.0, z=0.0),
    )


def make_trajectory(n_points=5, dt_us=100_000, start_us=0):
    """Create AlpaSim Trajectory"""
    poses = []
    for i in range(n_points):
        t = start_us + i * dt_us
        poses.append(make_pose_at_time(t, x=float(i) * 1.0))
    return common_pb2.Trajectory(poses=poses)


def make_rgb_image(width=640, height=480):
    """Create random RGB image bytes"""
    return np.random.randint(0, 255, (height, width, 3), dtype=np.uint8).tobytes()


# --- Egodriver-specific helpers ---

def make_rollout_camera_image(session_uuid, logical_id, width=640, height=480,
                               frame_start_us=0, frame_end_us=100_000):
    """Create RolloutCameraImage for submit_image_observation"""
    return egodriver_pb2.RolloutCameraImage(
        session_uuid=session_uuid,
        camera_image=egodriver_pb2.RolloutCameraImage.CameraImage(
            logical_id=logical_id,
            frame_start_us=frame_start_us,
            frame_end_us=frame_end_us,
            image_bytes=make_rgb_image(width, height),
        )
    )


def make_rollout_ego_trajectory(session_uuid, n_points=5, dt_us=100_000, start_us=0):
    """Create RolloutEgoTrajectory for submit_egomotion_observation"""
    return egodriver_pb2.RolloutEgoTrajectory(
        session_uuid=session_uuid,
        trajectory=make_trajectory(n_points, dt_us, start_us),
        dynamic_state=make_dynamic_state(vx=10.0),
    )


def make_route_request(session_uuid, waypoints_xyz, timestamp_us=0):
    """Create RouteRequest for submit_route"""
    route = egodriver_pb2.Route(
        timestamp_us=timestamp_us,
        waypoints=[common_pb2.Vec3(x=x, y=y, z=z) for x, y, z in waypoints_xyz]
    )
    return egodriver_pb2.RouteRequest(
        session_uuid=session_uuid,
        route=route,
    )


def make_drive_request(session_uuid, time_now_us, time_query_us):
    """Create DriveRequest"""
    return egodriver_pb2.DriveRequest(
        session_uuid=session_uuid,
        time_now_us=time_now_us,
        time_query_us=time_query_us,
    )


def make_drive_session_request(session_uuid, camera_logical_ids, random_seed=0):
    """Create DriveSessionRequest with camera specs"""
    try:
        from alpasim_grpc.v0.sensorsim_pb2 import AvailableCamerasReturn, CameraSpec
    except ImportError:
        # Fallback for testing without sensorsim
        cameras = []
    else:
        cameras = [
            AvailableCamerasReturn.AvailableCamera(
                logical_id=camera_id,
                intrinsics=CameraSpec(
                    logical_id=camera_id,
                    resolution_w=640,
                    resolution_h=480,
                )
            )
            for camera_id in camera_logical_ids
        ]

    rollout_spec = egodriver_pb2.DriveSessionRequest.RolloutSpec(
        vehicle=egodriver_pb2.DriveSessionRequest.RolloutSpec.VehicleDefinition(
            available_cameras=cameras
        )
    )

    return egodriver_pb2.DriveSessionRequest(
        session_uuid=session_uuid,
        random_seed=random_seed,
        rollout_spec=rollout_spec,
    )
```

### test_clock_publisher.py, test_trajectory_listener.py など

（Layer 2 テストは既存と同じ構造。省略）

---

## Layer 3: gRPC Integration Tests

### test_server.py

EgodriverService の submit_*() + drive() の統合フローをテストする。

```python
"""gRPC + ROS2 統合テスト（EgodriverService）。
submit_*() でデータを送信し、drive() で ROS2 publish + trajectory 待機が
正しく動作することを検証する。"""

import asyncio
import grpc
import pytest
import pytest_asyncio

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_planning_msgs")

from rosgraph_msgs.msg import Clock
from autoware_planning_msgs.msg import Trajectory as AwTrajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration

from alpasim_grpc.v0 import egodriver_pb2, egodriver_pb2_grpc, common_pb2
from alpasim_ros2_bridge.server import EgodriverServicer
from tests.mock_data import (
    make_rollout_camera_image,
    make_rollout_ego_trajectory,
    make_route_request,
    make_drive_request,
    make_drive_session_request,
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
```

---

## 各開発ステップとテストの対応

| 実装ステップ | 追加するテスト | 確認ポイント |
|---|---|---|
| Step 1: conversions | `test_conversions.py` | 時刻・座標・trajectory 変換 |
| Step 2: /clock | `test_clock_publisher.py` | タイムスタンプ精度、publish 動作 |
| Step 3: TF broadcast | `test_tf_broadcaster.py` | 座標系・四元数の並び順 |
| Step 4: センサー | `test_sensor_publisher.py` | 画像フォーマット、frame_id |
| Step 5: trajectory_listener | `test_trajectory_listener.py` | subscribe + asyncio 待機、タイムアウト |
| Step 6: Velocity | `test_traffic_publisher.py` | VelocityReport publish、ego 速度情報 |
| Step 7: server.py | `test_server.py` | EgodriverService 統合フロー |

---

## テストで AlpaSim 本体が不要な理由

```
alpasim_ros2_bridge が依存するもの:
  ├── alpasim_grpc                  ← protobuf 定義のみ（軽量）
  ├── grpcio                        ← gRPC サーバー/クライアント
  ├── rclpy                         ← ROS2 Python ランタイム
  ├── sensor_msgs, geometry_msgs    ← ROS2 common_interfaces
  ├── autoware_planning_msgs        ← Trajectory, TrajectoryPoint
  └── autoware_vehicle_msgs         ← VelocityReport

alpasim_ros2_bridge が依存しないもの:
  ├── alpasim_runtime     ← シミュレーションループ（重い）
  ├── alpasim_driver      ← PyTorch モデル（非常に重い）
  ├── sensorsim           ← NRE レンダラー（GPU 必要）
  ├── alpasim_controller  ← MPC ソルバー
  └── alpasim_physics     ← Warp-lang（GPU 必要）
```

Bridge は AlpaSim のループから **gRPC 経由** でデータを受け取る。
テストでは gRPC クライアントから mock データを送信するため、
AlpaSim 本体のビルド・起動は一切不要。

Bridge コンテナ (Ubuntu 24.04 + ROS2 Jazzy + Python 3.12) と
AlpaSim コンテナ (Python 3.12) は完全に分離される。

---

## AlpaSim との統合

### アーキテクチャ

```
┌─── AlpaSim Container ─────────────────┐
│  loop.py → driver (EgodriverService)  │
│             ↓ gRPC                    │
└───────────────────────────────────────┘
              ↓
┌─── Bridge Container (driver 名で起動) ┐
│  EgodriverServicer                    │
│    ├─ submit_image()                  │
│    ├─ submit_egomotion()              │
│    ├─ submit_route()                  │
│    ├─ drive() → ROS2 publish          │
│    └─   → /planning/trajectory 待機   │
└───────────────────────────────────────┘
              ↓ ROS2 Topics
┌─── Planner Container ─────────────────┐
│  /planning/trajectory publisher       │
└───────────────────────────────────────┘
```

### Docker Compose による統合テスト

```yaml
services:
  driver:  # Bridge を driver 名で起動
    build:
      context: ./alpasim-ros2-bridge
    command: ["python3", "-m", "alpasim_ros2_bridge.server", "--port", "50060"]
    network_mode: host

  mock-planner:
    build:
      context: ./alpasim-ros2-bridge
    command: ["python3", "-m", "tests.mock_planner"]
    network_mode: host
    depends_on:
      - driver

  alpasim:
    build:
      context: ./alpasim
    command: ["python3", "-m", "alpasim_runtime.simulate", "--config", "test.yaml"]
    network_mode: host
    environment:
      DRIVER_ADDRESS: "driver:50060"
    depends_on:
      - driver
      - mock-planner
```

### 重要事項

- **AlpaSim 側のコード変更は不要**（設定ファイルで driver アドレスを変更するだけ）
- Bridge コンテナを `driver` という名前で起動
- AlpaSim の設定で `driver.addresses: ["driver:50060"]` を指定
- 既存の driver サービスの代わりに Bridge が動作

---

## Mock Planner（テスト用）

`alpasim-ros2-bridge/tests/mock_planner.py`

```python
"""テスト用ダミープランナー。
/clock を受信したら固定の /planning/trajectory を publish する。"""

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint


class MockPlanner(Node):
    def __init__(self):
        super().__init__("mock_planner")
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self._pub = self.create_publisher(
            Trajectory, "/planning/trajectory", 10
        )

    def _on_clock(self, msg):
        traj = Trajectory()
        traj.header.stamp = msg.clock
        traj.header.frame_id = "map"
        for i in range(10):
            pt = TrajectoryPoint()
            pt.pose.position.x = float(i)
            pt.pose.orientation.w = 1.0
            pt.longitudinal_velocity_mps = 10.0
            traj.points.append(pt)
        self._pub.publish(traj)


def main():
    rclpy.init()
    rclpy.spin(MockPlanner())


if __name__ == "__main__":
    main()
```

---

## まとめ

- **Layer 1-3** のテストで Bridge の動作を完全に検証
- AlpaSim 本体は不要（gRPC + mock データで完結）
- **EgodriverService** として実装することで、AlpaSim 側の変更は設定のみ
- Docker Compose で統合テストも可能
