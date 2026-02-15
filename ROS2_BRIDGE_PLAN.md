# AlpaSim ROS2 Bridge 設計・実装計画

## 目的

AlpaSim のシミュレーション出力を ROS2 エコシステムに接続し、
ROS2 ベースのプランナー（運転ポリシー）で AlpaSim のクローズドループシミュレーションを駆動できるようにする。

リアルタイム実行は目的としない。`/clock` によるシミュレーション時刻管理を前提とする。

---

## アーキテクチャ概要

**重要**: AlpaSim のコードは一切変更しない。

Bridge は AlpaSim の標準ドライバーインターフェース **EgodriverService** を実装する。
Bridge コンテナを **`driver`** という名前で起動することで、AlpaSim の既存の driver 呼び出しが自動的に Bridge に到達する。

Bridge は **別プロセス・別コンテナ** で動作する。
AlpaSim 本体と完全に分離されたリポジトリ（`alpasim-ros2-bridge`）で管理する。

```
┌─── AlpaSim Container (既存, Python 3.12) ─────────────────┐
│                                                             │
│  loop.py ──gRPC──→ sensorsim   (画像レンダリング)           │
│           ──gRPC──→ controller (trajectory → 車両制御)      │
│           ──gRPC──→ physics    (地面交差判定)                │
│           ──gRPC──→ traffic    (周辺車両シミュレーション)    │
│           ──gRPC──→ driver     (★ Bridge が実装)            │
│                                                             │
│  ※ AlpaSim のコード・設定は一切変更なし                      │
└─────────────────────────────────────────────────────────────┘
                        │ gRPC (egodriver.proto)
                        ▼
┌─── Bridge Container (新規, Ubuntu 24.04, ROS2 Jazzy) ─────┐
│  ※ コンテナ名: "driver" で起動                               │
│                                                             │
│  EgodriverService 実装 (gRPC server + ROS2 node)            │
│    ├─ gRPC: AlpaSim から submit_*() / drive() を受信        │
│    │   - submit_image_observation()                         │
│    │   - submit_egomotion_observation()                     │
│    │   - submit_route()                                     │
│    │   - drive() → ROS2 publish & wait for trajectory       │
│    │                                                         │
│    ├─ ROS2 Publish (drive() 内で実行):                      │
│    │    /clock              (rosgraph_msgs/Clock)            │
│    │    /camera/{name}/image_raw  (sensor_msgs/Image)        │
│    │    /camera/{name}/camera_info (sensor_msgs/CameraInfo)  │
│    │    /tf                 (tf2_msgs/TFMessage)             │
│    │    /vehicle/status/velocity (VelocityReport)            │
│    │                                                         │
│    └─ ROS2 Subscribe:                                        │
│         /planning/trajectory (autoware Trajectory)           │
│         ← プランナーの応答を待ってから gRPC レスポンスを返す  │
│                                                             │
│  Python 3.12 + ROS2 Jazzy + autoware_msgs                   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                        ▲
                        │ ROS2 Topics
                        ▼
┌─── Planner Container (ユーザー実装) ───────────────────────┐
│                                                             │
│  ROS2 Planner Node                                          │
│  Subscribe: /camera/*, /tf, /vehicle/status/velocity, ...   │
│  Publish:   /planning/trajectory                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### デプロイ方法

AlpaSim と Bridge を起動する際、Bridge コンテナに **`driver`** という名前を付けるだけ：

```bash
# Bridge を driver という名前で起動
docker run --rm --net=host --name driver alpasim-bridge:latest \
  python3 -m alpasim_ros2_bridge.server

# AlpaSim を起動（driver:50060 に自動接続）
docker run --rm --net=host alpasim:latest \
  python3 -m alpasim_runtime.main
```

AlpaSim のコードや設定ファイルは一切変更不要。

---

## gRPC API 定義 (egodriver.proto)

Bridge は AlpaSim の標準インターフェース **EgodriverService** を実装する。
`egodriver.proto` は AlpaSim から Bridge リポジトリにコピーし、Docker ビルド時にコンパイルする。

### サービス定義

```protobuf
syntax = "proto3";

package egodriver;

import "alpasim_grpc/v0/common.proto";
import "alpasim_grpc/v0/sensorsim.proto";

service EgodriverService {
    rpc start_session (DriveSessionRequest) returns (common.SessionRequestStatus);
    rpc close_session (DriveSessionCloseRequest) returns (common.Empty);
    rpc submit_image_observation (RolloutCameraImage) returns (common.Empty);
    rpc submit_egomotion_observation (RolloutEgoTrajectory) returns (common.Empty);
    rpc submit_route (RouteRequest) returns (common.Empty);
    rpc submit_recording_ground_truth (GroundTruthRequest) returns (common.Empty);
    rpc drive (DriveRequest) returns (DriveResponse);
    rpc get_version (common.Empty) returns (common.VersionId);
    rpc shut_down (common.Empty) returns (common.Empty);
}
```

### 主要メッセージ

#### DriveSessionRequest

```protobuf
message DriveSessionRequest {
    message RolloutSpec {
        message VehicleDefinition {
            repeated nre.grpc.protos.sensorsim.AvailableCamerasReturn.AvailableCamera available_cameras = 3;
        }
        VehicleDefinition vehicle = 1;
    }

    string session_uuid = 1;
    fixed64 random_seed = 2;
    optional DebugInfo debug_info = 4;
    RolloutSpec rollout_spec = 5;
}
```

- `available_cameras` から各カメラの `logical_id`, `resolution_w`, `resolution_h` を取得

#### RolloutCameraImage

```protobuf
message RolloutCameraImage {
    message CameraImage {
        fixed64 frame_start_us = 2;
        fixed64 frame_end_us = 3;
        bytes image_bytes = 4;
        string logical_id = 5;
    }

    string session_uuid = 1;
    CameraImage camera_image = 3;
}
```

#### RolloutEgoTrajectory

```protobuf
message RolloutEgoTrajectory {
    string session_uuid = 1;
    common.Trajectory trajectory = 3;          // local->rig の推定姿勢
    common.DynamicState dynamic_state = 4;     // 速度・加速度
}
```

#### RouteRequest

```protobuf
message Route {
    fixed64 timestamp_us = 2;
    repeated common.Vec3 waypoints = 1;
}

message RouteRequest {
    string session_uuid = 1;
    Route route = 3;
}
```

#### DriveRequest / DriveResponse

```protobuf
message DriveRequest {
    string session_uuid = 1;
    fixed64 time_now_us = 3;
    fixed64 time_query_us = 4;
    bytes renderer_data = 5;  // optional (今回は未使用)
}

message DriveResponse {
    common.Trajectory trajectory = 2;
    DebugInfo debug_info = 3;  // optional
}
```

---

## データフロー（1ステップの流れ）

AlpaSim は各シミュレーションステップで以下のフローで gRPC を呼び出す：

```
AlpaSim loop.py                          Bridge (EgodriverService 実装)
     │                                        │
     ├─ sensorsim.render_rgb() [既存]          │
     │    → 画像取得                           │
     │                                         │
     ├─ asyncio.gather で並列実行:             │
     │   ├─ submit_image_observation() ─gRPC───┤
     │   │   {session_uuid, camera_image}      ├─ 画像データをキャッシュ
     │   │                                     │   (まだ ROS2 には publish しない)
     │   ├─ submit_egomotion_observation() ─┐  │
     │   │   {session_uuid, trajectory,     │  │
     │   │    dynamic_state}                ├─gRPC─┤
     │   │                                  │  ├─ ego 姿勢・速度をキャッシュ
     │   ├─ submit_route() ────────────────┤  │
     │   │   {session_uuid, route}         │  ├─ ルートをキャッシュ
     │   │                                  │  │
     │   └─ submit_recording_ground_truth() │  │
     │       (条件付き、実装では無視)       └──┤
     │                                         │
     ├─ drive() ──────────────gRPC─────────────┤
     │   {session_uuid, time_now_us,           │
     │    time_query_us, renderer_data}        │
     │                                         ├─ 1. /clock publish (time_now_us)
     │        (gRPC 応答待ち)                  ├─ 2. /tf publish (キャッシュした ego pose)
     │                                         ├─ 3. /camera/* publish (キャッシュした画像)
     │                                         ├─ 4. /vehicle/status/velocity publish
     │                                         │
     │                                         ├─ 5. /planning/trajectory を待つ
     │                                         │     ← ROS2 Planner
     │                                         │
     │   DriveResponse ←───────────gRPC────────┤
     │   {trajectory}                          │
     │                                         │
     ├─ controller.run() [既存]                │
     ├─ physics.ground_intersection() [既存]   │
     ├─ traffic.simulate() [既存]              │
     └─ update_pose → Step N+1                 │
```

**注**:
- AlpaSim 内部では `driver.submit_image()`, `driver.submit_trajectory()`, `driver.submit_route()` という高レベル API が使われていますが、実際の gRPC 呼び出しは `submit_image_observation()`, `submit_egomotion_observation()`, `submit_route()` となります
- submit_*() メソッドは **asyncio.gather で並列実行** されるため、gRPC サーバー側では到着順序は保証されません（Bridge 実装では順序に依存しない設計が必要）
- `submit_recording_ground_truth()` は条件付きで呼ばれますが、Bridge 実装では無視します（今回は対応しない）

### バリア同期の実現

gRPC の request/response モデルにより、バリア同期が自然に実現される:

1. AlpaSim が `driver.drive()` を gRPC で呼ぶ
2. Bridge はキャッシュしたデータを ROS2 トピックに publish
3. Bridge は `/planning/trajectory` の受信を**待機**
4. プランナーが trajectory を publish
5. Bridge が trajectory を gRPC レスポンスとして返す
6. AlpaSim が次の処理 (controller) に進む

非リアルタイムなので待機コストは問題にならない。
プランナーの推論速度がシミュレーション速度を決定する。

### セッションステート管理

Bridge 内部で `SessionState` クラスを使用してセッションごとのデータをキャッシュ：

```python
class SessionState:
    def __init__(self, camera_specs: list):
        self.camera_specs = {spec.logical_id: spec for spec in camera_specs}
        self.latest_images = {}           # logical_id -> (bytes, width, height, timestamp)
        self.latest_ego_trajectory = None # Trajectory
        self.latest_dynamic_state = None  # DynamicState
        self.latest_route = None          # List[Vec3]
        self.route_timestamp_us = 0
```

- `submit_*()` メソッド: データをキャッシュするだけ（即座に return）
- `drive()` メソッド: キャッシュされたデータを ROS2 に publish し、trajectory を待つ

---

## リポジトリ構成

Bridge は AlpaSim とは **別リポジトリ** で管理する。

### alpasim-ros2-bridge/ （Bridge リポジトリ）

```
alpasim-ros2-bridge/
├── Dockerfile
├── pyproject.toml            # pytest 設定、メタデータ（ビルドには使わない）
├── .pre-commit-config.yaml
├── alpasim_grpc/             # proto 定義（alpasim からコピー、Docker ビルド時にコンパイル）
│   ├── __init__.py
│   └── v0/
│       ├── __init__.py
│       ├── common.proto      # AlpaSim からコピー
│       ├── egodriver.proto   # AlpaSim からコピー
│       └── sensorsim.proto   # AlpaSim からコピー (CameraSpec 定義に必要)
├── alpasim_ros2_bridge/      # Bridge 本体
│   ├── __init__.py
│   ├── server.py             # EgodriverServicer 実装 + serve() 関数
│   ├── clock_publisher.py
│   ├── sensor_publisher.py
│   ├── tf_broadcaster.py
│   ├── traffic_publisher.py
│   ├── trajectory_listener.py
│   └── conversions.py
├── tests/
│   ├── conftest.py
│   ├── mock_data.py
│   ├── test_conversions.py   # Layer 1 (ROS2 不要)
│   ├── test_clock_publisher.py
│   ├── test_tf_broadcaster.py
│   ├── test_sensor_publisher.py
│   ├── test_traffic_publisher.py
│   ├── test_trajectory_listener.py
│   └── test_server.py        # Layer 3 (gRPC + ROS2)
└── config/
    └── bridge_params.yaml
```

### alpasim/ （AlpaSim リポジトリ）

**変更なし**。Bridge は AlpaSim のコードを一切変更しない。

---

## Docker ビルド戦略

### 設計方針

- **ベースイメージ**: `ros:jazzy`（Ubuntu 24.04 + Python 3.12）
- **autoware_msgs は apt パッケージ**: `ros-jazzy-autoware-{planning,perception,vehicle}-msgs`
- **alpasim_grpc は .proto をローカル管理**: Docker ビルド時に `grpcio-tools` でコンパイル
- **Bridge コードは colcon 不使用**: PYTHONPATH に配置するだけで十分
- **Python パッケージ管理は uv + venv**: `--system-site-packages` で apt の ROS2 パッケージが見える

### Dockerfile

```dockerfile
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install autoware message packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-autoware-planning-msgs \
        ros-jazzy-autoware-perception-msgs \
        ros-jazzy-autoware-vehicle-msgs \
    && rm -rf /var/lib/apt/lists/*

# 2. Install uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# 3. Create venv (--system-site-packages to access rclpy and apt packages)
RUN uv venv --system-site-packages --python python3.12 /opt/venv
ENV VIRTUAL_ENV=/opt/venv PATH="/opt/venv/bin:$PATH"

# 4. Install Python dependencies
RUN uv pip install grpcio grpcio-tools "protobuf>=4.0.0,<5.0.0" \
    dataclasses-json numpy pytest pytest-asyncio

# 5. Copy proto definitions and compile (.proto -> _pb2.py)
COPY alpasim_grpc/ /app/alpasim_grpc/
RUN cd /app && python -c \
    "from grpc_tools.command import build_package_protos; build_package_protos('.', strict_mode=True)"

# 6. Place bridge code on PYTHONPATH
COPY alpasim_ros2_bridge/ /app/alpasim_ros2_bridge/
COPY tests/ /app/tests/
COPY config/ /app/config/

WORKDIR /app

ENTRYPOINT ["/bin/bash", "-c", \
    ". /opt/ros/jazzy/setup.sh && exec \"$@\"", "--"]
CMD ["python3", "-m", "pytest", "tests/", "-v"]
```

### venv + `--system-site-packages` について

uv で venv を作成し `--system-site-packages` を指定することで:
- venv 内から **rclpy, sensor_msgs 等の apt パッケージが見える**
- grpcio, pytest 等は venv 内にクリーンにインストール
- apt パッケージとの競合が発生しない（pluggy 等）
- `--break-system-packages` や `--ignore-installed` 等のハックが不要

---

## AlpaSim との統合

### 必要な変更: なし

AlpaSim のコードや設定ファイルは一切変更しない。

### デプロイ方法

1. Bridge コンテナを **`driver`** という名前で起動：
   ```bash
   docker run --rm --net=host --name driver alpasim-bridge:latest \
     python3 -m alpasim_ros2_bridge.server
   ```

2. AlpaSim コンテナを起動（既存の起動方法と同じ）：
   ```bash
   docker run --rm --net=host alpasim:latest \
     python3 -m alpasim_runtime.main --config /path/to/config.yaml
   ```

3. ROS2 プランナーを起動：
   ```bash
   ros2 run my_planner planner_node --ros-args -p use_sim_time:=true
   ```

AlpaSim は設定ファイルで指定された `driver:50060` に接続する。
Docker ネットワーク上で `driver` という名前が Bridge コンテナに解決されるため、自動的に Bridge に到達する。

---

## 座標系マッピング

| AlpaSim | ROS2 TF frame |
|---------|---------------|
| `local` (world) | `map` |
| `rig` (vehicle body) | `base_link` |
| camera frames | `camera_{name}` |

Quaternion の並び順:
- AlpaSim (protobuf): `Quat { w, x, y, z }`
- ROS2: `geometry_msgs/Quaternion { x, y, z, w }`

変換関数は `conversions.py` で定義：
```python
def pose_from_grpc(grpc_pose: common_pb2.Pose) -> geometry_msgs.msg.Pose
def trajectory_to_grpc(ros_traj: AwTrajectory) -> common_pb2.Trajectory
```

---

## ROS2 プランナー側の要件（ユーザー実装）

- **Subscribe**（入力）:
  - `/camera/{name}/image_raw` (`sensor_msgs/Image`)
  - `/camera/{name}/camera_info` (`sensor_msgs/CameraInfo`)
  - `/tf` (`tf2_msgs/TFMessage`) — ego pose
  - `/vehicle/status/velocity` (`autoware_vehicle_msgs/VelocityReport`)
- **Publish**（出力）:
  - `/planning/trajectory` (`autoware_planning_msgs/Trajectory`)
- **パラメータ**: `use_sim_time: true`

---

## 依存関係

### Bridge Container

```
apt (ros:jazzy ベースイメージに同梱):
  rclpy, sensor_msgs, geometry_msgs, tf2_ros, rosgraph_msgs

apt (追加インストール):
  ros-jazzy-autoware-planning-msgs
  ros-jazzy-autoware-perception-msgs
  ros-jazzy-autoware-vehicle-msgs

uv pip (venv 内にインストール):
  grpcio, grpcio-tools, protobuf>=4.0.0,<5.0.0
  dataclasses-json, numpy, pytest, pytest-asyncio

alpasim_grpc:
  .proto ファイルを Bridge リポジトリに直接配置
  (common.proto, egodriver.proto, sensorsim.proto)
  Docker ビルド時に grpcio-tools で _pb2.py にコンパイル
```

### AlpaSim Container (既存)

**変更なし**。

---

## 実装の鍵となるポイント

### 1. セッション管理

`start_session()` で `DriveSessionRequest.rollout_spec.vehicle.available_cameras` からカメラ情報を取得：

```python
async def start_session(self, request, context):
    session_uuid = request.session_uuid
    camera_specs = [cam.intrinsics for cam in request.rollout_spec.vehicle.available_cameras]

    self._sessions[session_uuid] = SessionState(camera_specs)
    self._sensor_pubs[session_uuid] = SensorPublisher(
        self._node,
        camera_ids=[spec.logical_id for spec in camera_specs]
    )

    return common_pb2.SessionRequestStatus()
```

### 2. データキャッシュ (submit_*() メソッド)

```python
async def submit_image_observation(self, request, context):
    state = self._sessions[request.session_uuid]
    cam_img = request.camera_image

    spec = state.camera_specs[cam_img.logical_id]
    state.latest_images[cam_img.logical_id] = (
        cam_img.image_bytes,
        spec.resolution_w,
        spec.resolution_h,
        cam_img.frame_start_us,
    )
    return common_pb2.Empty()

async def submit_egomotion_observation(self, request, context):
    state = self._sessions[request.session_uuid]
    state.latest_ego_trajectory = request.trajectory
    state.latest_dynamic_state = request.dynamic_state
    return common_pb2.Empty()

async def submit_route(self, request, context):
    state = self._sessions[request.session_uuid]
    state.latest_route = list(request.route.waypoints)
    state.route_timestamp_us = request.route.timestamp_us
    return common_pb2.Empty()

async def submit_recording_ground_truth(self, request, context):
    # Ground truth データは今回の実装では使用しない（無視）
    return common_pb2.Empty()
```

**注**: `submit_recording_ground_truth()` は AlpaSim から条件付きで呼ばれますが、ROS2 プランナーには ground truth は不要なため、Bridge 実装では受信のみ行い、データは破棄します。

### 3. ROS2 統合 (drive() メソッド)

```python
async def drive(self, request, context):
    state = self._sessions[request.session_uuid]
    time_now_us = request.time_now_us

    # 1. Publish /clock
    self._clock_pub.publish(time_now_us)

    # 2. Publish /tf (ego pose)
    if state.latest_ego_trajectory and state.latest_ego_trajectory.poses:
        latest_pose = state.latest_ego_trajectory.poses[-1].pose
        self._tf_broadcaster.send_ego_transform(latest_pose, time_now_us)

    # 3. Publish camera images
    sensor_pub = self._sensor_pubs[request.session_uuid]
    for logical_id, (img_bytes, w, h, timestamp_us) in state.latest_images.items():
        sensor_pub.publish_image(logical_id, img_bytes, w, h, time_now_us)

    # 4. Publish ego velocity
    if state.latest_dynamic_state:
        self._traffic_pub.publish_velocity_report(state.latest_dynamic_state, time_now_us)

    # 5. Wait for planner trajectory
    try:
        trajectory = await self._traj_listener.wait_for_trajectory(timeout_sec=30.0)
        return egodriver_pb2.DriveResponse(trajectory=trajectory)
    except asyncio.TimeoutError:
        logger.warning("Trajectory timeout")
        return egodriver_pb2.DriveResponse()
```

### 4. Trajectory 待機ロジック

`trajectory_listener.py` で `asyncio.Event` を使用：

```python
class TrajectoryListener:
    def __init__(self, node: Node):
        self._event = asyncio.Event()
        self._latest_trajectory = None
        self._sub = node.create_subscription(
            AwTrajectory, "/planning/trajectory", self._callback, 10
        )

    def _callback(self, msg: AwTrajectory):
        self._latest_trajectory = trajectory_to_grpc(msg)
        self._event.set()

    async def wait_for_trajectory(self, timeout_sec: float) -> common_pb2.Trajectory:
        self._event.clear()
        await asyncio.wait_for(self._event.wait(), timeout=timeout_sec)
        return self._latest_trajectory
```

---

## まとめ

このアーキテクチャの利点：

1. **AlpaSim のコード変更が不要** — 完全に分離された設計
2. **AlpaSim の標準インターフェースを使用** — egodriver.proto は AlpaSim が既にサポート
3. **デプロイが簡単** — Docker コンテナ名を `driver` にするだけ
4. **既存の ROS2 コンポーネントを再利用** — publisher/listener は変更不要
5. **テストしやすい** — TDD アプローチで段階的に実装可能

次のステップ：
1. Proto ファイルを AlpaSim からコピー（egodriver.proto, sensorsim.proto, common.proto）
2. テストを書き直す（test_server.py）
3. EgodriverServicer を実装（server.py）
4. Docker ビルドとテスト実行
