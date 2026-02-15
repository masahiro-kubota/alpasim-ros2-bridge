# Joy Teleop 機能設計

## 概要

AlpaSim ROS2 Bridge に手動操作機能を追加する。
ジョイスティック（PS4コントローラー等）で車両を操作できるようにし、
開発・デバッグ時の動作確認を容易にする。

## アーキテクチャ

```
┌─────────────────┐
│   joy_node      │  ROS2標準パッケージ
│  (ros2 run joy) │  ジョイスティック入力を /joy トピックに publish
└────────┬────────┘
         │ /joy (sensor_msgs/Joy)
         ▼
┌─────────────────────────┐
│  joy_to_trajectory      │  新規ノード（本機能で追加）
│                         │  joy入力 → trajectory変換
│                         │  速度フィードバック受信
└────┬───────────┬────────┘
     │           ▲ /vehicle/status/velocity (autoware_vehicle_msgs/VelocityReport)
     │           │
     │           └─────────┐
     │                     │
     │ /planning/trajectory (autoware_planning_msgs/Trajectory)
     ▼                     │
┌─────────────────────────┐
│  Bridge                 │  既存の TrajectoryListener
│  (変更不要)              │  trajectory を AlpaSim に転送
│                         │  velocity を ROS2 に公開
└─────────────────────────┘
```

**メリット**:
- Bridge 本体は一切変更不要
- 自律プランナーと手動操作を切り替え可能（ノード起動で制御）
- `/planning/trajectory` という統一インターフェース
- AlpaSim からの実速度フィードバックで正確な速度制御

---

## ジョイスティックマッピング

### デフォルト: PS4 DualShock 4 コントローラー

| 入力 | 軸/ボタン | 機能 | 値の範囲 |
|------|-----------|------|----------|
| 左スティック 横 | axis 0 | **ステアリング** | -1.0 (左旋回) ～ 1.0 (右旋回) |
| R2 トリガー | axis 4 | **スロットル** (加速) | 0.0 (なし) ～ 1.0 (最大) |
| L2 トリガー | axis 5 | **ブレーキ** (減速) | 0.0 (なし) ～ 1.0 (最大) |
| R1 ボタン | button 4 | **デッドマンスイッチ** | 0 (離す=停止) / 1 (押す=有効) |

**パラメータでカスタマイズ可能** — Xbox等の他のコントローラーに対応

---

## Trajectory 生成アルゴリズム

### 入力処理

#### 1. 速度計算

```python
# /vehicle/status/velocity から current_velocity を取得（フィードバック）
# current_velocity の初期値: 0.0 m/s (VelocityReport 受信前)

# update_dt: タイマー周期 = 1.0 / publish_rate_hz (例: 20Hz → 0.05秒)
update_dt = 1.0 / publish_rate_hz

# throttle (0.0~1.0) と brake (0.0~1.0) から目標速度を計算
accel = throttle * accel_rate  # 加速度 [m/s²]
decel = brake * decel_rate      # 減速度 [m/s²]

target_velocity = current_velocity + update_dt * (accel - decel)

# 速度制限
velocity = clamp(target_velocity, min_velocity_mps, max_velocity_mps)
```

**速度フィードバック**:
- トピック: `/vehicle/status/velocity` (autoware_vehicle_msgs/VelocityReport)
- `current_velocity` = `msg.longitudinal_velocity` (AlpaSim からの実速度)
- 初期値: `0.0 m/s` (VelocityReport 未受信時)

**デフォルトパラメータ**:
- `accel_rate`: 2.0 m/s²
- `decel_rate`: 4.0 m/s²
- `max_velocity_mps`: 15.0 m/s (54 km/h)
- `min_velocity_mps`: 0.0 m/s

#### 2. ヨーレート計算

```python
# steering (-1.0~1.0) を直接ヨーレートにマッピング
heading_rate = steering * max_heading_rate_rps
```

**デフォルトパラメータ**:
- `max_heading_rate_rps`: 0.5 rad/s (約 28°/秒)

### Waypoint 生成（簡易 Kinematic Model）

**生成パラメータ**:
- ポイント数: 5
- 時間範囲: 1.0 秒
- 時間間隔: 0.25 秒 (250ms × 4 間隔 = 1.0秒)

**アルゴリズム**:

```python
from scipy.spatial.transform import Rotation

waypoint_dt = time_horizon_sec / (trajectory_points - 1)  # 0.25秒

for i in range(trajectory_points):
    t = i * waypoint_dt

    # 簡易な等速円運動モデル
    if abs(heading_rate) < 1e-6:  # 直進
        dx = velocity * t
        dy = 0.0
        dyaw = 0.0
    else:  # 円弧運動
        if abs(velocity) < 1e-6:  # 速度がほぼ0の場合
            dx = 0.0
            dy = 0.0
            dyaw = 0.0
        else:
            radius = velocity / heading_rate
            dyaw = heading_rate * t
            dx = radius * sin(dyaw)
            dy = radius * (1 - cos(dyaw))

    # map座標系での相対位置（原点からの前進軌道）
    # 初期位置は (0, 0, yaw=0) とし、X軸正方向（前方）への軌道
    point.pose.position.x = dx
    point.pose.position.y = dy
    point.pose.position.z = 0.0

    # Quaternion変換 (scipy使用)
    quat = Rotation.from_euler('z', dyaw).as_quat()  # [x, y, z, w]
    point.pose.orientation.x = quat[0]
    point.pose.orientation.y = quat[1]
    point.pose.orientation.z = quat[2]
    point.pose.orientation.w = quat[3]

    point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
    point.longitudinal_velocity_mps = velocity
    point.heading_rate_rps = heading_rate
```

**座標系**:
- 軌道は **map 座標系**で生成（Autoware 標準に準拠）
- 原点 (0, 0, 0) からの前進軌道（簡易実装）
- X軸: 前方、Y軸: 左方向
- **注**: 厳密な位置制御ではなく、速度・ヨーレートの指令が主目的

**実装の制約**:
- 簡易 kinematic model（等速円運動）
- 車両の実位置は使わず、原点からの相対軌道のみ生成
- AlpaSim は velocity と heading_rate を使って車両を制御
- production 品質ではないが、teleop には十分

---

## Safety 機能

### 1. デッドマンスイッチ (R1 ボタン)

- R1 を **押している間のみ** trajectory を公開
- R1 を **離すと** 空の trajectory (`points=[]`) を公開 → 車両停止

### 2. Joy タイムアウト

- 最後の `/joy` メッセージから **0.5秒** 経過したら停止
- 空の trajectory を公開

```python
if (current_time - last_joy_time) > joy_timeout_sec:
    # 空のtrajectoryを公開
    empty_traj = AwTrajectory()
    empty_traj.header.frame_id = "map"
    empty_traj.header.stamp = self.get_clock().now().to_msg()
    self._trajectory_pub.publish(empty_traj)
```

---

## ROS2 トピック

### Subscribe (入力)

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/joy` | `sensor_msgs/Joy` | ジョイスティック入力 |
| `/vehicle/status/velocity` | `autoware_vehicle_msgs/VelocityReport` | 車両速度フィードバック（AlpaSim から） |

### Publish (出力)

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/planning/trajectory` | `autoware_planning_msgs/Trajectory` | 生成した軌道（Bridge に送信） |

**Trajectory メッセージの構成**:
```python
trajectory.header.frame_id = "map"  # Autoware標準のmap座標系
trajectory.header.stamp = self.get_clock().now().to_msg()
trajectory.points = [...]  # TrajectoryPoint のリスト
```

---

## ROS2 パラメータ

`config/joy_to_trajectory_params.yaml`:

```yaml
joy_to_trajectory:
  ros__parameters:
    # Joystick マッピング
    axis_steering: 0        # 左スティック横
    axis_throttle: 4        # R2 トリガー
    axis_brake: 5           # L2 トリガー
    button_deadman: 4       # R1 ボタン

    # 速度制限
    max_velocity_mps: 15.0  # 最大速度 [m/s]
    min_velocity_mps: 0.0   # 最小速度 [m/s]
    accel_rate: 2.0         # 加速度 [m/s²]
    decel_rate: 4.0         # 減速度 [m/s²]

    # ステアリング
    max_heading_rate_rps: 0.5  # 最大ヨーレート [rad/s]

    # Trajectory 生成
    trajectory_points: 5     # 生成するポイント数
    time_horizon_sec: 1.0    # 予測時間 [秒]
    publish_rate_hz: 20.0    # 公開レート [Hz] (固定レートでタイマー駆動)

    # Safety
    joy_timeout_sec: 0.5     # Joyタイムアウト [秒]
```

**動作モード**:
- タイマー駆動（固定レート 20Hz）で trajectory を公開
- joy メッセージ受信時に内部状態を更新
- デッドマンスイッチが OFF またはタイムアウト時は空の trajectory を公開

**実装の詳細**:
- 速度計算の `update_dt`: `1.0 / publish_rate_hz` (= 0.05秒 @ 20Hz)
- 軌道生成の `waypoint_dt`: `time_horizon_sec / (trajectory_points - 1)` (= 0.25秒)
- `current_velocity` 初期値: `0.0 m/s`
- Trajectory の `frame_id`: `"map"` (Autoware標準)
- Trajectory の `header.stamp`: `self.get_clock().now().to_msg()`
- Quaternion変換: `scipy.spatial.transform.Rotation` を使用

---

## 実装ファイル

| ファイル | 役割 |
|---------|------|
| `alpasim_ros2_bridge/joy_to_trajectory.py` | メインクラス（joy → trajectory 変換） |
| `tests/test_joy_to_trajectory.py` | ユニットテスト |
| `tests/mock_data.py` | テストヘルパー関数（`make_joy_message()` 等） |
| `config/joy_to_trajectory_params.yaml` | パラメータ設定 |
| `launch/joy_to_trajectory.launch.py` | joy_node + joy_to_trajectory 起動 |
| `Dockerfile` | `ros-jazzy-joy`, `scipy` 依存追加 |
| `README.md` | 使い方ドキュメント更新 |

---

## 使い方

### Docker コンテナでの起動

```bash
# ジョイスティックデバイスをマウントしてコンテナを起動
docker run -it --rm --net=host --device=/dev/input \
  alpasim-bridge:latest bash

# ROS2 環境を source
source /opt/ros/jazzy/setup.sh

# Launch ファイルで起動
ros2 launch /app/launch/joy_to_trajectory.launch.py
```

### 手動起動（デバッグ用）

```bash
# ターミナル1: joy_node
ros2 run joy joy_node

# ターミナル2: joy_to_trajectory
python3 -m alpasim_ros2_bridge.joy_to_trajectory \
  --ros-args --params-file /app/config/joy_to_trajectory_params.yaml

# ターミナル3: trajectory 確認
ros2 topic echo /planning/trajectory
```

### 操作方法

1. **R1 ボタンを押し続ける** — デッドマンスイッチ有効
2. **R2 トリガー** — 前進加速
3. **L2 トリガー** — 減速
4. **左スティック 横** — ステアリング（左右旋回）
5. **R1 を離す** — 車両停止

---

## テスト戦略

### Layer 2 テスト（rclpy 必要）

`tests/test_joy_to_trajectory.py`:

```python
class TestJoyToTrajectory:
    def test_no_deadman_publishes_empty_trajectory(self, ros2_node):
        """デッドマンボタンを押していない時は空のtrajectoryを公開"""

    def test_forward_motion_generates_straight_trajectory(self, ros2_node):
        """throttle入力で直進軌道を生成"""

    def test_steering_generates_heading_rate(self, ros2_node):
        """steering入力でheading_rateを設定"""

    def test_trajectory_has_five_points(self, ros2_node):
        """5つのwaypointを生成"""

    def test_trajectory_points_chronological(self, ros2_node):
        """time_from_startが時系列順"""

    def test_velocity_clamped_to_max(self, ros2_node):
        """速度が最大値を超えない"""

    def test_velocity_feedback_affects_acceleration(self, ros2_node):
        """速度フィードバックを使って加減速を計算"""

    def test_joy_timeout_publishes_empty(self, ros2_node):
        """joyタイムアウト後は空のtrajectoryを公開"""

    def test_zero_velocity_with_steering(self, ros2_node):
        """速度0でステアリング入力がある場合の処理"""
```

### 実行方法

```bash
# Docker コンテナ内でテスト実行
docker run --rm alpasim-bridge:latest \
  python3 -m pytest tests/test_joy_to_trajectory.py -v
```

---

## 制限事項

- **簡易 kinematic model** — production 品質ではない
- **原点からの相対軌道** — 車両の実位置を使わない簡易実装
- **速度・ヨーレート指令が主** — 軌道の位置情報は参考値
- **加速度・ジャーク制御なし** — 速度とヨーレートのみ

これらは teleop の用途には十分だが、自律走行プランナーには不適切。

**動作原理**:
- AlpaSim は trajectory の `longitudinal_velocity_mps` と `heading_rate_rps` を使って車両を制御
- 位置情報 (pose) は厳密には使われない可能性が高い
- あくまで「速度」と「旋回率」の指令として機能

---

## 参考資料

- [sensor_msgs/Joy](https://docs.ros2.org/latest/api/sensor_msgs/msg/Joy.html)
- [autoware_planning_msgs/Trajectory](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_planning_msgs)
- [joy package](https://index.ros.org/p/joy/)
