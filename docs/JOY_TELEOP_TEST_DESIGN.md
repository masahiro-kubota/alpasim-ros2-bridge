# Joy Teleop テスト設計書

## 概要

本文書は `joy_to_trajectory` ノードのテスト設計を定義する。
テストの目的は、ジョイスティック入力から軌道生成までの正確性と安全性を保証することである。

---

## テスト方針

### テストレベル

**Layer 2 テスト（ROS2統合テスト）**
- rclpy を使用した ROS2 ノードの統合テスト
- 実際のメッセージ型を使用
- Docker コンテナ内で実行

### テスト環境

- **OS**: Ubuntu 24.04 (Docker)
- **ROS2**: Jazzy
- **Python**: 3.12
- **テストフレームワーク**: pytest + pytest-asyncio

### カバレッジ目標

- **機能カバレッジ**: 100%（全機能をテスト）
- **コードカバレッジ**: 80% 以上（主要ロジック）
- **境界値テスト**: すべての数値パラメータ

---

## テストケース一覧

| ID | テストケース名 | 優先度 | カテゴリ |
|----|--------------|--------|---------|
| TC-001 | デッドマンスイッチOFF時の空軌道公開 | 高 | Safety |
| TC-002 | 直進軌道の生成 | 高 | 基本動作 |
| TC-003 | ステアリング入力時のヨーレート設定 | 高 | 基本動作 |
| TC-004 | 5ポイント軌道の生成 | 高 | 基本動作 |
| TC-005 | time_from_start の時系列順序 | 高 | データ整合性 |
| TC-006 | 速度上限のクランプ | 高 | Safety |
| TC-007 | 速度フィードバックによる加減速 | 中 | 速度制御 |
| TC-008 | Joyタイムアウト時の空軌道公開 | 高 | Safety |
| TC-009 | 速度0でのステアリング入力処理 | 中 | エッジケース |
| TC-010 | frame_id が "map" であることの確認 | 高 | データ整合性 |
| TC-011 | Quaternion の正規化確認 | 中 | データ整合性 |
| TC-012 | 負の速度入力の拒否 | 中 | 入力検証 |
| TC-013 | 円弧運動の軌道生成 | 中 | 基本動作 |
| TC-014 | パラメータ変更の反映 | 低 | 設定 |

---

## テストケース詳細

### TC-001: デッドマンスイッチOFF時の空軌道公開

**目的**: デッドマンスイッチが押されていない時、安全のため空の軌道を公開することを確認

**前提条件**:
- joy_to_trajectory ノードが起動済み
- タイマーが動作中

**入力**:
```python
joy_msg = Joy()
joy_msg.axes = [0.0] * 6      # すべて0
joy_msg.buttons = [0] * 5     # button[4] = 0 (デッドマンOFF)
```

**期待される出力**:
```python
trajectory.header.frame_id == "map"
trajectory.header.stamp != None
len(trajectory.points) == 0   # 空の軌道
```

**検証項目**:
- [ ] Trajectory メッセージが公開される
- [ ] points が空リスト
- [ ] header.frame_id が "map"
- [ ] header.stamp が設定されている

---

### TC-002: 直進軌道の生成

**目的**: スロットル入力のみで直進軌道が正しく生成されることを確認

**前提条件**:
- デッドマンスイッチがON
- current_velocity = 0.0 m/s

**入力**:
```python
joy_msg = Joy()
joy_msg.axes = [
    0.0,   # axis 0: steering = 0 (直進)
    0.0, 0.0, 0.0,
    0.5,   # axis 4: throttle = 0.5
    0.0,   # axis 5: brake = 0
]
joy_msg.buttons = [0, 0, 0, 0, 1]  # button 4 = 1 (デッドマンON)
```

**期待される出力**:
```python
len(trajectory.points) == 5

for point in trajectory.points:
    # 直進なので y = 0
    assert abs(point.pose.position.y) < 1e-6
    # x は時間とともに増加
    assert point.pose.position.x >= 0
    # ヨーレートは0
    assert abs(point.heading_rate_rps) < 1e-6
    # 速度は正
    assert point.longitudinal_velocity_mps > 0
```

**検証項目**:
- [ ] 5つのポイントが生成される
- [ ] すべてのポイントで y ≈ 0 (直進)
- [ ] x が単調増加
- [ ] heading_rate_rps ≈ 0
- [ ] longitudinal_velocity_mps > 0

---

### TC-003: ステアリング入力時のヨーレート設定

**目的**: ステアリング入力がヨーレートに正しくマッピングされることを確認

**前提条件**:
- デッドマンスイッチがON
- max_heading_rate_rps = 0.5

**入力**:
```python
# 左旋回 (steering = 1.0)
joy_msg.axes[0] = 1.0   # 左スティック横 = 1.0
joy_msg.axes[4] = 0.5   # throttle
joy_msg.buttons[4] = 1  # デッドマンON
```

**期待される出力**:
```python
for point in trajectory.points:
    # heading_rate = 1.0 * 0.5 = 0.5 rad/s
    assert abs(point.heading_rate_rps - 0.5) < 1e-6
```

**検証項目**:
- [ ] heading_rate_rps = steering * max_heading_rate_rps
- [ ] 右旋回（steering = -1.0）で負のヨーレート

---

### TC-004: 5ポイント軌道の生成

**目的**: パラメータ通り5つのウェイポイントが生成されることを確認

**前提条件**:
- trajectory_points = 5
- time_horizon_sec = 1.0

**入力**: 任意の有効なjoy入力（デッドマンON）

**期待される出力**:
```python
len(trajectory.points) == 5
```

**検証項目**:
- [ ] points のリスト長が5
- [ ] すべてのポイントが有効なデータを持つ

---

### TC-005: time_from_start の時系列順序

**目的**: time_from_start が正しく時系列順に並んでいることを確認

**前提条件**:
- trajectory_points = 5
- time_horizon_sec = 1.0

**入力**: 任意の有効なjoy入力（デッドマンON）

**期待される出力**:
```python
expected_times = [0.0, 0.25, 0.5, 0.75, 1.0]  # 秒

for i, point in enumerate(trajectory.points):
    time_sec = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
    assert abs(time_sec - expected_times[i]) < 1e-6
```

**検証項目**:
- [ ] time_from_start が単調増加
- [ ] 最初のポイントが t=0.0
- [ ] 最後のポイントが t=1.0
- [ ] 間隔が 0.25秒

---

### TC-006: 速度上限のクランプ

**目的**: 速度が max_velocity_mps を超えないことを確認

**前提条件**:
- max_velocity_mps = 15.0
- current_velocity = 14.0 m/s (初期速度)

**入力**:
```python
# フルスロットル
joy_msg.axes[4] = 1.0   # throttle = 1.0
joy_msg.buttons[4] = 1  # デッドマンON
```

**期待される出力**:
```python
for point in trajectory.points:
    assert point.longitudinal_velocity_mps <= 15.0
```

**検証項目**:
- [ ] 速度が max_velocity_mps 以下
- [ ] 加速が max_velocity_mps で飽和

---

### TC-007: 速度フィードバックによる加減速

**目的**: VelocityReport の速度フィードバックが加減速計算に反映されることを確認

**前提条件**:
- accel_rate = 2.0 m/s²
- update_dt = 0.05秒
- current_velocity = 5.0 m/s (VelocityReportから)

**入力**:
```python
# VelocityReport
velocity_msg = VelocityReport()
velocity_msg.longitudinal_velocity = 5.0  # 現在5 m/s

# Joy (スロットル半分)
joy_msg.axes[4] = 0.5   # throttle = 0.5
joy_msg.buttons[4] = 1  # デッドマンON
```

**期待される出力**:
```python
# 加速度 = 0.5 * 2.0 = 1.0 m/s²
# 1ステップ後の速度 = 5.0 + 1.0 * 0.05 = 5.05 m/s
expected_velocity = 5.05

for point in trajectory.points:
    assert abs(point.longitudinal_velocity_mps - expected_velocity) < 0.01
```

**検証項目**:
- [ ] VelocityReport 受信前は 0.0 m/s
- [ ] VelocityReport 受信後は実速度を使用
- [ ] 加速度計算が正しい

---

### TC-008: Joyタイムアウト時の空軌道公開

**目的**: Joy メッセージが一定時間来ない場合、空の軌道を公開することを確認

**前提条件**:
- joy_timeout_sec = 0.5
- 最後のjoyメッセージから0.6秒経過

**入力**:
- joyメッセージを送信しない（タイムアウト待ち）

**期待される出力**:
```python
# 0.6秒後のタイマーコールバック
trajectory.header.frame_id == "map"
len(trajectory.points) == 0   # 空の軌道
```

**検証項目**:
- [ ] タイムアウト前は通常の軌道を公開
- [ ] タイムアウト後は空の軌道を公開
- [ ] タイムアウト後にjoyメッセージを受信すると復帰

---

### TC-009: 速度0でのステアリング入力処理

**目的**: 速度が0の時にステアリング入力があってもクラッシュしないことを確認

**前提条件**:
- current_velocity = 0.0 m/s
- throttle = 0.0
- brake = 0.0

**入力**:
```python
joy_msg.axes[0] = 1.0   # steering = 1.0 (左旋回)
joy_msg.axes[4] = 0.0   # throttle = 0.0
joy_msg.buttons[4] = 1  # デッドマンON
```

**期待される出力**:
```python
# 速度0、ステアリング入力あり
# radius = 0 / heading_rate でゼロ除算にならない
for point in trajectory.points:
    assert point.pose.position.x == 0.0
    assert point.pose.position.y == 0.0
    assert point.longitudinal_velocity_mps == 0.0
```

**検証項目**:
- [ ] クラッシュしない
- [ ] 位置が (0, 0) のまま
- [ ] heading_rate_rps は設定される

---

### TC-010: frame_id が "map" であることの確認

**目的**: Autoware 標準に準拠し、frame_id が "map" であることを確認

**前提条件**: なし

**入力**: 任意の有効なjoy入力

**期待される出力**:
```python
trajectory.header.frame_id == "map"
```

**検証項目**:
- [ ] frame_id が "map"
- [ ] "base_link" ではない

---

### TC-011: Quaternion の正規化確認

**目的**: 生成された Quaternion が正規化されていることを確認

**前提条件**: なし

**入力**: ステアリング入力あり（円弧運動）

**期待される出力**:
```python
for point in trajectory.points:
    q = point.pose.orientation
    norm = sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    assert abs(norm - 1.0) < 1e-6  # 正規化されている
```

**検証項目**:
- [ ] Quaternion のノルムが1.0
- [ ] scipy.spatial.transform.Rotation が正しく使われている

---

### TC-012: 負の速度入力の拒否

**目的**: min_velocity_mps で負の速度が拒否されることを確認

**前提条件**:
- min_velocity_mps = 0.0
- current_velocity = 0.5 m/s

**入力**:
```python
# フルブレーキ（減速が大きすぎて負になる可能性）
joy_msg.axes[5] = 1.0   # brake = 1.0
joy_msg.buttons[4] = 1  # デッドマンON
```

**期待される出力**:
```python
for point in trajectory.points:
    assert point.longitudinal_velocity_mps >= 0.0
```

**検証項目**:
- [ ] 速度が min_velocity_mps 以上
- [ ] 負の速度にならない

---

### TC-013: 円弧運動の軌道生成

**目的**: ステアリング入力時に円弧軌道が正しく生成されることを確認

**前提条件**:
- VelocityReport で current_velocity = 10.0 m/s に設定済み
- max_heading_rate_rps = 0.5 rad/s
- 期待される radius = 10.0 / 0.5 = 20.0 m

**入力**:
```python
# VelocityReport（事前に送信して定常速度を設定）
velocity_msg = VelocityReport()
velocity_msg.longitudinal_velocity = 10.0  # 定常速度 10 m/s

# Joy メッセージ
joy_msg.axes[0] = 1.0   # steering = 1.0 (左旋回)
joy_msg.axes[4] = 0.0   # throttle = 0.0 (定常速度維持)
joy_msg.axes[5] = 0.0   # brake = 0.0
joy_msg.buttons[4] = 1  # デッドマンON
```

**期待される出力**:
```python
# velocity = 10.0 m/s (定常)
# heading_rate = 1.0 * 0.5 = 0.5 rad/s
# radius = 10.0 / 0.5 = 20.0 m

for i, point in enumerate(trajectory.points):
    t = i * 0.25  # 時間
    dyaw = 0.5 * t  # heading_rate * t

    # 円弧の式
    dx = 20.0 * sin(dyaw)
    dy = 20.0 * (1 - cos(dyaw))

    assert abs(point.pose.position.x - dx) < 0.01
    assert abs(point.pose.position.y - dy) < 0.01
    assert abs(point.longitudinal_velocity_mps - 10.0) < 0.01  # 定常速度確認
```

**検証項目**:
- [ ] 円弧の x 座標が正しい
- [ ] 円弧の y 座標が正しい
- [ ] 姿勢が円弧の接線方向
- [ ] 速度が定常（10.0 m/s）のまま

---

### TC-014: パラメータ変更の反映

**目的**: ROS2パラメータの変更が動作に反映されることを確認

**前提条件**:
- カスタムパラメータファイルを使用

**入力**:
```yaml
# カスタムパラメータ
max_velocity_mps: 20.0  # 15.0 → 20.0 に変更
```

**期待される出力**:
```python
# 速度が20.0まで出る
assert max(p.longitudinal_velocity_mps for p in trajectory.points) <= 20.0
```

**検証項目**:
- [ ] パラメータが正しく読み込まれる
- [ ] 変更が動作に反映される

---

## テストデータ

### Joy メッセージのテンプレート

```python
def make_joy_message(
    steering: float = 0.0,
    throttle: float = 0.0,
    brake: float = 0.0,
    deadman: bool = False,
) -> Joy:
    """Joyメッセージを生成するヘルパー関数"""
    msg = Joy()
    msg.axes = [
        steering,  # axis 0
        0.0,       # axis 1
        0.0,       # axis 2
        0.0,       # axis 3
        throttle,  # axis 4
        brake,     # axis 5
    ]
    msg.buttons = [0, 0, 0, 0, 1 if deadman else 0]
    return msg
```

### VelocityReport メッセージのテンプレート

```python
def make_velocity_report(
    longitudinal_velocity: float = 0.0,
    lateral_velocity: float = 0.0,
    heading_rate: float = 0.0,
) -> VelocityReport:
    """VelocityReportメッセージを生成するヘルパー関数"""
    msg = VelocityReport()
    msg.longitudinal_velocity = longitudinal_velocity
    msg.lateral_velocity = lateral_velocity
    msg.heading_rate = heading_rate
    return msg
```

---

## テスト実装構造

### ファイル構成

```
tests/
├── conftest.py                  # pytest設定、フィクスチャ
├── mock_data.py                 # テストヘルパー関数
└── test_joy_to_trajectory.py    # メインテスト
```

### pytest フィクスチャ

```python
@pytest.fixture
def ros2_node():
    """ROS2ノードの初期化とクリーンアップ"""
    rclpy.init()
    node = rclpy.create_node('test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def joy_to_traj_node(ros2_node):
    """joy_to_trajectoryノードのインスタンス"""
    from alpasim_ros2_bridge.joy_to_trajectory import JoyToTrajectory

    # カスタムパラメータ
    node = JoyToTrajectory()
    yield node
    node.destroy_node()
```

---

## テスト実行方法

### Docker コンテナ内でのテスト実行

```bash
# すべてのテストを実行
docker run --rm alpasim-bridge:latest \
  python3 -m pytest tests/test_joy_to_trajectory.py -v

# 特定のテストを実行
docker run --rm alpasim-bridge:latest \
  python3 -m pytest tests/test_joy_to_trajectory.py::TestJoyToTrajectory::test_no_deadman_publishes_empty_trajectory -v

# カバレッジ付き実行
docker run --rm alpasim-bridge:latest \
  python3 -m pytest tests/test_joy_to_trajectory.py --cov=alpasim_ros2_bridge.joy_to_trajectory --cov-report=html
```

---

## 成功基準

### テスト合格条件

- [ ] すべてのテストケースが PASS
- [ ] コードカバレッジ 80% 以上
- [ ] Safety 関連のテスト（TC-001, TC-006, TC-008）がすべて PASS
- [ ] 境界値テスト（速度0、最大速度）がすべて PASS

### リリース基準

- [ ] すべてのテストが継続的に成功
- [ ] Docker コンテナ内でテストが実行可能
- [ ] テストの実行時間が 30秒以内

---

## 既知の制約事項

- **タイミング依存**: タイマー駆動のため、テストが時間に依存する
  - VelocityReport → Joy → Timer callback の順序を保証する必要あり
  - テスト実装時は `rclpy.spin_once()` で明示的にコールバックを処理
  - または、十分な待機時間（100ms程度）を設ける
- **ROS2環境必須**: Layer 2 テストは rclpy が必須
- **モック不使用**: 実際のメッセージ型を使用（シンプルだが環境依存）
- **定常速度の再現**: TC-013等で定常速度を再現するには、VelocityReportで事前に速度を設定する必要あり

---

## 参考資料

- [pytest-ros documentation](https://github.com/pytest-dev/pytest-ros)
- [ROS2 Testing Guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html)
- [autoware_planning_msgs/Trajectory](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_planning_msgs)
