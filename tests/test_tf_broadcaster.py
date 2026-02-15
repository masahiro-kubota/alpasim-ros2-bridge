"""Tests for TFBroadcaster (Layer 2: requires rclpy).

Verifies that ego and traffic poses are broadcast correctly on /tf.
"""

pytest = __import__("pytest")
rclpy = pytest.importorskip("rclpy")
tf2_ros = pytest.importorskip("tf2_ros")

from tf2_ros import Buffer, TransformListener

from alpasim_ros2_bridge.tf_broadcaster import TFBroadcaster
from tests.mock_data import make_pose


class TestTFBroadcaster:

    def test_ego_transform(self, ros2_node):
        """Ego pose should be broadcast as map -> base_link."""
        tf_buffer = Buffer()
        TransformListener(tf_buffer, ros2_node)

        broadcaster = TFBroadcaster(ros2_node)
        pose = make_pose(x=10.0, y=5.0, z=0.0)
        broadcaster.send_ego_transform(pose, timestamp_us=1_000_000)

        rclpy.spin_once(ros2_node, timeout_sec=0.1)

        transform = tf_buffer.lookup_transform(
            "map", "base_link",
            rclpy.time.Time(seconds=1),
        )
        assert transform.transform.translation.x == pytest.approx(10.0)
        assert transform.transform.translation.y == pytest.approx(5.0)

    def test_quaternion_order(self, ros2_node):
        """AlpaSim(w,x,y,z) -> ROS2(x,y,z,w) conversion should be correct."""
        tf_buffer = Buffer()
        TransformListener(tf_buffer, ros2_node)

        broadcaster = TFBroadcaster(ros2_node)
        # 90 degree Z-axis rotation: w=0.707, x=0, y=0, z=0.707
        pose = make_pose(qw=0.707, qx=0.0, qy=0.0, qz=0.707)
        broadcaster.send_ego_transform(pose, timestamp_us=0)

        rclpy.spin_once(ros2_node, timeout_sec=0.1)

        transform = tf_buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time()
        )
        q = transform.transform.rotation
        assert q.w == pytest.approx(0.707, abs=0.01)
        assert q.z == pytest.approx(0.707, abs=0.01)

    def test_traffic_actors(self, ros2_node):
        """Traffic actors should be broadcast as map -> actor_{id}."""
        tf_buffer = Buffer()
        TransformListener(tf_buffer, ros2_node)

        broadcaster = TFBroadcaster(ros2_node)
        broadcaster.send_actor_transform(
            actor_id="vehicle_42",
            pose=make_pose(x=20.0, y=-3.0),
            timestamp_us=1_000_000,
        )

        rclpy.spin_once(ros2_node, timeout_sec=0.1)

        transform = tf_buffer.lookup_transform(
            "map", "actor_vehicle_42",
            rclpy.time.Time(seconds=1),
        )
        assert transform.transform.translation.x == pytest.approx(20.0)
