"""Joy Teleop to Trajectory Converter Node.

Subscribes to /joy and /vehicle/status/velocity.
Publishes /planning/scenario_planning/trajectory.

Converts joystick input to an Autoware-compatible trajectory using
a kinematic model with velocity feedback.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Joy
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header


class JoyToTrajectoryNode(Node):
    """Converts Joy input to Autoware Trajectory using kinematic model."""

    def __init__(self):
        super().__init__("joy_to_trajectory")

        # Declare parameters
        self.declare_parameter("update_dt", 0.05)
        self.declare_parameter("time_horizon_sec", 1.0)
        self.declare_parameter("trajectory_points", 5)
        self.declare_parameter("max_speed_mps", 20.0)
        self.declare_parameter("min_speed_mps", 0.0)
        self.declare_parameter("default_speed_mps", 5.0)
        self.declare_parameter("max_acceleration_mps2", 10.0)
        self.declare_parameter("max_deceleration_mps2", 10.0)
        self.declare_parameter("max_steering_rad", 0.5)
        self.declare_parameter("joy_timeout_sec", 1.0)

        # Get parameters
        self.update_dt = self.get_parameter("update_dt").value
        self.time_horizon_sec = self.get_parameter("time_horizon_sec").value
        self.trajectory_points = self.get_parameter("trajectory_points").value
        self.max_speed_mps = self.get_parameter("max_speed_mps").value
        self.min_speed_mps = self.get_parameter("min_speed_mps").value
        self.default_speed_mps = self.get_parameter("default_speed_mps").value
        self.max_acceleration_mps2 = self.get_parameter("max_acceleration_mps2").value
        self.max_deceleration_mps2 = self.get_parameter("max_deceleration_mps2").value
        self.max_steering_rad = self.get_parameter("max_steering_rad").value
        self.joy_timeout_sec = self.get_parameter("joy_timeout_sec").value

        # State
        self._current_velocity_mps: float = self.default_speed_mps
        self._last_joy_msg: Optional[Joy] = None
        self._last_joy_time: Optional[rclpy.time.Time] = None
        self._current_yaw: float = 0.0  # Accumulated yaw in map frame

        # Subscribers
        self._joy_sub = self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self._velocity_sub = self.create_subscription(
            VelocityReport, "/vehicle/status/velocity", self._velocity_callback, 10
        )

        # Publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._traj_pub = self.create_publisher(
            Trajectory, "/planning/scenario_planning/trajectory", qos
        )

        # Timer for trajectory generation
        self._timer = self.create_timer(self.update_dt, self._timer_callback)

        self.get_logger().info("JoyToTrajectory node started")

    def _joy_callback(self, msg: Joy):
        """Store latest Joy message and timestamp."""
        self._last_joy_msg = msg
        self._last_joy_time = self.get_clock().now()

    def _velocity_callback(self, msg: VelocityReport):
        """Update current velocity from vehicle feedback."""
        self._current_velocity_mps = msg.longitudinal_velocity

    def _timer_callback(self):
        """Generate and publish trajectory at fixed rate."""
        # Check Joy timeout
        if self._last_joy_time is None:
            # No Joy input yet, publish zero-velocity trajectory
            self._publish_zero_trajectory()
            return

        time_since_joy = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if time_since_joy > self.joy_timeout_sec:
            # Joy timeout, publish zero-velocity trajectory
            self.get_logger().warn("Joy timeout, publishing zero velocity trajectory")
            self._publish_zero_trajectory()
            return

        # Check deadman switch (X button = buttons[0])
        if self._last_joy_msg is None or len(self._last_joy_msg.buttons) == 0:
            self._publish_zero_trajectory()
            return

        deadman_pressed = self._last_joy_msg.buttons[0] == 1
        if not deadman_pressed:
            # Deadman not pressed, publish zero-velocity trajectory
            self._publish_zero_trajectory()
            return

        # Extract Joy inputs
        steering_input = self._last_joy_msg.axes[0] if len(self._last_joy_msg.axes) > 0 else 0.0
        throttle_input = self._last_joy_msg.axes[4] if len(self._last_joy_msg.axes) > 4 else 0.0

        # Compute target velocity
        if throttle_input > 0.0:
            # Acceleration
            accel = throttle_input * self.max_acceleration_mps2
        elif throttle_input < 0.0:
            # Deceleration
            accel = throttle_input * self.max_deceleration_mps2
        else:
            # Zero throttle, maintain current velocity
            accel = 0.0

        # Update velocity
        target_velocity = self._current_velocity_mps + accel * self.update_dt
        target_velocity = max(self.min_speed_mps, min(self.max_speed_mps, target_velocity))
        self._current_velocity_mps = target_velocity

        # Compute steering rate
        steering_rate_rad_per_sec = steering_input * self.max_steering_rad / self.update_dt

        # Generate trajectory
        trajectory = self._generate_trajectory(target_velocity, steering_rate_rad_per_sec)
        self._traj_pub.publish(trajectory)

    def _publish_zero_trajectory(self):
        """Publish a trajectory with all velocities set to zero."""
        trajectory = self._generate_trajectory(0.0, 0.0)
        self._traj_pub.publish(trajectory)

    def _generate_trajectory(self, velocity_mps: float, steering_rate_rad_per_sec: float) -> Trajectory:
        """Generate trajectory using kinematic model.

        Args:
            velocity_mps: Target velocity (m/s)
            steering_rate_rad_per_sec: Yaw rate (rad/s)

        Returns:
            Trajectory message with waypoints in "map" frame
        """
        trajectory = Trajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "map"

        waypoint_dt = self.time_horizon_sec / (self.trajectory_points - 1)

        # Start from current pose (origin in map frame for simplicity)
        x, y, yaw = 0.0, 0.0, self._current_yaw

        for i in range(self.trajectory_points):
            t = i * waypoint_dt

            # Kinematic model (constant velocity + yaw rate)
            if abs(steering_rate_rad_per_sec) < 1e-6:
                # Straight motion
                dx = velocity_mps * t * math.cos(yaw)
                dy = velocity_mps * t * math.sin(yaw)
                dyaw = 0.0
            else:
                # Circular motion
                radius = velocity_mps / abs(steering_rate_rad_per_sec)
                dyaw = steering_rate_rad_per_sec * t
                dx = radius * (math.sin(yaw + dyaw) - math.sin(yaw))
                dy = radius * (-math.cos(yaw + dyaw) + math.cos(yaw))

            # Create waypoint
            point = TrajectoryPoint()
            point.pose = Pose()
            point.pose.position = Point(x=dx, y=dy, z=0.0)

            # Orientation (yaw as quaternion)
            orientation_yaw = yaw + dyaw
            quat = Rotation.from_euler("z", orientation_yaw).as_quat()  # [x, y, z, w]
            point.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

            # Velocity and heading rate
            point.longitudinal_velocity_mps = velocity_mps
            point.heading_rate_rps = steering_rate_rad_per_sec

            # Time from start
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            trajectory.points.append(point)

        # Update accumulated yaw for next iteration
        self._current_yaw += steering_rate_rad_per_sec * self.update_dt

        return trajectory


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
