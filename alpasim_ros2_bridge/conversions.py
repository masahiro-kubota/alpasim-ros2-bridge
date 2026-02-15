"""AlpaSim <-> ROS2 type conversion utilities.

Operates on dicts and protobuf types only (no ROS2 message imports),
so Layer 1 tests can run without rclpy.
"""

from __future__ import annotations

from alpasim_grpc.v0 import common_pb2


# ---------------------------------------------------------------------------
# Timestamp conversion
# ---------------------------------------------------------------------------

def timestamp_us_to_sec_nanosec(timestamp_us: int) -> tuple[int, int]:
    """AlpaSim timestamp_us → ROS2 (sec, nanosec)"""
    sec = int(timestamp_us // 1_000_000)
    nanosec = int((timestamp_us % 1_000_000) * 1_000)
    return sec, nanosec


def sec_nanosec_to_timestamp_us(sec: int, nanosec: int) -> int:
    """ROS2 (sec, nanosec) → AlpaSim timestamp_us"""
    return sec * 1_000_000 + nanosec // 1_000


# ---------------------------------------------------------------------------
# Quaternion conversion
# ---------------------------------------------------------------------------

def alpasim_quat_to_ros(w: float, x: float, y: float, z: float) -> tuple[float, float, float, float]:
    """AlpaSim Quat(w,x,y,z) → ROS2 (x,y,z,w)"""
    return (x, y, z, w)


def ros_quat_to_alpasim(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    """ROS2 (x,y,z,w) → AlpaSim Quat(w,x,y,z)"""
    return (w, x, y, z)


# ---------------------------------------------------------------------------
# Pose conversion
# ---------------------------------------------------------------------------

def alpasim_pose_to_ros_pose(pose: common_pb2.Pose) -> dict:
    """Convert AlpaSim Pose to a ROS2-compatible dict (position, orientation).

    Returns a plain dict so that ROS2 message types are not required here.
    The caller (ROS2 node layer) converts it to geometry_msgs/Pose.
    """
    q = pose.quat
    return {
        "position": {
            "x": pose.vec.x,
            "y": pose.vec.y,
            "z": pose.vec.z,
        },
        "orientation": {
            "x": q.x,
            "y": q.y,
            "z": q.z,
            "w": q.w,
        },
    }


# ---------------------------------------------------------------------------
# Trajectory conversion (autoware -> AlpaSim protobuf)
# ---------------------------------------------------------------------------

def autoware_trajectory_to_alpasim(
    points: list[dict],
    header_stamp_sec: int,
    header_stamp_nanosec: int,
) -> common_pb2.Trajectory:
    """Convert a list of autoware TrajectoryPoint dicts to AlpaSim protobuf Trajectory.

    Each point dict has the structure:
        {
            "pose": {"position": {"x", "y", "z"}, "orientation": {"x", "y", "z", "w"}},
            "time_from_start": {"sec": int, "nanosec": int},
            "longitudinal_velocity_mps": float,
            "heading_rate_rps": float,
        }

    header_stamp is the base time (sec, nanosec). Each point's time_from_start
    is added to compute the absolute timestamp_us.
    """
    base_us = sec_nanosec_to_timestamp_us(header_stamp_sec, header_stamp_nanosec)

    grpc_poses = []
    for pt in points:
        tfs = pt["time_from_start"]
        offset_us = sec_nanosec_to_timestamp_us(tfs["sec"], tfs["nanosec"])
        timestamp_us = base_us + offset_us

        ori = pt["pose"]["orientation"]
        pos = pt["pose"]["position"]

        grpc_pose = common_pb2.PoseAtTime(
            timestamp_us=timestamp_us,
            pose=common_pb2.Pose(
                vec=common_pb2.Vec3(x=pos["x"], y=pos["y"], z=pos["z"]),
                quat=common_pb2.Quat(w=ori["w"], x=ori["x"], y=ori["y"], z=ori["z"]),
            ),
        )
        grpc_poses.append(grpc_pose)

    return common_pb2.Trajectory(poses=grpc_poses)
