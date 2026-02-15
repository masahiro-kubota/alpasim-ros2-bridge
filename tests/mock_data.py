"""Mock data generators for AlpaSim service outputs.

Builds alpasim_grpc protobuf types directly for use in tests.
"""

import numpy as np
from alpasim_grpc.v0 import common_pb2, egodriver_pb2


def make_pose(x=0.0, y=0.0, z=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    """Create a mock AlpaSim Pose."""
    return common_pb2.Pose(
        vec=common_pb2.Vec3(x=x, y=y, z=z),
        quat=common_pb2.Quat(w=qw, x=qx, y=qy, z=qz),
    )


def make_pose_at_time(timestamp_us, x=0.0, y=0.0, z=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    """Create a mock PoseAtTime."""
    return common_pb2.PoseAtTime(
        pose=make_pose(x=x, y=y, z=z, qw=qw, qx=qx, qy=qy, qz=qz),
        timestamp_us=timestamp_us,
    )


def make_dynamic_state(vx=10.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
    """Create a mock DynamicState."""
    return common_pb2.DynamicState(
        linear_velocity=common_pb2.Vec3(x=vx, y=vy, z=vz),
        angular_velocity=common_pb2.Vec3(x=wx, y=wy, z=wz),
    )


def make_trajectory(n_points=5, dt_us=100_000, start_us=0):
    """Create a mock trajectory with evenly spaced waypoints."""
    poses = []
    for i in range(n_points):
        t = start_us + i * dt_us
        poses.append(make_pose_at_time(t, x=float(i) * 1.0))
    return common_pb2.Trajectory(poses=poses)


def make_rgb_image(width=640, height=480):
    """Create a random RGB image as bytes for testing."""
    return np.random.randint(0, 255, (height, width, 3), dtype=np.uint8).tobytes()


def make_traffic_object(object_id, x=0.0, y=0.0, z=0.0, size_x=4.5, size_y=2.0, size_z=1.5,
                        is_static=False):
    """Create a mock traffic object dict."""
    return {
        "object_id": object_id,
        "pose": make_pose(x=x, y=y, z=z),
        "aabb": common_pb2.AABB(size_x=size_x, size_y=size_y, size_z=size_z),
        "is_static": is_static,
    }


def make_traffic_objects(n_actors=3, timestamp_us=0):
    """Create a list of mock traffic objects."""
    return [
        make_traffic_object(
            object_id=f"actor_{i}",
            x=float(i) * 10.0,
            y=5.0,
        )
        for i in range(n_actors)
    ]


def make_joy_message(
    axes_l_stick_lr=0.0,
    axes_l_stick_ud=0.0,
    axes_l2_trigger=0.0,
    axes_r_stick_lr=0.0,
    axes_r_stick_ud=0.0,
    axes_r2_trigger=0.0,
    button_x=0,
):
    """Create a mock Joy message dict (PS4 DualShock 4 layout).

    Returns a dict matching sensor_msgs/Joy structure:
        {
            "axes": [l_lr, l_ud, l2, r_lr, r_ud, r2, dpad_lr, dpad_ud],
            "buttons": [x, o, tri, sq, l1, r1, l2, r2, share, options, ps, l3, r3, dpad_up, dpad_right, dpad_down, dpad_left, center],
        }

    For trajectory generation:
        - axes[0]: L-stick left/right (steering)
        - axes[4]: R-stick up/down (throttle/brake)
        - buttons[0]: X button (deadman switch)
    """
    return {
        "axes": [
            axes_l_stick_lr,
            axes_l_stick_ud,
            axes_l2_trigger,
            axes_r_stick_lr,
            axes_r_stick_ud,
            axes_r2_trigger,
            0.0,  # dpad_lr
            0.0,  # dpad_ud
        ],
        "buttons": [
            button_x,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ],
    }


def make_velocity_report(longitudinal_velocity=0.0, lateral_velocity=0.0, heading_rate=0.0):
    """Create a mock VelocityReport message dict.

    Returns a dict matching autoware_vehicle_msgs/VelocityReport:
        {
            "longitudinal_velocity": float,
            "lateral_velocity": float,
            "heading_rate": float,
        }
    """
    return {
        "longitudinal_velocity": longitudinal_velocity,
        "lateral_velocity": lateral_velocity,
        "heading_rate": heading_rate,
    }


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
