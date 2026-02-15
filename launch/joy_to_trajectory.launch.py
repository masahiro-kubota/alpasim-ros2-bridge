"""Launch file for Joy Teleop to Trajectory node.

This launch file starts:
1. joy_node (sensor_msgs/Joy publisher from /dev/input/js0)
2. joy_to_trajectory node (converts Joy to Trajectory)

Usage:
    ros2 launch alpasim_ros2_bridge joy_to_trajectory.launch.py

Note: Requires a PS4 DualShock 4 controller connected via USB or Bluetooth.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Joy Teleop system."""

    # Get package directory (if using ament package structure)
    # For now, use relative path from workspace root
    config_file = Path(__file__).parent.parent / "config" / "joy_to_trajectory_params.yaml"

    return LaunchDescription([
        # Joy node (reads /dev/input/js0 and publishes to /joy)
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "device_id": 0,
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }],
            output="screen",
        ),

        # Joy to Trajectory converter
        Node(
            package="alpasim_ros2_bridge",
            executable="joy_to_trajectory",
            name="joy_to_trajectory",
            parameters=[str(config_file)],
            output="screen",
        ),
    ])
