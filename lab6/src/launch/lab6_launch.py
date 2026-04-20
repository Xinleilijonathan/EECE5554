"""
lab6_launch.py

Single launch file that brings up the IMU (VectorNav VN-100) and GPS (standalone puck)
drivers simultaneously for NUance Lab 6 data collection.

Usage:
    ros2 launch lab6_launch.py
    ros2 launch lab6_launch.py imu_port:=/dev/ttyUSB0 gps_port:=/dev/ttyUSB1

Notes:
  - The NUance on-board drivers (LiDAR, camera, radar) are started separately
    on the Nuvo PC via: ros2 launch username main.launch.xml
  - Adjust imu_port and gps_port to the actual /dev/ttyUSB* devices.
    Run `ls /dev/ttyUSB*` before launching to confirm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── IMU port / baud ──────────────────────────────────────────────
        DeclareLaunchArgument(
            "imu_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for VectorNav VN-100 IMU",
        ),
        DeclareLaunchArgument(
            "imu_baud",
            default_value="115200",
            description="Baud rate for VectorNav VN-100",
        ),
        # ── GPS port / baud ──────────────────────────────────────────────
        DeclareLaunchArgument(
            "gps_port",
            default_value="/dev/ttyUSB1",
            description="Serial port for standalone GPS puck",
        ),
        DeclareLaunchArgument(
            "gps_baud",
            default_value="4800",
            description="Baud rate for GPS puck",
        ),
        # ── IMU node ─────────────────────────────────────────────────────
        Node(
            package="imu",
            executable="driver",
            name="vn_driver",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("imu_port"),
                "baud": LaunchConfiguration("imu_baud"),
            }],
        ),
        # ── GPS node ─────────────────────────────────────────────────────
        Node(
            package="gnss",
            executable="gnss_node",
            name="gnss_driver",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("gps_port"),
                "baud": LaunchConfiguration("gps_baud"),
            }],
        ),
    ])
