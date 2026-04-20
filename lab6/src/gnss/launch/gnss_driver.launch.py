from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB1",
                              description="Serial port for GPS puck"),
        DeclareLaunchArgument("baud", default_value="4800",
                              description="Baud rate"),
        Node(
            package="gnss",
            executable="gnss_node",
            name="gnss_driver",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baud": LaunchConfiguration("baud"),
            }],
        ),
    ])
