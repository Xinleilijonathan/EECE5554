from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the VectorNav IMU'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate'
        ),
        Node(
            package='imu',
            executable='driver',
            name='vn_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }]
        ),
    ])
