from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnss_rtk',
            executable='rtk_driver',
            name='rtk_driver',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 4800},
                {'topic_name': '/rtk_gnss'}
            ]
        )
    ])
