from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for GNSS (e.g. /dev/ttyUSB0)'
    )

    node = Node(
        package='gnss_driver',
        executable='gnss_driver',
        name='gnss_driver',
        output='screen',
        parameters=[{'port': LaunchConfiguration('port')}],
    )

    return LaunchDescription([port_arg, node])
