from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    danger_zone_node = Node(
        package='danger_zone',
        namespace='gaia',
        executable='danger_zone',
        name='danger_zone',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([danger_zone_node])
