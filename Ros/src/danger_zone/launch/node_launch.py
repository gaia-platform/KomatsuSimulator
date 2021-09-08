import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

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
