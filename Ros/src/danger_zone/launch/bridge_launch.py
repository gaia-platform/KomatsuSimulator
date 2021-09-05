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

    unity_bridge_node = Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_bridge',
            emulate_tty=True,
            parameters=[
                {'/ROS_IP': '127.0.0.1'},
                {'/ROS_TCP_PORT': 10000},
            ]
        )

    return LaunchDescription([unity_bridge_node])
