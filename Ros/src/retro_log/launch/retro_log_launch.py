import os
import yaml
import launch

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = "retro_log"
      
def generate_launch_description():

    recordbag = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', 'max-bag-size', '1000000' 'all'],
        output='screen'
    )
        
    return LaunchDescription([recordbag])

    


