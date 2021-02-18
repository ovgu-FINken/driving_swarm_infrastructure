#!/usr/bin/env python


import os
import launch_ros
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'map_topic': 'map',
        'occupied_thresh': 1,
        'box_height': 0.3,
        'export_dir': get_package_share_directory('map2gazebo') 
    } 

    map2gazebo_node = Node(
        executable='map2gazebo',
        package='map2gazebo',
        namespace='robot1',
        parameters=[params]
    )

    ld = LaunchDescription()
    ld.add_action(map2gazebo_node)
    return ld