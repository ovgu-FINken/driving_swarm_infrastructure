#!/usr/bin/env python

import os
import launch_ros
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def controller_spawning(context, *args, **kwargs):

    #robots_file = LaunchConfiguration('robot_names_file').perform(context)
    robots_file = os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'robot_names_real.yaml')
    
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    
    return [Node(
        executable='battery_watcher',
        package='experiment_supervisor',
        output='screen',
        parameters=[{'robot_names': robots}]
        #arguments=[],
        
    )]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld