#!/usr/bin/env python

import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def controller_spawning(context, *args, **kwargs):
    controllers = []

    n_robots = LaunchConfiguration('n_robots').perform(context)
    robots_file = LaunchConfiguration('robot_names_file').perform(context)
    use_sim_time = TextSubstitution(text='true')
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
        
    for robot in robots[:int(n_robots)]:
        controllers.append(Node(
           package='driving_swarm_behaviour',
           executable='reactive',
           namespace=robot,
           parameters=[{
            'use_sim_time': use_sim_time,
            }],
           remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
           output='screen',
        ))
    
    return controllers


def generate_launch_description():
    args = {
         'behaviour': 'false',
         'world': 'icra2024.world',
         'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'icra2024.yaml'),
         'poses_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2024_real_poses.yaml'),
         'robot_names_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'robot_names_sim.yaml'),
         'rosbag_topics_file': os.path.join(get_package_share_directory('trajectory_follower'), 'params', 'rosbag_topics_reactive.yaml'),
         'qos_override_file': os.path.join(get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml')
    }
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('driving_swarm_bringup'), 'launch', 'multi_robot.launch.py')),
        launch_arguments=args.items())

    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld
