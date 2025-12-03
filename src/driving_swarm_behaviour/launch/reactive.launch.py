#!/usr/bin/env python

import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def controller_spawning(context, *args, **kwargs):
    controllers = []

    n_robots = int(LaunchConfiguration('n_robots').perform(context))
    robots_file = LaunchConfiguration('robot_names_file').perform(context)
    use_sim_time = TextSubstitution(text='true')
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
        
    controllers.append(Node(
       package='experiment_measurement',
       executable='direct_data_export',
       parameters=[{
          'use_sim_time': use_sim_time,
          'robot_names': robots[:n_robots],
          'data_export_config_file': os.path.join(get_package_share_directory('ccr'), 'params', 'data_export.yaml'),
          'data_file': LaunchConfiguration('data_file').perform(context),
       }],
       output='both',
    ))

    for robot in robots[:n_robots]:
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
    }
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('driving_swarm_bringup'), 'launch', 'multi_robot.launch.py')),
        launch_arguments=args.items())

    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(DeclareLaunchArgument('data_file', default_value=EnvironmentVariable('DATA_FILE', default_value='data.csv.gz')))
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld
