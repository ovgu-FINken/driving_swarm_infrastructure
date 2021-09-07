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
    robots_file = LaunchConfiguration('robots_file').perform(context)
    use_sim_time = TextSubstitution(text='true')
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
        
    controllers.append(Node(
       package='driving_swarm_nav_graph',
       executable='global_graph_planner',
       parameters=[{
        'use_sim_time': use_sim_time,
        'robot_names': [robot['name'] for robot in robots[:int(n_robots)]],
        'tiling': LaunchConfiguration('tiling'),
        # graph file can either be a .yaml for a map, or a file containing an xml-representation for the graph
        'graph_file': os.path.join(
           get_package_share_directory('driving_swarm_bringup'),
           'maps',
           'icra2021_map.yaml'),
        }],
       output='screen',
    ))
    
    for robot in robots[:int(n_robots)]:
        controllers.append(Node(
           package='goal_provider',
           executable='simple_goal',
           namespace=robot['name'],
           parameters=[{
              'use_sim_time': use_sim_time,
              'x': robot['goals']['x'],
              'y': robot['goals']['y'],
              'theta': robot['goals']['theta'],
           }],
           output='screen',
        ))
        controllers.append(Node(
           package='system_status',
           executable='scan_delay',
           namespace=robot['name'],
           parameters=[{
              'use_sim_time': use_sim_time,
           }],
           output='screen',
        ))

        controllers.append(Node(
           package='driving_swarm_nav_graph',
           executable='nav_graph_planner',
           namespace=robot['name'],
           parameters=[{
            'use_sim_time': use_sim_time,
            'vehicle_model': 3,
            'turn_radius': 0.2,
            'turn_speed': 0.5,
            'step_size': 0.1,
            'tiling': LaunchConfiguration('tiling'),
            # graph file can either be a .yaml for a map, or a file containing an xml-representation for the graph
            'graph_file': os.path.join(
               get_package_share_directory('driving_swarm_bringup'),
               'maps',
               'icra2021_map.yaml'),
            }],
           output='screen',
        ))
        controllers.append(Node(
           package='trajectory_follower',
           executable='trajectory_follower',
           namespace=robot['name'],
           parameters=[
              {
                  "use_sim_time": use_sim_time,
                  "dt": 2.0,
                  "w1": 0.5,
                  "w2": 0.5,
                  "fail_radius": 0.3
              }
           ],
           output='screen',
        ))
    
    return controllers


def generate_launch_description():
    args = {
         'behaviour': 'false',
         'world': 'icra2021_world.world',
         'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'icra2021_map.yaml'),
         'robots_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2021.yaml'),
         'rosbag_topics_file': os.path.join(get_package_share_directory('trajectory_follower'), 'params', 'rosbag_topics.yaml'),
         'qos_override_file': os.path.join(get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml')
    }
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('driving_swarm_bringup'), 'launch', 'multi_robot.launch.py')),
        launch_arguments=args.items())

    declare_tiling_cmd = DeclareLaunchArgument(
        'tiling',
        default_value='hex'
    )

    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(declare_tiling_cmd)
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld