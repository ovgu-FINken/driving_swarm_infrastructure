#!/usr/bin/env python

import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def controller_spawning(context, *args, **kwargs):
    controllers = []

    n_robots = LaunchConfiguration('n_robots').perform(context)
    n_robots = int(n_robots)
    robots_file = LaunchConfiguration('robot_names_file').perform(context)
    waypoints_file = LaunchConfiguration('waypoints_file').perform(context)
    use_sim_time = TextSubstitution(text='True')
    grid_params = {
              'graph_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps', 'icra2024.yaml'),
              'x_min': -2.25,
              'x_max': 2.75,
              'y_min': -1.75,
              'y_max': 1.25,
              'grid_type': 'square',
              'grid_size': 0.5,
              'inflation_size': 0.1,
              'laser_inflation_size': 0.1,} 
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    with open(waypoints_file, 'r') as stream:
         waypoints = yaml.safe_load(stream)
    
    for wp, robot in zip(waypoints[:n_robots], robots[:n_robots]):
        controllers.append(Node(
           package='goal_provider',
           executable='simple_goal',
           namespace=robot,
           parameters=[{
              'use_sim_time': use_sim_time,
              'waypoints': yaml.dump(wp['waypoints']),
           }],
           remappings=[('/tf',"tf"), ('/tf_static',"tf_static")],
           output='screen',
        ))
        controllers.append(Node(
           package='trajectory_follower',
           executable='dwa',
           namespace=robot,
           parameters=[
              {
                  "use_sim_time": use_sim_time,
                  "dt": 2.0,
                  "w1": 1.2,
                  "w2": 0.9,
                  "w3": 0.9,
                  "obstacle_threshold": 0.17,
                  "tb_radius": 0.3,
                  "n_samples": 7,
                  "fail_radius": 0.3
              }
           ],
           remappings=[('/tf',"tf"), ('/tf_static',"tf_static")],
           output='screen',
        ))
        controllers.append(Node(
           package='ccr',
           executable='ccr_local_planner',
           namespace=robot,
           parameters=[{
              'use_sim_time': use_sim_time,
              'robot_names': robots[:n_robots],
           }, grid_params
             ],
           remappings=[('/tf',"tf"), ('/tf_static',"tf_static")],
           output='screen',
        ))
        controllers.append(Node(
           package='ccr',
           executable='ccr_global_planner',
           namespace=robot,
           parameters=[{
              'use_sim_time': use_sim_time,
              'robot_names': robots[:n_robots],
           }, grid_params
              ],
           output='screen',
        ))
    
    return controllers


def generate_launch_description():
    args = {
         'behaviour': 'false',      
         'world': 'icra2024.world',
         'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'icra2024.yaml'),
         'robot_names_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'robot_names_sim.yaml'),
         'waypoints_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2024_real_waypoints.yaml'),
         'poses_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2024_real_poses.yaml'),
         'rosbag_topics_file': os.path.join(get_package_share_directory('trajectory_follower'), 'params', 'rosbag_topics.yaml'),
         'qos_override_file': os.path.join(get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml')
    }
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('driving_swarm_bringup'), 'launch', 'multi_robot.launch.py')),
        launch_arguments=args.items())

    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld