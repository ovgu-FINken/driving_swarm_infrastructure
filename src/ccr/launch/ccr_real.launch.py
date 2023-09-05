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
    param_file = os.path.join(get_package_share_directory('ccr'), 'params', 'ccr_params.yaml')
    with open(param_file, 'r') as stream:
         params = yaml.safe_load(stream)
         
    grid_params = params['grid_params']
    grid_params['graph_file'] = os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps', 'icra2024.yaml')
    local_planner_params = params['local_planner_params']
    global_planner_params = params['global_planner_params']
    dwa_params = params['dwa_params']
    

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
              'waypoints': yaml.dump(wp['waypoints']),
              'goal_radius': 0.25,
           }],
           remappings=[('/tf',"tf"), ('/tf_static',"tf_static")],
           output='both',
        ))
        controllers.append(Node(
           package='trajectory_follower',
           executable='dwa',
           namespace=robot,
           parameters=[
              dwa_params
           ],
           remappings=[('/tf',"tf"), ('/tf_static',"tf_static")],
           output='log',
        ))
        # we need to pass all params to the local planner and global planner, so the exact same graph is generated
        controllers.append(Node(
           package='ccr',
           executable='ccr_local_planner',
           namespace=robot,
           parameters=[{
           }, grid_params, local_planner_params, global_planner_params
             ],
           remappings=[('/tf',"tf"), ('/tf_static',"tf_static")],
           output='both',
        ))
        controllers.append(Node(
           package='ccr',
           executable='ccr_global_planner',
           namespace=robot,
           parameters=[{
              'robot_names': robots[:n_robots],
           }, grid_params, local_planner_params, global_planner_params
              ],
           output='both',
        ))
    
    return controllers


def generate_launch_description():
    args = {
         'behaviour': 'false',      
         'world': 'icra2024.world',
         'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'icra2024.yaml'),
         'robot_names_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'robot_names_real.yaml'),
         'waypoints_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2024_waypoints.yaml'),
         'poses_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2024_poses.yaml'),
         'rosbag_topics_file': os.path.join(get_package_share_directory('trajectory_follower'), 'params', 'rosbag_topics.yaml'),
         'qos_override_file': os.path.join(get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml'),
    }
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('driving_swarm_bringup'), 'launch', 'multi_real_robot_fix_pos.launch.py')),
        launch_arguments=args.items())

    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld
