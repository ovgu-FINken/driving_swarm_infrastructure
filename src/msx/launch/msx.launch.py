#!/usr/bin/env python

import os
import yaml

import random
import tempfile

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction

import random
import tempfile
import yaml

#Generates n random positions inside of a definded x_range and y_range with an orientation theta
def generate_random_poses(n_robots, x_range=(-0.77, 0.73), y_range=(-1.6, 1.1), theta_range=(-3.14, 3.14)):
    poses = []
    for _ in range(n_robots):
        x = random.uniform(*x_range)
        y = random.uniform(*y_range)
        theta = random.uniform(*theta_range)
        poses.append([x, y, theta])
    return poses


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
            package='msx',
            executable='sunburst_robot_calc',
            namespace=robot,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('topic', f'/{robot}/topic')],
            output='screen',
        ))

    controllers.append(Node(
                package='msx',
                executable='gt_formatting',
                #namespace=robot,
                parameters=[{'use_sim_time': use_sim_time}],
                #remappings=[('topic', f'/{robot}/topic')],
                output='screen',
            ))

    for robot in robots[:n_robots]:
        controllers.append(Node(
           package='msx',
           executable='reactive_behaviour',
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
        'world': 'msx_icra2024.world',
        'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'icra2024.yaml'),
        'robot_names_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'robot_names_sim.yaml'),
        'rosbag_topics_file': os.path.join(get_package_share_directory('trajectory_follower'), 'params', 'rosbag_topics_reactive.yaml'),
        'qos_override_file': os.path.join(get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml')
    }

    robots_file = args['robot_names_file']
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    n_robots = len(robots)

    
    poses = generate_random_poses(n_robots)

    tmp_poses_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(poses, tmp_poses_file, default_flow_style=True)
    tmp_poses_file.close()

    args['poses_file'] = tmp_poses_file.name

    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('driving_swarm_bringup'), 'launch', 'multi_robot.launch.py')),
        launch_arguments=args.items()
    )

    spawn_waldo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(get_package_share_directory('msx'), 'models', 'waldo.sdf'),
            '-entity', 'waldo',
            '-x', '1', '-y', '0', '-z', '0.05'
            ],
            output='screen'
        )
    
    delayed_roofcam = TimerAction(
        period=20.0, # Wait 20 Seconds
        actions=[Node(
            package='msx',
            executable='sunburst_skyview_calc',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )
    
    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(spawn_waldo)
    ld.add_action(delayed_roofcam)
    ld.add_action(DeclareLaunchArgument('data_file', default_value=EnvironmentVariable('DATA_FILE', default_value='data.csv.gz')))
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld
