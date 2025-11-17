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
import numpy as np

# Generates n random positions inside of a definded x_range and y_range with an orientation theta
def generate_random_poses(n_robots, x_range=(-0.8, 0.8), y_range=(-0.8, 0.8), theta_range=(-3.14, 3.14)):
    closeness_threshold = 0.2
    poses = []
    while len(poses) < n_robots:
        x = random.uniform(*x_range)
        y = random.uniform(*y_range)
        theta = random.uniform(*theta_range)
        if len(poses) > 0:
            good_to_add = True
            for pose in poses:
                if np.sqrt((x - pose[0]) ** 2 + (y - pose[1]) ** 2) < closeness_threshold:
                    good_to_add = False
                    break
            if good_to_add:
                poses.append([x, y, theta])
        else:
            poses.append([x, y, theta])
        pass
    # TODO: Remove these static positions.
    poses = [[-0.4, -0.5, 0], [0.23, -0.1123,0], [0.5, 0.1,0], [-0.7, 0.5,0]]
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
          'data_export_config_file': os.path.join(get_package_share_directory('msx'), 'params', 'data_export.yaml'),
          'data_file': LaunchConfiguration('data_file').perform(context),
       }],
       output='both',
    ))

    for robot in robots[:n_robots]:
        controllers.append(Node(
            package='msx',
            executable='sunburst_robot_calc',
            namespace=robot,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_names': robots[:n_robots],
            }],
            remappings=[('topic', f'/{robot}/topic')],
            output='screen',
        ))

    controllers.append(Node(
                package='msx',
                executable='gt_formatting',
                #namespace=robot,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_names': robots[:n_robots],
                }],
                #remappings=[('topic', f'/{robot}/topic')],
                output='screen',
            ))

    controllers.append(Node(
                package='msx',
                executable='error_calc',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_names': robots[:n_robots],
                }],
                #namespace=robot,
                #remappings=[('topic', f'/{robot}/topic')],
                output='screen',
            ))

    controllers.append(Node(
                package='msx',
                executable='visualization',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_names': robots[:n_robots],
                }],
                #namespace=robot,
                #remappings=[('topic', f'/{robot}/topic')],
                output='screen',
            ))

    # for robot in robots[:n_robots]:
    #    controllers.append(Node(
    #        package='msx',
    #        executable='reactive_behaviour',
    #        parameters=[{
    #            'use_sim_time': use_sim_time,
    #            'robot_names': robots[:n_robots],
    #        }],
    #        namespace=robot,
    #        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    #        output='screen',
    #    ))
    
    return controllers


def generate_launch_description():
    args = {
        'behaviour': 'false',
        'world': 'square_room.world',
        # 'world': 'face.world',
        'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'square_room.yaml'),
        # 'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'face.yaml'),
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
            '-x', '0', '-y', '0', '-z', '0.05'
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
