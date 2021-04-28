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
    controllers = []

    n_robots = LaunchConfiguration("n_robots").perform(context)
    robots_file = LaunchConfiguration("robots_file").perform(context)
    use_sim_time = TextSubstitution(text="false")
    with open(robots_file, "r") as stream:
        robots = yaml.safe_load(stream)

    for robot in robots[: int(n_robots)]:
        controllers.append(
            Node(
                package="goal_provider",
                executable="simple_goal",
                namespace=robot["name"],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "x": robot["goals"]["x"],
                        "y": robot["goals"]["y"],
                        "theta": robot["goals"]["theta"],
                    }
                ],
                output="screen",
            )
        )
        controllers.append(Node(
           package='system_status',
           executable='scan_delay',
           namespace=robot['name'],
           parameters=[{
              'use_sim_time': use_sim_time,
           }],
           output='screen',
        ))
        controllers.append(
            Node(
                package="trajectory_generator",
                executable="direct_planner",
                namespace=robot["name"],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "vehicle_model": 1,
                        "turn_radius": 0.2,
                        "step_size": 0.06,
                    }
                ],
                output="screen",
                # arguments=[],
            )
        )
        controllers.append(
            Node(
                package="trajectory_follower",
                executable="trajectory_follower",
                namespace=robot["name"],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "dt": 2.0,
                        "w1": 0.5,
                        "w2": 0.5,
                        "fail_radius": 0.3
                    }
                ],
                output="screen",
                # arguments=[],
            )
        )

    return controllers


def generate_launch_description():
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("driving_swarm_bringup"),
                "launch",
                "multi_real_robot_fix_pos.launch.py",
            )
        ),
        launch_arguments={
            "behaviour": "false",
            "map": os.path.join(
                get_package_share_directory("driving_swarm_bringup"),
                "maps",
                "swarmlab_two_walls.yaml",
            ),
            "robots_file": os.path.join(
                get_package_share_directory("driving_swarm_bringup"),
                "params",
                "swarmlab_two_walls_real.yaml",
            ),
            "rosbag_topics_file": os.path.join(
                get_package_share_directory("trajectory_follower"),
                "params",
                "rosbag_topics.yaml",
            ),
            "qos_override_file": os.path.join(
                get_package_share_directory("experiment_measurement"),
                "params",
                "qos_override.yaml",
            ),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld
