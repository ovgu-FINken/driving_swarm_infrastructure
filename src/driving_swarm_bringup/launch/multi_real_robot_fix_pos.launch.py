# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
For spawing multiple robots in Gazebo.

This is from an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os
import subprocess
import re
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, ThisLaunchFileDir

def get_robot_config(robots_file):
    robots = []
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    return robots


def initialize_robots(context, *args, **kwargs):
    """initialize robots"""
    # Names and poses of the robots
    spawner_dir = get_package_share_directory('driving_swarm_bringup')
    n_robots = LaunchConfiguration('n_robots').perform(context)
    robots_file = LaunchConfiguration('robots_file').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    single_robot_launch_file = LaunchConfiguration(
        'single_robot_launch_file', 
        default=os.path.join(
            spawner_dir, 
            'launch', 
            'single_real_robot.launch.py'
            )
        ).perform(context)
    robots = get_robot_config(robots_file)

    nav_bringup_cmds = []
    i = 1
    for robot in robots:
        if i <= int(n_robots):
            nav_bringup_cmds.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        single_robot_launch_file
                    ),
                    launch_arguments={
                        'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                        'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                        'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                        'yaw_pose': TextSubstitution(text=str(robot['yaw_pose'])),
                        'robot_name': TextSubstitution(text=str(robot['name'])),
                        'base_frame': TextSubstitution(text=base_frame),
                        'turtlebot_type': TextSubstitution(text='burger'),
                        'n_robots': n_robots
                    }.items()
                )
            )
            i += 1
    return nav_bringup_cmds


def generate_launch_description():
    spawner_dir = get_package_share_directory('driving_swarm_bringup')
    exp_measurement_dir = get_package_share_directory('experiment_measurement')

    declare_n_robots_cmd = DeclareLaunchArgument(
        'n_robots',
        default_value='2'
    )
    declare_robots_file_cmd = DeclareLaunchArgument(
        'robots_file',
        default_value=os.path.join(spawner_dir, 'params', 'swarmlab_two_walls_real.yaml')
    )
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint'
    )
    declare_rosbag_file_cmd = DeclareLaunchArgument(
        'rosbag_topics_file',
        default_value='NONE'
    )

    rosbag_recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exp_measurement_dir, 'launch', 'rosbag_recording.launch.py')),
        launch_arguments={
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add declarations for launch file arguments
    ld.add_action(declare_n_robots_cmd)
    ld.add_action(declare_robots_file_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_rosbag_file_cmd)

    # Add the actions to start rosbag recording
    ld.add_action(rosbag_recording)

    # The opaque function is neccesary to resolve the context of the launch file and read the LaunchDescription param at runtime
    ld.add_action(OpaqueFunction(function=initialize_robots))

    return ld
