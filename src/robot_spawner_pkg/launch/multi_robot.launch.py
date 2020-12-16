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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def get_ip_name():  # TODO how to make this work for simulation?
    ipa = subprocess.check_output(['ip', 'a']).decode('ascii')
    ips = re.findall(r'inet\s[0-9.]+', ipa)
    ips = [ip[5:] for ip in ips]
    x = [ip.split('.') for ip in ips]
    name = 'T'
    for ip in x:
        if ip[0] != '127':
            name += ip[-1]
    return name


def generate_launch_description():
    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.01},
        {'name': 'robot4', 'x_pose': 0.0, 'y_pose': 1.5, 'z_pose': 0.01},
        {'name': 'robot5', 'x_pose': 0.5, 'y_pose': 0.5, 'z_pose': 0.01},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.01},
        {'name': 'robot3', 'x_pose': 0.5, 'y_pose': 0.0, 'z_pose': 0.01, 'yaw_pose': 1.0},
    ]
    spawner_dir = get_package_share_directory('robot_spawner_pkg')

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(spawner_dir, "single_robot.launch.py")),
                launch_arguments={
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                    'robot_name': robot['name'],
                    'turtlebot_type': TextSubstitution(text='burger')
                }.items()))

    # Define commands for launching the navigation instances

    simulator = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(spawner_dir, 'simulator.launch.py')),
                launch_arguments={
                }.items()
            )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(simulator)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld
