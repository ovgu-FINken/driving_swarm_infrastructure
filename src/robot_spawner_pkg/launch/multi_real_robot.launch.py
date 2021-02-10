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


def initialize_robots(context, *args, **kwargs):
    """initialize robots"""
    base_frame = LaunchConfiguration('base_frame').perform(context)
    robot_name_list = []
    robot_name_list_dir = os.path.join(
        get_package_share_directory('system_status'), 'local_ips.yaml')

    with open(robot_name_list_dir, 'r') as f:
        data = yaml.safe_load(f)
        robot_name_list = data[0]['local_ips']

    nav_bringup_cmds = []
    for robot_name in robot_name_list:
        robot_name = 'robot'+robot_name
        nav_bringup_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    ThisLaunchFileDir(),
                    "/single_real_robot.launch.py"
                ]),
                launch_arguments={
                    'robot_name': TextSubstitution(text=robot_name),
                    'base_frame': TextSubstitution(text=base_frame),
                    # 'turtlebot_type': TextSubstitution(text='burger')
                }.items()
            )
        )
    return nav_bringup_cmds


def generate_launch_description():
    spawner_dir = get_package_share_directory('robot_spawner_pkg')
    declare_robots_file_cmd = DeclareLaunchArgument(
        'robots_file',
        default_value=os.path.join(spawner_dir, 'params', 'robots.yaml')
    )
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add declarations for launch file arguments
    ld.add_action(declare_robots_file_cmd)
    ld.add_action(declare_base_frame_cmd)

    # The opaque function is neccesary to resolve the context of the launch file and read the LaunchDescription param at runtime
    ld.add_action(OpaqueFunction(function=initialize_robots))

    return ld
