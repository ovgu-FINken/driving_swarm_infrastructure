#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.substitutions import LaunchConfiguration, TextSubstitution

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def initialize_world_and_robot(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time',
                                       default='true').perform(context)
    # complete absolute path to the .world file NOT only the file name!
    # note that this world needs a robot you can move around
    model_file = LaunchConfiguration('model_file',
                                     default='turtlebot3_worlds/' +
                                     TURTLEBOT3_MODEL +
                                     '.model').perform(context)

    world = LaunchConfiguration(
        'world',
        default=os.path.join(
            get_package_share_directory('driving_swarm_bringup'), 'worlds',
            model_file))
    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')

    cmd = [
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ]
    return cmd


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()
    # The opaque function is neccesary to resolve the context of the launch file and read the LaunchDescription param at runtime
    ld.add_action(OpaqueFunction(function=initialize_world_and_robot))
    return ld
