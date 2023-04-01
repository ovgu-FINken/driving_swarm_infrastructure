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


import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, EmitEvent)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit


def get_robot_config(robots_file):
    robots = []
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    return robots


def initialize_robots(context, *args, **kwargs):
    """initialize robots"""
    # Names and poses of the robots
    bringup_dir = get_package_share_directory('driving_swarm_bringup')
    n_robots = LaunchConfiguration('n_robots').perform(context)
    run_timeout = LaunchConfiguration('run_timeout')
    init_timeout = LaunchConfiguration('init_timeout')
    robots_file = LaunchConfiguration('robots_file').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    single_robot_launch_file = LaunchConfiguration(
        'single_robot_launch_file', 
        default=os.path.join(
            bringup_dir, 
            'launch', 
            'single_real_robot.launch.py'
            )
        ).perform(context)
    robots = get_robot_config(robots_file)

    command_node = Node(package="experiment_supervisor",
                        executable="command_node",
                        output="screen",
                        parameters=[{
                           'run_timeout': run_timeout,
                           'init_timeout': init_timeout,
                           'robot_names': [robot["name"] for robot in robots[:int(n_robots)]],
                           }])

    exit_event_handler = RegisterEventHandler(event_handler=OnProcessExit(
            target_action=command_node,
            on_exit=EmitEvent(event=Shutdown(reason="command node exited"))
        )
    )

    nav_bringup_cmds = [
        command_node, exit_event_handler
    ]
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
    bringup_dir = get_package_share_directory('driving_swarm_bringup')
    exp_measurement_dir = get_package_share_directory('experiment_measurement')

    declare_n_robots_cmd = DeclareLaunchArgument(
        'n_robots',
        default_value='2'
    )
    declare_robots_file_cmd = DeclareLaunchArgument(
        'robots_file',
        default_value=os.path.join(bringup_dir, 'params', 'swarmlab_two_walls_real.yaml')
    )
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint'
    )

    declare_run_timeout_cmd = DeclareLaunchArgument(
        'run_timeout',
        default_value='0.0'
    )

    declare_init_timeout_cmd = DeclareLaunchArgument(
        'init_timeout',
        default_value='0.0'
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
    ld.add_action(declare_run_timeout_cmd)
    ld.add_action(declare_init_timeout_cmd)

    # Add the actions to start rosbag recording
    ld.add_action(rosbag_recording)

    # The opaque function is neccesary to resolve the context of the launch file and read the LaunchDescription param at runtime
    ld.add_action(OpaqueFunction(function=initialize_robots))

    return ld
