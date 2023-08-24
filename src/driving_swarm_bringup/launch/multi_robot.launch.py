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

This is from an example on how to create a launch file 
for spawning multiple robots into Gazebo and launch 
multiple instances of the navigation stack, each controlling
one robot.
"""

import os
import yaml


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch_ros.actions import Node



def initialize_robots(context, *args, **kwargs):
    """initialize robots"""
    # Names and poses of the robots
    # Ignore this comment
    bringup_dir = get_package_share_directory('driving_swarm_bringup')
    n_robots = LaunchConfiguration('n_robots').perform(context)
    run_timeout = LaunchConfiguration('run_timeout')
    init_timeout = LaunchConfiguration('init_timeout')
    poses_file = LaunchConfiguration('poses_file').perform(context)
    robot_names_file = LaunchConfiguration('robot_names_file').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    single_robot_launch_file = LaunchConfiguration(
        'single_robot_launch_file', 
        default=os.path.join(bringup_dir, 'launch', "single_robot.launch.py")
        ).perform(context)
    with open(poses_file, 'r') as stream:
        poses = yaml.safe_load(stream)
    with open(robot_names_file, 'r') as stream:
        robot_names = yaml.safe_load(stream)
        
    command_node = Node(package="experiment_supervisor",
                        executable="command_node",
                        output="screen",
                        parameters=[{
                           'use_sim_time': LaunchConfiguration('use_sim_time'),
                           'run_timeout': run_timeout,
                           'init_timeout': init_timeout,
                           'reset_timeout': LaunchConfiguration('reset_timeout'),
                           'robot_names': robot_names[:int(n_robots)],
                           }])

    exit_event_handler = RegisterEventHandler(event_handler=OnProcessExit(
            target_action=command_node,
            on_exit=EmitEvent(event=Shutdown(reason="command node exited"))
        )
    )

    spawn_robots_cmds = [
        command_node, exit_event_handler
    ]
    for name, pose in list(zip(robot_names, poses))[:int(n_robots)]:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    single_robot_launch_file
                ),
                launch_arguments={
                    'x_pose': TextSubstitution(text=str(pose[0])),
                    'y_pose': TextSubstitution(text=str(pose[1])),
                    'z_pose': TextSubstitution(text="0.0"),
                    'yaw_pose': TextSubstitution(text=str(pose[2])),
                    'robot_name': name,
                    'base_frame': TextSubstitution(text=base_frame),
                    'turtlebot_type': TextSubstitution(text='burger')
                }.items()
            )
        )
    return spawn_robots_cmds


def generate_launch_description():
    bringup_dir = get_package_share_directory('driving_swarm_bringup')
    exp_measurement_dir = get_package_share_directory('experiment_measurement')

    declare_n_robots_cmd = DeclareLaunchArgument(
        'n_robots',
        default_value=EnvironmentVariable('N_ROBOTS', default_value='2')
    )

    declare_poses_file_cmd = DeclareLaunchArgument(
        'poses_file',
        default_value=os.path.join(bringup_dir, 'params', 'tb3_world_poses.yaml')
    )
    
    declare_robot_name_file_cmd = DeclareLaunchArgument(
        'robot_names_file',
        default_value=os.path.join(bringup_dir, 'params', 'robot_names_sim.yaml')
    )

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true'
    )
    
    declare_run_timeout_cmd = DeclareLaunchArgument(
        'run_timeout',
        default_value='0.0'
    )

    declare_init_timeout_cmd = DeclareLaunchArgument(
        'init_timeout',
        default_value='0.0'
    )

    declare_reset_timeout_cmd = DeclareLaunchArgument(
        'reset_timeout',
        default_value=EnvironmentVariable('RESET_TIMEOUT', default_value="0.0")
    )

    # Define commands for launching the navigation instances
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'simulator.launch.py')),
        launch_arguments={
        }.items()
    )

    rosbag_recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exp_measurement_dir,
                         'launch', 'rosbag_recording.launch.py')),
        launch_arguments={
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add declarations for launch file arguments
    ld.add_action(declare_n_robots_cmd)
    ld.add_action(declare_poses_file_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_run_timeout_cmd)
    ld.add_action(declare_init_timeout_cmd)
    ld.add_action(declare_robot_name_file_cmd)
    ld.add_action(declare_reset_timeout_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(simulator)

    # Add the actions to start rosbag recording
    ld.add_action(rosbag_recording)

    # The opaque function is neccesary to resolve the context
    # of the launch file and read the LaunchDescription param at runtime
    ld.add_action(OpaqueFunction(function=initialize_robots))

    return ld
