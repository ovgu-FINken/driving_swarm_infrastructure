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

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    spawner_dir = get_package_share_directory('robot_spawner_pkg')
    tf_exchange_dir = get_package_share_directory('tf_exchange')

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.01},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.01}
    ]

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    declare_base_frame = DeclareLaunchArgument('base_frame', default_value='base_link')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        'robot1_params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'nav2_multirobot_params_1.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        'robot2_params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'nav2_multirobot_params_2.yaml'),
        description='Full path to the ROS2 parameters file to use for robot2 launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='false',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(spawner_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(spawner_dir, 'launch',
                                                           'spawn_tb3.launch.py')),
                launch_arguments={
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                    'robot_name': robot['name'],
                    'turtlebot_type': TextSubstitution(text='burger')
                }.items()))

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(robot['name'] + '_params_file')

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_launch_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'namespace': TextSubstitution(text=robot['name']),
                    'use_namespace': 'True',
                    'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                           'launch',
                                                           'tb3_simulation_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'default_bt_xml_filename': default_bt_xml_filename,
                                  'autostart': autostart,
                                  'use_rviz': 'False',
                                  'use_simulator': 'False',
                                  'headless': 'False',
                                  'use_robot_state_pub': use_robot_state_pub}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tf_exchange_dir, 'launch', 'tf_exchange.launch.py')),
                launch_arguments={
                    'namespace': TextSubstitution(text=robot['name']),
                    'robot_name': robot['name'],
                    'base_frame': LaunchConfiguration('base_frame')
                    # 'params_file': params_file
                }.items()
            ),


            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' map yaml: ', map_yaml_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' behavior tree xml: ', default_bt_xml_filename]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])

        nav_instances_cmds.append(group)

    simulator = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(spawner_dir, 'launch', 'simulator.launch.py')),
                launch_arguments={
                }.items()
            )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_base_frame)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(simulator)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
