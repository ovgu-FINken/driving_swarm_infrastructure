
import os
import subprocess
import re
import yaml
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def get_robot_config(robots_file):
    robots = []
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    return robots

def get_rosbag_config(rosbag_topics_file):
    rosbag_config = []
    with open(rosbag_topics_file, 'r') as stream:
        rosbag_config = yaml.safe_load(stream)
    return rosbag_config

def start_rosbag(context, *args, **kwargs):
    rosbag_topics_file = LaunchConfiguration("rosbag_topics_file").perform(context)
    start_rosbag_cmd = []
    #TODO: include -a for recording all topics

    if rosbag_topics_file != "NONE":
        robots_file = LaunchConfiguration('robot_names_file').perform(context)
        n_robots = LaunchConfiguration('n_robots').perform(context)
        qos_override_file = LaunchConfiguration('qos_override_file', default=None).perform(context)
        ts = datetime.utcnow().strftime('%Y-%m-%d_%H:%M:%S')
        default_rosbag_output_dir = 'rosbag_' + str(n_robots) + ' robots_' + ts
        rosbag_output_dir = LaunchConfiguration('rosbag_output_dir', default=default_rosbag_output_dir).perform(context)

        rosbag_topics = []
        robots = get_robot_config(robots_file)
        rosbag_config = get_rosbag_config(rosbag_topics_file)

        for topic in rosbag_config:
            if topic.startswith('/'):
                rosbag_topics.append(topic.strip())
            else:
                for robot in robots[:int(n_robots)]:
                    rosbag_topics.append('/' + str(robot) + '/' + topic.strip())

            # TODO: 
            # * create an ad-hoc qos override file, including the scan topic for each robot namespace
            # * pass it as qos_override_file


        # cmd syntax: https://github.com/ros2/launch/issues/263
        # https://github.com/ros2/rosbag2
        if not qos_override_file is None:
            start_rosbag_cmd.append(
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record'] + rosbag_topics + ['--output', str(rosbag_output_dir)] + ['--qos-profile-overrides-path', str(qos_override_file)],
                    output='screen',
                )
            )
        else:
            start_rosbag_cmd.append(
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record'] + rosbag_topics + ['--output', str(rosbag_output_dir)],
                    output='screen',
                )
            )
    return start_rosbag_cmd

def generate_launch_description():
    declare_rosbag_file_cmd = DeclareLaunchArgument(
        'rosbag_topics_file',
        default_value='NONE'
    )

    declare_rosbag_flag_cmd = DeclareLaunchArgument(
        'use_rosbag',
        default_value='False'
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_rosbag_file_cmd)
    ld.add_action(declare_rosbag_flag_cmd)
    # The opaque function is neccesary to resolve the context of the launch file and read the LaunchDescription param at runtime
    ld.add_action(OpaqueFunction(function=start_rosbag, condition=IfCondition(LaunchConfiguration('use_rosbag'))))

    return ld
