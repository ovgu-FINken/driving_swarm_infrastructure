# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    db_file_path = LaunchConfiguration(
        'db_file_path'
    )
    csv_file_dir = LaunchConfiguration(
        'csv_file_dir', default=""
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'db_file_path',
            default_value=db_file_path,
            description='Absolute path to sqlite3 database file to load'),

        DeclareLaunchArgument(
            'csv_file_dir',
            default_value=csv_file_dir,
            description='Absolute path to the csv files generated by the node.'),

        Node(
            package='experiment_measurement',
            executable='rosbag2df',
            name='rosbag2df',
            parameters=[{'db_file_path': db_file_path},
                        {'csv_file_dir': csv_file_dir}],
            output='screen'
        ),
    ])