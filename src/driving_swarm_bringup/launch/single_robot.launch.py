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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch.actions
import launch_ros.actions


def generate_launch_description():
    tf_exchange_dir = get_package_share_directory('tf_exchange')
    nav2_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('driving_swarm_bringup')
    slam = LaunchConfiguration('slam')

    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
        nav2_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')
    
    declare_robot_name = DeclareLaunchArgument('robot_name', default_value='robot1')
    declare_base_frame = DeclareLaunchArgument('base_frame', default_value='base_link')
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value=launch.substitutions.TextSubstitution(text='0.0'))
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value=launch.substitutions.TextSubstitution(text='0.0'))
    declare_z_pose = DeclareLaunchArgument('z_pose', default_value=launch.substitutions.TextSubstitution(text='0.0'))
    declare_yaw_pose = DeclareLaunchArgument('yaw_pose', default_value=launch.substitutions.TextSubstitution(text='0.0'))
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(bringup_dir, 'rviz', 'custom.rviz'))

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='Full path to the RVIZ config file to use.')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_behaviour_cmd = DeclareLaunchArgument(
        'behaviour',
        default_value='False',
        description='Whether run the default behaviour.')
    
    
    
    tf_exchange = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tf_exchange_dir, 'launch', 'tf_exchange.launch.py')),
                launch_arguments={
                    'namespace': LaunchConfiguration('robot_name'),
                    'robot_name': LaunchConfiguration('robot_name'),
                    'base_frame': LaunchConfiguration('base_frame')
                    #'params_file': [LaunchConfiguration('robot_name'), launch.substitutions.TextSubstitution(text='_params_file')]
                }.items()
            )
    
    spawner = launch_ros.actions.Node(
            package='driving_swarm_bringup',
            executable='nav2_gazebo_spawner',
            output='screen',
            arguments=[
                '--robot_name', LaunchConfiguration('robot_name'),
                '--robot_namespace', LaunchConfiguration('robot_name'),
                '--turtlebot_type', launch.substitutions.EnvironmentVariable('TURTLEBOT3_MODEL'),
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-z', LaunchConfiguration('z_pose'),
                '-yaw', LaunchConfiguration('yaw_pose'),
                ])

    rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, 'launch', 'rviz_launch.py')),
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                launch_arguments={
                    'namespace': LaunchConfiguration('robot_name'),
                    'use_namespace': 'true',
                    'use_sim_time': 'true',
                    'rviz_config': rviz_config_file
                }.items()
            )

    namespace = LaunchConfiguration('robot_name')
    use_sim_time = TextSubstitution(text='True')
    autostart = 'True'
    params_file = os.path.join(bringup_dir, 'params', 'nav2_params_namespaced.yaml')
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_burger.urdf')


    bringup_cmd_group = GroupAction([
        launch_ros.actions.PushRosNamespace(
            namespace=namespace),
            
        launch_ros.actions.Node(package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            #namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch',
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': LaunchConfiguration('map'),
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items())
    ])
    
    drive = launch_ros.actions.Node(
        executable='turtlebot3_drive',
        condition=IfCondition(LaunchConfiguration('behaviour')),
        package='turtlebot3_gazebo',
        namespace=LaunchConfiguration('robot_name'),
    )
    
    
    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_base_frame)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)
    ld.add_action(declare_yaw_pose)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_behaviour_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(spawner)
    ld.add_action(rviz)
    ld.add_action(bringup_cmd_group)
    ld.add_action(tf_exchange)
    ld.add_action(drive)
    return ld 