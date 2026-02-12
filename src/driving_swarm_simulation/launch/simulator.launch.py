#!/usr/bin/env python

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, AppendEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable


def write_tmp_sdf_file(robot_name, sdf_source, yaml_source, dest_folder='/tmp'):

    with open(sdf_source, 'r') as f:
        content = f.read()

    unique_topic_root = f'/{robot_name}'
    unique_frame_root = f'{robot_name}'

    # Cmd_vel
    content = content.replace('<topic>cmd_vel</topic>', f'<topic>{unique_topic_root}/cmd_vel</topic>')
    
    # Odom
    content = content.replace('<topic>odom</topic>', f'<topic>{unique_topic_root}/odom</topic>')
    content = content.replace('<odom_topic>odom</odom_topic>', f'<odom_topic>{unique_topic_root}/odom</odom_topic>')

    # Scan
    content = content.replace('<topic>scan</topic>', f'<topic>{unique_topic_root}/scan</topic>')
    
    # IMU
    content = content.replace('<topic>imu</topic>', f'<topic>{unique_topic_root}/imu</topic>')
    
    # Camera
    #content = content.replace('<topic>camera/image_raw</topic>', f'<topic>{unique_topic_root}/camera/image_raw</topic>')
    
    # TF Topic (Global -> Unique)
    content = content.replace('<tf_topic>/tf</tf_topic>', f'<tf_topic>{unique_topic_root}/tf</tf_topic>')


    # Odom Frame
    content = content.replace('<frame_id>odom</frame_id>', f'<frame_id>{unique_frame_root}/odom</frame_id>')
    
    # Base Footprint (Handling the specific snippet you showed me: /robot/base_footprint)
    content = content.replace('<child_frame_id>/robot/base_footprint</child_frame_id>', f'<child_frame_id>{unique_frame_root}/base_footprint</child_frame_id>')
    
    # Backup for standard files
    content = content.replace('<child_frame_id>base_footprint</child_frame_id>', f'<child_frame_id>{unique_frame_root}/base_footprint</child_frame_id>')

    # Lidar Frame (Handling your snippet: gz_frame_id)
    content = content.replace('<gz_frame_id>base_scan</gz_frame_id>', f'<gz_frame_id>{unique_frame_root}/base_scan</gz_frame_id>')
    
    # Backup for older files
    content = content.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{unique_frame_root}/base_scan</frame_name>')

    # Write
    sdf_file = f'{dest_folder}/{robot_name}.sdf'
    with open(sdf_file, 'w') as f:
        f.write(content)


    with open(yaml_source, 'r') as f:
        content = f.read()

    
    # We must match the Gazebo names we just created in the SDF above.

    # Cmd_vel
    content = content.replace('gz_topic_name: "cmd_vel"', f'gz_topic_name: "{unique_topic_root}/cmd_vel"')
    
    # Odom
    content = content.replace('gz_topic_name: "odom"', f'gz_topic_name: "{unique_topic_root}/odometry"')
    # Check for "odometry" key just in case
    content = content.replace('gz_topic_name: "odometry"', f'gz_topic_name: "{unique_topic_root}/odometry"')

    # Scan
    content = content.replace('gz_topic_name: "scan"', f'gz_topic_name: "{unique_topic_root}/scan"')
    
    # IMU
    content = content.replace('gz_topic_name: "imu"', f'gz_topic_name: "{unique_topic_root}/imu"')
    
    # TF
    content = content.replace('gz_topic_name: "tf"', f'gz_topic_name: "{unique_topic_root}/tf"')
    
    # Camera
    # content = content.replace('gz_topic_name: "camera/image_raw"', f'gz_topic_name: "{unique_topic_root}/camera/image_raw"')
    # content = content.replace('gz_topic_name: "camera/camera_info"', f'gz_topic_name: "{unique_topic_root}/camera/camera_info"')

    # Write
    yaml_file = f'{dest_folder}/{robot_name}.yaml'
    with open(yaml_file, 'w') as f:
        f.write(content)
    return sdf_file, yaml_file 

def spawn_tb3_cmds(name, pose, robot_sdf, robot_yaml):
    # spawn robot in simulation
    # start robot state publisher
    cmds = []
    return cmds

def spawn_robots(context, *args, **kwargs):
    save_path = '/tmp/'
    poses_file = LaunchConfiguration('poses_file').perform(context)
    robot_names_file = LaunchConfiguration('robot_names_file').perform(context)
    n_robots = LaunchConfiguration('n_robots').perform('context')

    with open(poses_file, 'r') as stream:
        poses = yaml.safe_load(stream)
    with open(robot_names_file, 'r') as stream:
        robot_names = yaml.safe_load(stream)
    
    spawn_robot_cmds = []
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + model_name, 'model.sdf')
    yaml_file = os.path.join(pkg_turtlebot3_gazebo, 'params', 'turtlebot3_' + model_name + '_bridge.yaml'),
    if int(n_robots) < 1:
        return spawn_robot_cmds
    for name, pose in list(zip(robot_names, poses))[:int(n_robots)]:
        robot_sdf, robot_yaml = write_tmp_sdf_file(name, sdf_file, yaml_file, save_path)
        spawn_robot_cmds += spawn_tb3_cmds(name, pose, robot_sdf, robot_yaml)
    return spawn_robot_cmds
     

def generate_launch_description():
    # from https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/jazzy/turtlebot3_gazebo/launch/turtlebot3_world.launch.py
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Simulation settings
    world = LaunchConfiguration('world')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join('turtlebot3_gazebo', 'worlds', 'turtlebot3_world.world'),
        description='Full path to world file to load')


    # from https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/jazzy/turtlebot3_gazebo/launch/turtlebot3_world.launch.py
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(gzclient_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(set_env_vars_resources)

    return ld
