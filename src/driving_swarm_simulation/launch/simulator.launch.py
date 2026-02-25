#!/usr/bin/env python

import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression

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
    with open(f'{dest_folder}/{robot_name}.sdf', 'w') as f:
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
    with open(f'{dest_folder}/{robot_name}.yaml', 'w') as f:
        f.write(content)

# def spawn_robots(context, *args, **kwargs):
#     save_path = '/tmp/'
#     poses_file = LaunchConfiguration('poses_file').perform(context)
#     robot_names_file = LaunchConfiguration('robot_names_file').perform(context)
#     n_robots = LaunchConfiguration('n_robots').perform('context')

#     with open(poses_file, 'r') as stream:
#         poses = yaml.safe_load(stream)
#     with open(robot_names_file, 'r') as stream:
#         robot_names = yaml.safe_load(stream)
    
#     spawn_robot_cmds = []
#     sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + model_name, 'model.sdf')
#     yaml_file = os.path.join(pkg_turtlebot3_gazebo, 'params', 'turtlebot3_' + model_name + '_bridge.yaml'),
#     for name, pose in list(zip(robot_names, poses))[:int(n_robots)]:
#         write_tmp_sdf_file(name, sdf_file, yaml_file, save_path)
#         spawn_robots_cmds.append(
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(
#                     os.path.join(launch_dir, 'multi_spawn_turtlebot3.launch.py'),
#                 ),
#                 launch_arguments={
#                     'x_pose': str(poses[i][0]),
#                     'y_pose': str(poses[i][1]),
#                     'theta': str(poses[i][2]),
#                     'robot_name': name,
#                     'urdf_path': f'{save_path}/{name}.sdf',
#                     'bridge_params': f'{save_path}/{name}.yaml'
#                 }.items()
#             )

#         )
#     return spawn_robot_cmds
     

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
   
    with open(poses_file, 'r') as stream:
        poses = yaml.safe_load(stream)
    with open(robot_names_file, 'r') as stream:
        robot_names = yaml.safe_load(stream)

    '''    
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
    '''
    spawn_turtlebot_cmd_list = []
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    use_sim_time = TextSubstitution(text='True')
    model_name = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    save_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_name,
        'tmp'
    )
    os.makedirs(save_path, exist_ok=True)
    print(f"[INFO] Saving parsed files to: {save_path}", flush=True)
    
    sdf_path = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + model_name, 'model.sdf')
    
    nav2_dir = get_package_share_directory('nav2_bringup')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(bringup_dir, 'rviz', 'custom.rviz'))
    slam = LaunchConfiguration('slam', default='True')
    autostart = 'True'
    params_file = os.path.join(bringup_dir, 'params', 'nav2_params_namespaced.yaml')
    
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    print(f"number of n {int(n_robots)}")
    for i in range(int(n_robots)):
        robot_name = robot_names[i]

        robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true', 
                'frame_prefix': f'{robot_name}'
            }.items()
        )   

        write_tmp_sdf_file(
                robot_name=robot_name,
                sdf_source=sdf_path,
                yaml_source=os.path.join(pkg_turtlebot3_gazebo, 'params', 'turtlebot3_' + model_name + '_bridge.yaml'),
                dest_folder=save_path
            )
        

        start_gazebo_ros_spawner_cmd = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-file', f'{save_path}/{robot_name}.sdf',
                '-x', str(poses[i][0]),
                '-y', str(poses[i][1]),
                '-z', '0.192',
                '-Y', str(poses[i][2])
            ],
            output='screen',
        )
        

        
        start_gazebo_ros_bridge_cmd = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                ['config_file:=',  f'{save_path}/{robot_name}.yaml'], 
                
            ],
            remappings=[
                ('tf', '/tf'),             # Force TF to global
                ('tf_static', '/tf_static') # Force Static TF to global
            ],
            output='screen',
        )
        rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, 'launch', 'rviz_launch.py')),
                condition=IfCondition(LaunchConfiguration('use_rviz', default='True') ),
                launch_arguments={
                    'namespace': robot_name,
                    'use_namespace': 'true',
                    'use_sim_time': 'true',
                    'rviz_config': rviz_config_file
                }.items()
            )

        if(TURTLEBOT3_MODEL!='burger' ):
            start_gazebo_ros_image_bridge_cmd = Node(
                package='ros_gz_image',
                executable='image_bridge',
                arguments=[['/', robot_name, '/camera/image_raw']], 
                remappings=[
                    (['/', robot_name, '/camera/image_raw'], 'camera/image_raw')
                ],
                output='screen',
            )
        


        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_transform_{robot_name}',
            arguments=[
                str(poses[i][0]), str(poses[i][1]), '0', # X, Y, Z
                str(poses[i][2]), '0', '0',  # Yaw, Pitch, Roll
                'world',                           # Parent Frame
                f'{robot_name}/odom'               # Child Frame
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        print("after static")
        

        nodes_in_group = [
            PushRosNamespace(robot_name),
            SetRemap(src='tf', dst='/tf'),
            SetRemap(src='tf_static', dst='/tf_static'),
            robot_state_publisher,
            static_tf,
            start_gazebo_ros_spawner_cmd,
            start_gazebo_ros_bridge_cmd,
            rviz,
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': robot_name,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch',
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': robot_name,
                              'map': LaunchConfiguration('map'),
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items())
    
        ]
        if(TURTLEBOT3_MODEL!='burger' ):
            nodes_in_group.append(start_gazebo_ros_image_bridge_cmd)


        group = GroupAction(nodes_in_group)

        spawn_turtlebot_cmd_list.append(group)

    
    return spawn_turtlebot_cmd_list

def generate_launch_description():

    bringup_dir = get_package_share_directory('driving_swarm_bringup')

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
        default_value=EnvironmentVariable('RUN_TIMEOUT', default_value="0.0")
    )

    declare_init_timeout_cmd = DeclareLaunchArgument(
        'init_timeout',
        default_value=EnvironmentVariable('INIT_TIMEOUT', default_value="0.0")
    )

    declare_reset_timeout_cmd = DeclareLaunchArgument(
        'reset_timeout',
        default_value=EnvironmentVariable('RESET_TIMEOUT', default_value="0.0")
    )

    # from https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/jazzy/turtlebot3_gazebo/launch/turtlebot3_world.launch.py
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    '''
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # world = os.path.join(camera_turtle_bot_pkg, 'worlds', 'bug_world.world')
    sdf_path = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + model_name, 'model.sdf')
    
    model_name = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    save_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_name,
        'tmp'
    )
    os.makedirs(save_path, exist_ok=True)
    print(f"[INFO] Saving parsed files to: {save_path}", flush=True)
    '''

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


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(gzclient_cmd)
    ld.add_action(gzserver_cmd)

    ld.add_action(declare_n_robots_cmd)
    ld.add_action(declare_poses_file_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_run_timeout_cmd)
    ld.add_action(declare_init_timeout_cmd)
    ld.add_action(declare_robot_name_file_cmd)
    ld.add_action(declare_reset_timeout_cmd)
    ld.add_action(OpaqueFunction(function=initialize_robots))

    return ld
