'''import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap

# this is per robot -> it will go into spawn tb3
def parsing_yaml_sdf(robot_name, sdf_source, yaml_source, dest_folder='/tmp'):

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


def generate_launch_description():

    # --- 1. CONFIGURATION ---
    NUM_ROBOTS = 3
    POSES = [[0, 0, 0], [2, -1, 1.57],[1.5, 1, -1.57]]  # X, Y, Yaw for each robot
    NAMESPACE_PREFIX = 'TB3'
    
    # Get model name
    model_name = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Paths
    #camera_turtle_bot_pkg = get_package_share_directory('camera_turtle_bot')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    #launch_dir = os.path.join(camera_turtle_bot_pkg, 'launch')
    
    # world = os.path.join(camera_turtle_bot_pkg, 'worlds', 'bug_world.world')
    sdf_path = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + model_name, 'model.sdf')
    save_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_name,
        'tmp'
    )
    
    os.makedirs(save_path, exist_ok=True)
    print(f"[INFO] Saving parsed files to: {save_path}", flush=True)
    # --- 2. START GAZEBO SIM ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    # i don't use the bug_pose so this is not neccessary
    world_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/bug/pose_static@geometry_msgs/msg/PoseStamped@gz.msgs.Pose'
        ],
        remappings=[
            ('/model/bug/pose_static', '/bug_pose')
        ],
        output='screen'
    )

    spawn_turtlebot_cmd_list = []
    for i in range(NUM_ROBOTS):
        robot_name = f'{NAMESPACE_PREFIX}{i+1}'

        robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true', 
                'frame_prefix': f'{robot_name}'
            }.items()
        )   

        parsing_yaml_sdf(
                robot_name=robot_name,
                sdf_source=sdf_path,
                yaml_source=os.path.join(pkg_turtlebot3_gazebo, 'params', 'turtlebot3_' + model_name + '_bridge.yaml'),
                dest_folder=save_path
            )

        spawn_turtlebot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'multi_spawn_turtlebot3.launch.py'),
                ),
                launch_arguments={
                    'x_pose': str(POSES[i][0]),
                    'y_pose': str(POSES[i][1]),
                    'theta': str(POSES[i][2]),
                    'robot_name': robot_name,
                    'urdf_path': f'{save_path}/{robot_name}.sdf',
                    'bridge_params': f'{save_path}/{robot_name}.yaml'
                }.items()
            )
        
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_transform_{robot_name}',
            arguments=[
                str(POSES[i][0]), str(POSES[i][1]), '0', # X, Y, Z
                str(POSES[i][2]), '0', '0',  # Yaw, Pitch, Roll
                'world',                           # Parent Frame
                f'{robot_name}/odom'               # Child Frame
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        bug_detector = Node(
            package='camera_turtle_bot',
            executable='bug_detector_node',
            output='screen'
        )

        robot_detector = Node(
            package='camera_turtle_bot',
            executable='robot_detector_node',
            output='screen'
        )

        object_tracker = Node(
            package='camera_turtle_bot',
            executable='tracking_node',
            name=f'tracker_{robot_name}',
            output='screen'
        )

        robot_pos_publisher = Node(
            package='camera_turtle_bot',
            executable='robot_pos_publisher_node',
            parameters=[
                {'robot_name': robot_name}
            ],
            output='screen'
        )

        nodes_in_group = [
            PushRosNamespace(robot_name),
            SetRemap(src='tf', dst='/tf'),
            SetRemap(src='tf_static', dst='/tf_static'),
            robot_state_publisher,
            static_tf,
            spawn_turtlebot,
            robot_pos_publisher
        ]
        if model_name != 'burger':
            print(f"[INFO] Spawning {robot_name} with Bug Detector", flush=True)
            nodes_in_group.append(bug_detector)
            nodes_in_group.append(robot_detector)
            nodes_in_group.append(object_tracker)

        group = GroupAction(nodes_in_group)

        spawn_turtlebot_cmd_list.append(group)

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(world_bridge)
 
    
    for cmd in spawn_turtlebot_cmd_list:
        ld.add_action(cmd)

    return ld
'''  
