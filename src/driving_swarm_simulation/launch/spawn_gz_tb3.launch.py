'''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os



def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')


    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(set_env_vars_resources)

    return ld


    
    # return LaunchDescription([
    #     # Deklariere die Launch-Argumente
    #     DeclareLaunchArgument('robot_name', default_value='robot1'),
    #     DeclareLaunchArgument('robot_namespace', default_value='robot1'),
    #     DeclareLaunchArgument('turtlebot_type', default_value='burger'),
    #     DeclareLaunchArgument('x_pose', default_value='0.0'),
    #     DeclareLaunchArgument('y_pose', default_value='0.0'),
    #     DeclareLaunchArgument('z_pose', default_value='0.1'),       
        
        
        
 
    #     # ROS-GZ Bridge für Spawn Service
    #     launch_ros.actions.Node(
    #         package='ros_gz_bridge',
    #         executable='parameter_bridge',
    #         arguments=[
    #             '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
    #             '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity',
    #         ],
    #         output='screen'
    #     ),

    #     # Spawne den Roboter

    #     launch_ros.actions.Node(
    #         package='driving_swarm_simulation',
    #         executable='nav2_gz_spawner',
    #         output='screen',
    #         arguments=[
    #             '--robot_name', LaunchConfiguration('robot_name'),
    #             '--robot_namespace', LaunchConfiguration('robot_namespace'),
    #             '--turtlebot_type', LaunchConfiguration('turtlebot_type'),
    #             '-x', LaunchConfiguration('x_pose'),
    #             '-y', LaunchConfiguration('y_pose'),
    #             '-z', LaunchConfiguration('z_pose')
    #         ]
    #     ),
    #      launch_ros.actions.Node(
    #         package='ros_gz_bridge',
    #         executable='parameter_bridge',
    #         name='sensor_bridge',
    #         arguments=[
    #             '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    #             '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    #             '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    #         ],
    #         remappings=[
    #             ('/scan', '/scan'),
    #             ('/odometry', '/odom'),
    #             ('/cmd_vel', '/cmd_vel'),
    #         ],
    #         output='screen'
    #     ),
        
    

        # launch_ros.actions.Node(
        #     package='driving_swarm_simulation',
        #     executable='nav2_gz_spawner',
        #     output='screen',
        #     arguments=[
        #         '--robot_name', LaunchConfiguration('robot_name'),
        #         '--robot_namespace', LaunchConfiguration('robot_namespace'),
        #         '--turtlebot_type', LaunchConfiguration('turtlebot_type'),
        #         '-x', LaunchConfiguration('x_pose'),
        #         '-y', LaunchConfiguration('y_pose'),
        #         '-z', LaunchConfiguration('z_pose')
        #     ]
        # ),
    #])

'''