from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions



def generate_launch_description():
    return LaunchDescription([
        # Deklariere die Launch-Argumente
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('robot_namespace', default_value='robot1'),
        DeclareLaunchArgument('turtlebot_type', default_value='burger'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.1'),       
        
        # ROS-GZ Bridge für Spawn Service
        launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
                '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity',
            ],
            output='screen'
        ),

        # Spawne den Roboter
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
    ])