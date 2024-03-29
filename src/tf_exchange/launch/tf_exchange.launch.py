# from launch import LaunchDescription
# from launch_ros.actions import Node
import launch
import launch.actions
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    base_frame = LaunchConfiguration('base_frame')

    # TODO how to add a launch argument that is overloaded by configuration if not provided?
    # namespace = DeclareLaunchArgument(
    #     'namespace',
    #     default_value=LaunchConfiguration('namespace'))

    # robot_name = DeclareLaunchArgument(
    #     'robot_name',
    #     default_value=LaunchConfiguration('robot_name'))

    return launch.LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            name='world_map_tf_static_pub',
            namespace=namespace,
            arguments=['--x', '0', '--y', '0', '--z','0', '--qx', '0', '--qy','0', '--qz','0', '--qw', '1', '--frame-id','world', '--child-frame-id','map'],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        ),
        Node(
            package='tf_exchange',
            executable='local_tf_pub',
            output='screen',
            name='local_tf_pub',
            namespace=namespace,
            parameters=[
                {'robot_name': robot_name},
                {'base_frame': base_frame}
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ('tf_global', '/tf')]
        ),
        # Node(
        #     package='tf_exchange',
        #     executable='local_to_global_tf_pub',
        #     output='screen',
        #     name='local_to_global_tf_pub',
        #     namespace=namespace,
        #     parameters=[
        #         {'robot_name': robot_name}
        #     ],
        # ),
        Node(
            package='tf_exchange',
            executable='global_to_local_tf_pub',
            output='screen',
            name='global_to_local_tf_pub',
            namespace=namespace,
            parameters=[
                {'robot_name': robot_name}
            ],
        ),
    ])
