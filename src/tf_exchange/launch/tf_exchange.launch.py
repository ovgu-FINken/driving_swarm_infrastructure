# from launch import LaunchDescription
# from launch_ros.actions import Node
import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            name='world_map_tf_static_pub',
            namespace=namespace,
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        ),
        launch_ros.actions.Node(
            package='tf_exchange',
            executable='local_tf_pub',
            output='screen',
            name='local_tf_pub',
            namespace=namespace,
            parameters=[
                {'robot_name': robot_name}
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        ),
        launch_ros.actions.Node(
            package='tf_exchange',
            executable='local_to_global_tf_pub',
            output='screen',
            name='local_to_global_tf_pub',
            namespace=namespace,
            parameters=[
                {'robot_name': robot_name}
            ],
        ),
        launch_ros.actions.Node(
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
