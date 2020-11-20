# from launch import LaunchDescription
# from launch_ros.actions import Node
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import subprocess
import re


def get_ip_name():
    ipa = subprocess.check_output(['ip', 'a']).decode('ascii')
    ips = re.findall(r'inet\s[0-9.]+', ipa)
    ips = [ip[5:] for ip in ips]
    x = [ip.split('.') for ip in ips]
    name = 'T'
    for ip in x:
        if ip[0] != '127':
            name += ip[-1]
    return name


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'robot_name',
            default_value=[
                get_ip_name()
            ],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            name='world_map_tf_static_pub',
            namespace=launch.substitutions.LaunchConfiguration('robot_name'),
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        ),
        # launch_ros.actions.Node(
        #     package='tf_exchange',
        #     executable='local_to_global_tf_pub',
        #     output='screen',
        #     name='local_to_global_tf_pub',
        #     namespace=launch.substitutions.LaunchConfiguration('robot_name'),
        #     parameters=[
        #         {'robot_name': launch.substitutions.LaunchConfiguration('robot_name')}],
        #     remappings=[("tf", "/tf"), ("tf_static", "/tf_static")]
        # ),
    ])
