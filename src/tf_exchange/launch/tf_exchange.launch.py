# from launch import LaunchDescription
# from launch_ros.actions import Node
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import subprocess
import re


def get_ip():
    ipa = subprocess.check_output(['ip', 'a']).decode('ascii')
    ips = re.findall(r'inet\s[0-9.]+', ipa)
    ips = [ip[5:] for ip in ips]
    # TODO get only the last 3 digits of the ip
    # ip = ", ".join(ips)
    x = [ip.split(".") for ip in ips]  # PRINTOUT: [['.'], ['.']]

    print(x)
    name = "T"
    for ip in x:
        if ip[0] != "127":
            name += ip[-1]

    return name


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'ns',
            default_value=[
                get_ip()
            ],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            # name=[launch.substitutions.LaunchConfiguration('ns'), 'world_map_tf_static_pub'],
            name='world_map_tf_static_pub',
            namespace=launch.substitutions.LaunchConfiguration('ns'),
            arguments=["0", "0", "0", "0", "0", "0", "1", "world", "map"],
        ),
    ])
