"""
This launch file starts the node collecting the ips of all brought-up (real!) robots, and writing it into a file.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import re
import subprocess
from system_status import utils

def generate_launch_description():

    ip = launch_ros.actions.Node(
        package='system_status',
        executable='system_status',
        output='both',  # "screen", "log" or "both"
        name='system_status',
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/robot.launch.py']
        ),
    )

    watchdog = launch_ros.actions.Node(
        package='robot_spawner_pkg',
        executable='watchdog',
        namespace=utils.get_robot_name('robot'),
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(ip)
    ld.add_action(robot)
    # ld.add_action(watchdog)
    return ld
