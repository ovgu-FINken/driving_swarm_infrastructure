"""
This launch file starts the node collecting the ips of all brought-up (real!) robots, and writing it into a file.
"""

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="system_status",
                executable="ip_logger",
                output="screen",
                name="ip_logger",
            )
        ]
    )
