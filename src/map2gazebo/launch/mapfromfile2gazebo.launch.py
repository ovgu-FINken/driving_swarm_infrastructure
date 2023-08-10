"""Converts the map files generated from the turtlebot to dae file format."""

import launch
import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch the map server and calls map2gazebo."""
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('map_file', default_value='icra2021_map.yaml'),
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[
                {
                    'yaml_filename':
                        os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps', launch.substitutions.LaunchConfiguration('map_file').perform()),
                }
            ],
            remappings=[
                ('map', 'robot1/map')
            ]
        ),
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[
                # {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [
                    launch.substitutions.ThisLaunchFileDir(),
                    '/map2gazebo.launch.py'
                ]
            )
        )
    ])
