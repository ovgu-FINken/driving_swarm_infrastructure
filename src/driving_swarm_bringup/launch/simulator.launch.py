#!/usr/bin/env python

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():

    bringup_dir = get_package_share_directory("nav2_bringup")

    # Simulation settings
    world = LaunchConfiguration("world")
    simulator = LaunchConfiguration("simulator")

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(bringup_dir, "worlds", "world_only.model"),
        description="Full path to world file to load",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="gazebo",
        description="The simulator to use (gazebo or gzserver, i.e. with or without GUI)",
    )

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            simulator,
            "--verbose",
            "-s",
            "libgazebo_ros_factory.so",
            "-s",
            "libgazebo_ros_init.so",
            world,
        ],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    return ld
