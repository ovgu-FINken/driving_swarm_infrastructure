#!/usr/bin/bash
# Executes on every startup

source ~/.bashrc
source /opt/ros/foxy/setup.bash
source /home/turtle/driving_swarm_infrastructure/install/setup.bash
source /home/turtle/turtlebot3/install/setup.bash
export TURTLEBOT3_MODEL=burger

# at least one foreground process needs to be running
ros2 launch system_status robot_system_bringup.launch.py
