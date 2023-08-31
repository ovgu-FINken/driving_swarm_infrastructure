#!/usr/bin/bash

source ~/.rosrc

COMMAND="ros2 launch ccr ccr_sim.launch.py use_rviz:=false"
INIT_TIMEOUT=100
RUN_TIMEOUT=300

ROS_SIMULATOR="gzserver"

# Loop from 1 to 10
for ((N_ROBOTS = 1; N_ROBOTS <= 2; N_ROBOTS++)); do
    echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
    # Run the command with the current N_ROBOTS value
    $COMMAND
done