#!/usr/bin/bash

source ~/.rosrc

COMMAND="ros2 launch ccr ccr_sim.launch.py use_rviz:=false use_rosbag:=true"
export RUN_TIMEOUT="900.0"

export ROS_SIMULATOR="gzserver"

# Loop from 1 to 10
for ((RUN = 1; RUN <= 1; RUN++)); do
	for ((N_ROBOTS = 1; N_ROBOTS <= 10; N_ROBOTS++)); do
	    echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
	    # Run the command with the current N_ROBOTS value
	    $COMMAND n_robots:=$N_ROBOTS
	done
done
