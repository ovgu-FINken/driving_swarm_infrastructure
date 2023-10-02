#!/usr/bin/bash

source ~/.rosrc

COMMAND="ros2 launch ccr ccr_sim_1m.launch.py use_rviz:=false use_rosbag:=true"
export RUN_TIMEOUT="600.0"
export INIT_TIMEOUT="300.0"

export ROS_SIMULATOR="gzserver"
export CCR_VERSION="global_planner_baseline"
export CCR_PRIORITIES="index"

for ((RUN = 1; RUN <= $N_RUNS; RUN++)); do
	for ((N_ROBOTS = 1; N_ROBOTS <= 8; N_ROBOTS++)); do
	    echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
	    # Run the command with the current N_ROBOTS value
	    $COMMAND n_robots:=$N_ROBOTS
	done
done

mkdir -o ~/data/scenarios/fixed_priorities_1m
mv rosbag* ~/data/scenarios/fiexd_priorities_1m

export CCR_PRIORITIES="same"

for ((RUN = 1; RUN <= $N_RUNS; RUN++)); do
	for ((N_ROBOTS = 1; N_ROBOTS <= 8; N_ROBOTS++)); do
	    echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
	    # Run the command with the current N_ROBOTS value
	    $COMMAND n_robots:=$N_ROBOTS
	done
done

mkdir -o ~/data/scenarios/same_priorities_1m
mv rosbag* ~/data/scenarios/same_priorities_1m

export CCR_VERSION="global_planner_baseline"

for ((RUN = 1; RUN <= $N_RUNS; RUN++)); do
	for ((N_ROBOTS = 1; N_ROBOTS <= 8; N_ROBOTS++)); do
	    echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
	    # Run the command with the current N_ROBOTS value
	    $COMMAND n_robots:=$N_ROBOTS
	done
done
mv rosbag* ~/data/scenarios/ccr_1m
