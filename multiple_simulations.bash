#!/usr/bin/bash

source ~/.rosrc

COMMAND="ros2 launch ccr ccr_sim_1m.launch.py use_rviz:=false use_rosbag:=true"
export RUN_TIMEOUT="600.0"
export INIT_TIMEOUT="300.0"
export ROS_SIMULATOR="gzserver"


make_runs() {
	mkdir -o $DATA_DIR
	for ((RUN = 1; RUN <= $N_RUNS; RUN++)); do
		for ((N_ROBOTS = 1; N_ROBOTS <= 8; N_ROBOTS++)); do
			echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
			# Run the command with the current N_ROBOTS value
			$COMMAND n_robots:=$N_ROBOTS
			mv rosbag* $DATA_DIR
		done
	done
}

export CCR_VERSION="global_planner_baseline"
export CCR_PRIORITIES="index"
export DATA_DIR="~/data/scenarios/sim/fixed_priorities_1m"
make_runs

export CCR_PRIORITIES="same"
export DATA_DIR="~/data/scenarios/sim/same_priorities_1m"
make_runs

export CCR_VERSION="global_planner"
export DATA_DIR="~/data/scenarios/sim/ccr_1m"
make_runs