#!/usr/bin/bash

source ~/.rosrc

N_RUNS=$1
MODE="fake"
#MODE="real"
#MODE="sim"
COMMAND="ros2 launch ccr ccr_$MODE.launch.py use_rviz:=false use_rosbag:=true"
export RUN_TIMEOUT="600.0"
export INIT_TIMEOUT="300.0"
export ROS_SIMULATOR="gzserver"
BASE_DIR=~/data/test/
MAX_ROBOTS=5

make_runs() {
	mkdir -p $DATA_DIR
	for ((RUN = 1; RUN <= $N_RUNS; RUN++)); do
		for ((N_ROBOTS = 1; N_ROBOTS <= MAX_ROBOTS; N_ROBOTS++)); do
			echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
			# Run the command with the current N_ROBOTS value
			$COMMAND n_robots:=$N_ROBOTS
			mv rosbag* $DATA_DIR
		done
	done
}

export MODE="fake"
export CCR_VERSION="global_planner_baseline"

export CCR_PRIORITIES="index"
export DATA_DIR=$BASE_DIR/$MODE_$CCR_PRIORITIES_$CCR_VERSION
make_runs

export CCR_PRIORITIES="same"
export DATA_DIR=$BASE_DIR/$MODE_$CCR_PRIORITIES_$CCR_VERSION
make_runs

export MODE="sim"
export CCR_PRIORITIES="index"
export DATA_DIR=$BASE_DIR/$MODE_$CCR_PRIORITIES_$CCR_VERSION
make_runs

export CCR_PRIORITIES="same"
export DATA_DIR=$BASE_DIR/$MODE_$CCR_PRIORITIES_$CCR_VERSION
make_runs