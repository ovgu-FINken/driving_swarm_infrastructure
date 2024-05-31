#!/usr/bin/bash

source ~/.rosrc

N_RUNS=$1
MODE="fake"
#MODE="real"
#MODE="sim"
export RUN_TIMEOUT="600.0"
export INIT_TIMEOUT="300.0"
export ROS_SIMULATOR="gzserver"
BASE_DIR=~/data/experiment_$(date +%Y-%m-%d)
MAX_ROBOTS=5


make_runs() {
	mkdir -p $DATA_DIR
	for ((RUN = 1; RUN <= $N_RUNS; RUN++)); do
		for ((N_ROBOTS = 1; N_ROBOTS <= MAX_ROBOTS; N_ROBOTS++)); do
			echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
			# Run the command with the current N_ROBOTS value
			ros2 launch ccr ccr_$MODE.launch.py use_rviz:=false use_rosbag:=true n_robots:=$N_ROBOTS waypoints_file:=$MAP
			mv rosbag* $DATA_DIR
		done
	done
}

export MAP="icra2024_waypoints.yaml"
export MODE="fake"

export CCR_VERSION="global_planner_baseline"

export CCR_PRIORITIES="index"
export DATA_DIR=$BASE_DIR/$MODE.$CCR_PRIORITIES.$CCR_VERSION.$MAP
make_runs

export CCR_PRIORITIES="same"
export DATA_DIR=$BASE_DIR/$MODE.$CCR_PRIORITIES.$CCR_VERSION.$MAP
make_runs

export MODE="sim"
export CCR_PRIORITIES="index"
export DATA_DIR=$BASE_DIR/$MODE.$CCR_PRIORITIES.$CCR_VERSION.$MAP
make_runs

export CCR_PRIORITIES="same"
export DATA_DIR=$BASE_DIR/$MODE.$CCR_PRIORITIES.$CCR_VERSION.$MAP
make_runs

# CCR
export CCR_VERSION="global_planner"

export MODE="fake"
export DATA_DIR=$BASE_DIR/$MODE.$CCR_PRIORITIES.$CCR_VERSION.$MAP
make_runs

export MODE="sim"
export DATA_DIR=$BASE_DIR/$MODE.$CCR_PRIORITIES.$CCR_VERSION.$MAP
make_runs