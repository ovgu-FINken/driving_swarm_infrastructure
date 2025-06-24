#!/usr/bin/bash

source ~/.rosrc

N_RUNS=$1
#MODE="fake"
#MODE="real"
MODE="sim"
export RUN_TIMEOUT="600.0"
export INIT_TIMEOUT="300.0"
export ROS_SIMULATOR="gzserver"
BASE_DIR=~/ros/drving_swarm_infrastructure/Run_Data_FolderPBS
MAX_ROBOTS=5

make_runs() {
	DATA_DIR=$BASE_DIR
	for ((RUN = 1; RUN <= N_RUNS; RUN++)); do
		for ((N_ROBOTS = 1; N_ROBOTS <= MAX_ROBOTS; N_ROBOTS++)); do
			echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS (Run $RUN)"
			
			# Starte Launch-File
			ros2 launch ccr ccr_$MODE.launch.py use_rviz:=false use_rosbag:=false n_robots:=$N_ROBOTS
			
			# Zielordner für diese Run-Kombination
			RUN_DIR=$DATA_DIR/run_${RUN}_robots_${N_ROBOTS}
			mkdir -p $RUN_DIR
			
			# Verschiebe alle rosbag-Dateien in den Zielordner
			mv data.csv.gz $RUN_DIR
		done
	done
}

#make_runs() {
#	DATA_DIR=$BASE_DIR/$MODE.$CCR_PRIORITIES.$CCR_VERSION.$MAP
#	mkdir -p $DATA_DIR
#	for ((RUN = 1; RUN <= $N_RUNS; RUN++)); do
#		for ((N_ROBOTS = 1; N_ROBOTS <= MAX_ROBOTS; N_ROBOTS++)); do
#			echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
#			# Run the command with the current N_ROBOTS value
#			ros2 launch ccr ccr_$MODE.launch.py use_rviz:=false use_rosbag:=true n_robots:=$N_ROBOTS waypoints_file:=$MAP
#			mv rosbag* $DATA_DIR
#		done
#	done
#}

export MAP="icra2024_waypoints.yaml"
export MODE="fake"
export RUN_TIMEOUT="600.0"
export CCR_VERSION="centralized_planner"

#export MAP="icra2024_waypoints.yaml"
#export MODE="fake"
#export RUN_TIMEOUT="300.0"
#export CCR_VERSION="global_planner_baseline"

#export CCR_PRIORITIES="index"
#make_runs

#export CCR_PRIORITIES="same"
#make_runs

#export CCR_VERSION="global_planner"
#make_runs

#export MODE="sim"
#export RUN_TIMEOUT="900.0"
#export CCR_VERSION="global_planner_baseline"
#export CCR_PRIORITIES="index"
#make_runs

#export CCR_PRIORITIES="same"
#make_runs

#export CCR_VERSION="global_planner"
#make_runs