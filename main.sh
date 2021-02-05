#!/user/bin/bash
set -e

# Build the new sources
cd $HOME/driving_swarm_infrastructure

source /opt/ros/foxy/setup.bash

CC=gcc CXX=g++ colcon build --symlink-install --cmake-clean-cache --parallel-workers 1
