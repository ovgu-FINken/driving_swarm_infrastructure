#!/usr/bin/bash
# Executes after driving_swarm_infrastructure_update
source ~/.bashrc
source /opt/ros/foxy/setup.bash

cd /home/turtle/driving_swarm_infrastructure

sudo rm -r build/ install/ log/

CC=gcc CXX=g++ colcon build --symlink-install --cmake-clean-cache --parallel-workers 1
sudo reboot
