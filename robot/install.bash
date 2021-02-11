#!/usr/bin/bash
# Executes after driving_swarm_infrastructure_update

cd $HOME/drving_swarm_infrastructure

sudo rm -r build/ install/ log/

CC=gcc CXX=g++ colcon build --symlink-install --cmake-clean-cache --parallel-workers 1
sudo reboot
