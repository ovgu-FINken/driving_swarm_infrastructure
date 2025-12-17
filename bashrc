#source ~/.bashrc
# export general settings
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export N_ROBOTS=1
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source ~/ros/driving_swarm_infrastructure/install/local_setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/models/:~/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/worlds/
# Supress deprecation warning for setuptools format
export PYTHONWARNINGS=ignore:::setuptools.command.install
