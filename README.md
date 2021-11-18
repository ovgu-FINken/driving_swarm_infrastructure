# Main Repository for the DrivingSwarm Software Framework

## Installation

To install and run the DrivingSwarm framework we strongly recommend to use the Ubuntu version 20.04, which goes along with the ROS2 foxy-fitzroy.
To install the DrivingSwarm package, you need to install [ROS2-foxy-fitzroy](https://docs.ros.org/en/foxy/Installation.html).

For this project we use the following directory structure:
```
- dependencies
  - src
  - install
  - build
  - log

- driving_swarm_infrastructure
  - src
  - install
  - build
  - log
```

With two different workspaces `dependencies` and `driving_swarm_infrastructure`

To install the dependencies, you can create a new folder `mkdir -p dependencies/src`, change to the root of this workspace `cd dependencies` and run  `vcs import --input https://raw.githubusercontent.com/ovgu-FINken/driving_swarm_infrastructure/master/repo-files/turtlebot3.repos src/`.
Now you can source your global ros2 environment with `source /opt/ros/foxy/setup.bash`. Install the dependencies for those packages with the rosdep tool and then build the packages with `colcon build`.
```
rosdep update
rosdep install --from-paths src/ --ignore-src -y
```
Now you need to build the depent packages with `colcon build` and source the dependencies workspace (`source PATH_TO/dependencies/install/setup.bash`) you used in your `.bashrc`.
Now you can clone and build the driving_swarm_infrastructure workspace, in a folder on the same level as the dependencies workspace.

The `.bashrc` file in your home directory should contain the following lines:
```
export TURTLEBOT3_MODEL=burger
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/models/:~/ros/dependencies/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:~/ros/driving_swarm_representation/src/driving_swarm_representation/models/
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/worlds/:~/ros/dependencies/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models
source /opt/ros/foxy/local_setup.bash
source ~/ros/dependencies/install/setup.bash
source ~/ros/driving_swarm_infrastructure/install/local_setup.bash
```

Once the dependencies are correctly installed, you should be able to build the main workspace of the DrivingSwarm with colcon build.
You can source the workspace, now you should be able to run your first simulation.


## Useage

To use the DrivingSwarm framework, you best start with running the launch-file `ros2 launch driving_swarm_bringup multi_robot.launch.py`.
The launch file will start
- The gazebo simulation (you can use `simulator:=gzserver` to disable the visualization)
- `single_robot.launch.py` for each robot, this launchfile will in turn launch serveral nodes for each robot
  - amcl-localization
  - nodes to translate TF-information between local- an global namespace
  - rviz
  - a simple behaviour (turtlebot3 drive, from the TurtleBot3 repos)

To run real robots you can use `multi_real_robot_fix_pos.launch` to launch multiple real robots with fixed starting positions.
If you want to define your own behavior, you can include the main launchfile and deactivate the behaviours via parameters
- `behavior:=False` will deactivate the behavior
- `use_rviz:=False` will disable rviz

See the [wiki](https://github.com/ovgu-FINken/driving_swarm_infrastructure/wiki) for further documentation.
