# Main Repository for the Driving Swarm Framework

## Installation

You have two options to install the ros package. If you have not set up ros2 humble, and you are not familiar with ros, we recommend the automatic installation using ansible. If you are already familiar with the ros2 ecosystem, we recommend to install the package manually.

- For automatic installation have a look at the [Ansible](https://github.com/ovgu-FINken/driving_swarm_ansible) (recommended if you install driving swarm for real robots)
- For the manual installation see the instructinos below


## Using Driving Swarm

To use the DrivingSwarm framework, lets take a look at the launch-file `ros2 launch driving_swarm_bringup multi_robot.launch.py`.
The launch file will start
- The gazebo simulation (you can use `simulator:=gzserver` to disable the visualization)
- `single_robot.launch.py` for each robot, this launchfile will in turn launch serveral nodes for each robot
  - amcl-localization
  - nodes to translate TF-information between local- an global namespace
  - rviz
- We use a map, a gazebo-world and a robot-configuration containing the starting position and (optional) additional information for each robots
- `n_robots:=4` will start the behavior with four robots (you can change the value according to your own preference)

To run real robots you can use `multi_real_robot_fix_pos.launch` to launch multiple real robots with fixed starting positions.
If you want to define your own behavior, you can include the main launchfile and deactivate the behaviours via parameters
- `behavior:=False` will deactivate the behavior
- `use_rviz:=False` will disable rviz
- `use_rosbag:=True` will start a rosbag recording, which can be used in data-analysis

## Examples

The package `driving_swarm_behaviour` contains nodes and launch files to start a simple reactive behaviour.

```
ros2 launch driving_swarm_behaviour reactive.launch.py n_robots:=2 data_file:=data.csv.gz run_timeout:=300.0 init_timeout:=60.0
```
What does this mean:
- `ros2 launch driving_swarm_behaviour reactive.launch.py` starts the [launch-file for reactive behaviour](https://github.com/ovgu-FINken/driving_swarm_infrastructure/blob/humble/src/driving_swarm_behaviour/launch/reactive.launch.py)
- `n_robots:=2` specifies the number of robots, which can also be set via env-variable `export N_ROBOTS=2`
- `data_file:=data.csv.gz` specifies that run data should be saved with the filename `data.csv.gz` which can be read via pandas `pd.read_csv`
- `run_timeout:=300.0` specifies that a simulation experiment (or real experiment) takes 300 seconds
- `init_timeout:=300.0` specifies that after 300 seconds initialization should be complete, otherwise the experiment is aborted
- `simulator:=gzserver` or `gazebo`, specifies if gazebo is run in headless mode
For driving swarm usually the launch-arguments have a default value. If nothing is specified, the default value will be used. Otherwise the value can be set via environment variables or launch arguments, where the arguments take precedence over environment variables.

What happens once you launch:
- The launch file will start all associated ros-nodes:
  - gazebo simulator, robot state publisher, etc. (i.e., the simulated robots)
  - rviz for each robot (`use_rviz:=false` to disable)
  - the amcl-localization from the nav2 packages
  - nodes that translate tf messages from the local tf topic of each robot (`/robot/tf`, `/robot/tf_static`) to a global tf topic (`/tf`, `/tf_static`)
  - a command node, which sends commands `go` and `stop` to the topic `/command`
  - a behaviour node for each robot
  - a node that records and saves experiment data in 1s time resolution





A simple example workspace using the DrivingSwarm framework can be found in the tutorials for our lecture [Introduction to Robotics](https://github.com/ovgu-FINken/introduction_to_robotics_tutorial/tree/main/src/reactive_behaviour).Each package in this workspace contains simple implementations for behaviors and a launch-file, which launches the behvior and the infrastructure neccessary for running an experiment with simulated and real robots.
See the [wiki](https://github.com/ovgu-FINken/driving_swarm_infrastructure/wiki) for further documentation.


## Manual Installation

To install and run the DrivingSwarm framework we strongly recommend to use the Ubuntu version 22.04, which goes along with the ROS2 humble.
To install the DrivingSwarm package, you need to install [ROS2-humble](https://docs.ros.org/en/humble/Installation.html).
We assume in this guide, that you install the workspace under the directory '~/ros', if you use another location adapt the paths to your directories accordingly.


For this project we use the following directory structure:
```
- ~/ros
  - driving_swarm_infrastructure
    - src
    - install
    - build
    - log

  - your_package
    - src
    - install
    - build
    - log
```

If you want to extend the functionality of the package, we recommend to use two different workspaces `driving_swarm_infrastructure` and `your_package`. As an example for how a package using the framework looks like see [Introduction to Robotics](https://github.com/ovgu-FINken/introduction_to_robotics_tutorial/).

```
cd `~/ros/driving_swarm_infrastructure`
rosdep update
rosdep install --from-paths src/ --ignore-src -y
```
Now you need to build the DrivingSwarm packages with `colcon build`.

To use the package you need to set the following variables in `~/.bashrc`:
```
export TURTLEBOT3_MODEL=burger
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/models/:/opt/ros/humble/share/turtlebot3_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/worlds/:/opt/ros/humble/share/turtlebot3_gazebo/models
source /opt/ros/humble/local_setup.bash
source ~/ros/driving_swarm_infrastructure/install/local_setup.bash
source ~/ros/introduction_to_robotics_tutorial/install/local_setup.bash
export PYTHONWARNINGS=ignore:::setuptools.command.install
```

Source your `.bashrc` or restart your shell to load the environment variables and you should be able to start your first simulation.
```
ros2 launch trajectory_follower trajectory_follower_test.launch.py n_robots:=2 simulator:=gzserver
```

# References

If you use this package in your own academic research, please cite the following paper:

[1] S. Mai, N. Traichel, and S. Mostaghim, “Driving Swarm: A Swarm Robotics Framework for Intelligent Navigation in a Self-organized World,” in 2022 IEEE International Conference on Robotics and Automation (ICRA), Philadelphia, PA, USA: IEEE, 2022, pp. 4958–4964. doi: 10.1109/ICRA46639.2022.9811852.
```
@inproceedings{Mai2022DrivingSwarm,
  title = {Driving {{Swarm}}: {{A Swarm Robotics Framework}} for {{Intelligent Navigation}} in a {{Self-organized World}}},
  booktitle = {2022 {{IEEE International Conference}} on {{Robotics}} and {{Automation}} ({{ICRA}})},
  author = {Mai, Sebastian and Traichel, Nele and Mostaghim, Sanaz},
  year = {2022},
  pages = {4958--4964},
  publisher = {{IEEE}},
  address = {{Philadelphia, PA, USA}},
  doi = {10.1109/ICRA46639.2022.9811852},
}

```
