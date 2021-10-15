# Main Repository for the DrivingSwarm Software Framework

## Installation

To install and run the DrivingSwarm framework we strongly recommend to use the Ubuntu version 20.04, which goes along with the ROS2 foxy-fitzroy.
To install the DrivingSwarm package, you need to install [ROS2-foxy-fitzroy](https://docs.ros.org/en/foxy/Installation.html).

The easiest way to run the project is then to check out the repositories listed in the directory
`driving_swarm_infrastructure/repo-files/` with `vcs import`. Install the dependencies for those packages with the rosdep tool.
`rosdep update
rosdep install --from-paths src/ --ignore-src -y`.
Now you need to build the depent packages with `colcon build` and source the workspace you used in your `.bashrc`.

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
