
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
import functools
import numpy as np

class ErrorCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.get_list_of_robot_names()

        self.robot_pos = None
        self.waldo_pos = None

        for robot in self.robots:
            self.create_subscription(Float64MultiArray, f"/{robot}/sunburstRobotCalc/waldoPosition", functools.partial(self.estimated_waldo_pos_cb, robot), 10)

        self.sub_robots_pos = self.create_subscription(
            Float64MultiArray, "/robotPos", self.robot_pos_cb, 10
        )
        self.sub_waldo_pos = self.create_subscription(
            Point, "/waldoPos", self.waldo_pos_cb, 10
        )

    def estimated_waldo_pos_cb(self, robot, msg):
        self.get_logger().info(f'got msg for {robot}: {msg}')

    def robot_pos_cb(self, msg):
        # Convert Float64MultiArray back to Nx2 matrix
        data = np.array(msg.data)
        if len(data) % 2 != 0:
            self.get_logger().warn("Robot position data size is not even — ignoring message")
            return

        self.robot_positions = data.reshape((-1, 2))

    def waldo_pos_cb(self, msg):
        # msg is geometry_msgs/Point
        self.waldo_position = np.array([msg.x, msg.y])

def main():
    main_fn('ErrorCalc', ErrorCalc)

if __name__ == '__main__':
    main()
