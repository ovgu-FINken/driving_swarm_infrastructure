
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64

class ErrorCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.sub_robot_positions = self.create_subscription(
            Float64MultiArray,
            '/robotPos/data',
            self.listener_callback,
            10
        )

        self.pub_sunburst_error = self.create_publisher(Float64, "/sunburstError", 100)


def main():
    main_fn('ErrorCalc', ErrorCalc)

if __name__ == '__main__':
    main()
