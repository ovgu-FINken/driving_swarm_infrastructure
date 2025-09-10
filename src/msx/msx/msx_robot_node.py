#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

class MSXRobotNode(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sundata', # Change Topic later on
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f"Got data: {msg.data}")
    
def main():
    main_fn('MSXRobotNode', MSXRobotNode)

if __name__ == '__main__':
    main()
