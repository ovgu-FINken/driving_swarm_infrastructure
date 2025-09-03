#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from std_msgs.msg import String

class MSXRobotNode(DrivingSwarmNode):
    def __init__(self):
        super().__init__('robot_node')
        self.subscription = self.create_subscription(
            String,
            'topic', # Change Topic later on
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Got: "%s"' % msg.data)
    

def main(args=None):
    rclpy.init(args=args)
    node = MSXRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()