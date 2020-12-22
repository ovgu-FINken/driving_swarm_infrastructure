#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from nav2_msgs.action import FollowPath

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.get_logger().info('Starting trajectory follower')
        #self.follow_action = self.

    def action_cb(self):
        self.get_logger().info('Got action ...')
        pass

def main():
    rclpy.init()
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down after user interrupt')
    node.destroy_node()


if __name__ == '__main__':
    main()
