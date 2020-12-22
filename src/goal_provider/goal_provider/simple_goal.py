#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class SimpleGoalProvider(Node):
    def __init__(self):
        super().__init__('simple_goal_provider')
        self.publisher = self.create_publisher(PoseStamped, 'nav/goal', 9)
        self.get_logger().info('Starting')
        self.create_timer(5.0, self.timer_cb)
        
    def timer_cb(self):
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.5
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.publisher.publish(pose)

def main():
    rclpy.init()
    node = SimpleGoalProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'got keyboard interrupt, shutting down')
        node.destroy_node()

if __name__ == '__main__':
    main()