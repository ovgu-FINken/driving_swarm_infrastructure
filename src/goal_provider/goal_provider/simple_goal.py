#!/usr/bin/env python

import rclpy
import PyKDL
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion


class SimpleGoalProvider(Node):
    def __init__(self):
        super().__init__('simple_goal_provider')
        self.publisher = self.create_publisher(PoseStamped, 'nav/goal', 9)
        self.get_logger().info('Starting')
        self.goal_list = []
        self.current_goal_index = 0
        self.declare_parameter('x')
        self.declare_parameter('y')
        self.declare_parameter('theta')
        xs = self.get_parameter('x').get_parameter_value().double_array_value
        ys = self.get_parameter('y').get_parameter_value().double_array_value
        thetas = self.get_parameter('theta').get_parameter_value().double_array_value
        for x,y,theta in zip(xs, ys, thetas):
            p = PoseStamped()
            p.header.frame_id = 'map'
            #p.header.stamp = rclpy.time.Time().to_msg()
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            q = PyKDL.Rotation.RPY(0,0,theta).GetQuaternion()
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            self.goal_list.append(p)
        self.create_timer(1.0, self.timer_cb)
        
    def timer_cb(self):
        # todo: change goal when first goal is reached
        self.publisher.publish(self.goal_list[self.current_goal_index])

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