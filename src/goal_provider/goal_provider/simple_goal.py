#!/usr/bin/env python

import rclpy
import PyKDL
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion


class SimpleGoalProvider(Node):
    def __init__(self):
        super().__init__('simple_goal_provider')
        self.reference_frame = 'world'
        self.ego_frame = 'base_link'
        self.publisher = self.create_publisher(PoseStamped, 'nav/goal', 9)
        self.get_logger().info('Starting')
        self.goal_list = []
        self.goal_dist = 0.2
        self.current_goal_index = 0
        self.declare_parameter('x')
        self.declare_parameter('y')
        self.declare_parameter('theta')
        xs = self.get_parameter('x').get_parameter_value().double_array_value
        ys = self.get_parameter('y').get_parameter_value().double_array_value
        thetas = self.get_parameter('theta').get_parameter_value().double_array_value
        for x,y,theta in zip(xs, ys, thetas):
            p = PoseStamped()
            p.header.frame_id = self.reference_frame
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

        self.get_logger().info(f"{self.goal_list}")
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        f = self.tfBuffer.wait_for_transform_async(self.ego_frame, self.reference_frame, rclpy.time.Time().to_msg())
        self.get_logger().info('waiting for transform map -> baselink')
        rclpy.spin_until_future_complete(self, f)

        self.create_timer(0.1, self.timer_cb)
        
    def timer_cb(self):
        self.check_goal()
        self.publisher.publish(self.goal_list[self.current_goal_index])
    
    def increment_goal(self):
        self.current_goal_index = (self.current_goal_index + 1) % len(self.goal_list)
        self.get_logger().info(f"goal completed, going to goal {self.current_goal_index}")
    
    def check_goal(self):
        goal = self.goal_list[self.current_goal_index]
        try:
            # get the transform so we can read at wich time it was performed
            t = self.tfBuffer.lookup_transform(goal.header.frame_id, self.ego_frame, rclpy.time.Time().to_msg())
            # set the time to the most recent transform
            goal.header.stamp = t.header.stamp
            pose3d = self.tfBuffer.transform(goal, self.ego_frame)
            x = pose3d.pose.position.x
            y = pose3d.pose.position.y
            if x**2 + y**2 < self.goal_dist**2:
                self.increment_goal()
        except Exception as e:
            self.get_logger().info(f'{e}')

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