#!/usr/bin/env python

import rclpy
import tf_transformations
import tf2_ros
import tf2_geometry_msgs # import needed to transform geometry_msgs with tf2
import yaml
from geometry_msgs.msg import PoseStamped
from driving_swarm_utils.node import DrivingSwarmNode
from termcolor import colored
from std_msgs.msg import Int32


class SimpleGoalProvider(DrivingSwarmNode):
    def __init__(self):
        super().__init__('simple_goal_provider')
        self.publisher = self.create_publisher(PoseStamped, 'nav/goal', 10)
        self.completed_publisher = self.create_publisher(Int32, 'nav/goal_completed', 10)
        self.completed = 0
        self.goal_list = []
        self.declare_parameter('goal_radius', 0.2)
        self.goal_dist = self.get_parameter('goal_radius').get_parameter_value().double_value
        self.current_goal_index = 0
        self.get_frames()
        
        # declare parameters
        # if parameter goals is given (non-empty string), it will be used
        self.declare_parameter('waypoints', '')

        # if parameter goals is not given, a list of goals will be created from the following parameters
        # x, y, theta (list of double)
        self.declare_parameter('x', [])
        self.declare_parameter('y', [])
        self.declare_parameter('theta', [])
        waypoints = self.get_parameter('waypoints').get_parameter_value().string_value
        
        if waypoints == '':
            xs = self.get_parameter('x').get_parameter_value().double_array_value
            ys = self.get_parameter('y').get_parameter_value().double_array_value
            thetas = self.get_parameter('theta').get_parameter_value().double_array_value
            goals = zip(xs, ys, thetas)
        else:
            goals = yaml.load(waypoints)

        for x,y,theta in goals:
            p = PoseStamped()
            p.header.frame_id = self.reference_frame
            #p.header.stamp = rclpy.time.Time().to_msg()
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            q = tf_transformations.quaternion_from_euler(0, 0, theta)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            self.goal_list.append(p)
        assert(len(self.goal_list) > 0)

        self.get_logger().info(f"{self.goal_list}")
        self.setup_tf()
        self.wait_for_tf()

        self.create_timer(0.1, self.timer_cb)
        
    def timer_cb(self):
        self.check_goal()
        self.publisher.publish(self.goal_list[self.current_goal_index])
    
    def increment_goal(self):
        self.completed += 1
        self.current_goal_index = self.completed % len(self.goal_list)
        completed_str = f"completed {self.completed} goal "
        self.get_logger().info(colored(completed_str, "green") + f" -> going to goal index {self.current_goal_index}")
        self.completed_publisher.publish(Int32(data=self.completed))
    
    def check_goal(self):
        goal = self.goal_list[self.current_goal_index]
        try:
            # get the transform so we can read at wich time it was performed
            t = self.tf_buffer.lookup_transform(goal.header.frame_id, self.own_frame, rclpy.time.Time().to_msg())
            # set the time to the most recent transform
            goal.header.stamp = t.header.stamp
            pose3d = self.tf_buffer.transform(goal, self.own_frame)
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
    rclpy.shutdown()

if __name__ == '__main__':
    main()