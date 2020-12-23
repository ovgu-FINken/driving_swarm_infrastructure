#!/usr/bin/env python
import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import Twist, Pose2D

# for the action server seehttps://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_queue_goals.py
# as a reference.  

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.get_logger().info('Starting trajectory follower')
        self.reference_frame = 'base_link'
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.cmd_vel = Twist()
        
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 9)

        self.follow_action_server = ActionServer(self, FollowPath, 'nav/follow_path', 
            handle_accepted_callback=self.handle_accepted_cb,
            execute_callback=self.execute_cb, 
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup())
            
        self.current_goal = None
        
    def handle_accepted_cb(self, goal_handle):
        """start or defer execution of an accepted goal"""
        #we always execute the new goal and abort the previously running goal
        self.get_logger().info(f'accepting: {goal_handle.goal_id}')
        self.get_logger().info(f'aborting: {self.current_goal}')
        self.current_goal = goal_handle.goal_id
        goal_handle.execute()
            
    def goal_cb(self, goal_request):
        """ Accept or reject client action request """
        # TODO: Check if start(?) pose and time are valid
        self.get_logger().info(f'got new goal request: {goal_request}')
        return GoalResponse.ACCEPT
    
    def cancel_cb(self, goal_handle):
        if goal_handle == self.current_goal:
            self.current_goal = None
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        self.get_logger().info(f'Executing Goal: {goal_handle.goal_id}')
        rate = self.create_rate(10.0)
        while rclpy.ok():
            if self.goal_reached():
                break

            if self.current_goal != goal_handle.goal_id:
                self.get_logger().info('goal not active any more')
                goal_handle.abort()
                # what to do actually?
                return FollowPath.Result()

            ego_pose = self.get_current_ego_pose() 
            desired_pose = self.get_current_desired_pose()
            x_diff = desired_pose.x - ego_pose.x
            y_diff = desired_pose.y - ego_pose.y 
            theta_diff = desired_pose.theta - ego_pose.theta
            
            px = 0.1
            py = 0.1
            p_theta = 0.1
            
            self.cmd_vel.linear.x = x_diff * px
            self.cmd_vel.angular.z = y_diff * py + theta_diff * p_theta

            rate.sleep()
            self.cmd_publisher.publish(self.cmd_vel)
        return FollowPath.Result()
    
    def goal_reached(self):
        return False
    
    def get_current_ego_pose(self):
        return Pose2D(x=0.0, y=0.0, theta=0.0)

    def get_current_desired_pose(self):
        return Pose2D(x=0.1, y=0.1, theta=0.1)

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
