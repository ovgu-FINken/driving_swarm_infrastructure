#!/usr/bin/env python
import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
import numpy as np
import time

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, Quaternion
from nav_msgs.msg import Path


# for the action server seehttps://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_queue_goals.py
# as a reference. 

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.get_logger().info('Starting trajectory follower')
        self.reference_frame = 'base_link'
        self.trajectory = None
        self.current_goal = None
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.cmd_vel = Twist()
        
        #todo: eventually make this a configurable parameter
        self.fail_radius = 0.3
        
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 9)
        self.desired_pose_publisher = self.create_publisher(PoseStamped, 'nav/desired', 9)
        self.path_publisher = self.create_publisher(Path, 'nav/trajectory', 9)

        self.follow_action_server = ActionServer(self, FollowPath, 'nav/follow_path', 
            handle_accepted_callback=self.handle_accepted_cb,
            execute_callback=self.execute_cb, 
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup())

        self.create_timer(0.1, self.timer_cb)


    def pose2D_to_PoseStamped(self, pose2d):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.reference_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = pose2d.x
        pose_stamped.pose.position.y = pose2d.y
        pose_stamped.pose.position.z = 0.0
        q = Quaternion()
        q.x, q.y, q.z, q.w = PyKDL.Rotation.RPY(.0, .0, pose2d.theta).GetQuaternion()
        pose_stamped.pose.orientation = q
        return pose_stamped

    def poseStamped_to_Pose2D(self, pose_stamped):
        pose2d = Pose2D()
        try:
            # get the transform so we can read at wich time it was performed
            t = self.tfBuffer.lookup_transform(pose_stamped.header.frame_id, self.reference_frame, rclpy.time.Time().to_msg())
            # set the time to the most recent transform
            pose_stamped.header.stamp = t.header.stamp
            pose3d = self.tfBuffer.transform(pose_stamped, self.reference_frame)
            pose2d.x = pose3d.pose.position.x
            pose2d.y = pose3d.pose.position.y
            q = pose3d.pose.orientation
            pose2d.theta = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w).GetRPY()[2]
            
        except Exception as e:
            self.get_logger().warn(f'could not transform pose to reference frame \n {e}')
        return pose2d
        
    def handle_accepted_cb(self, goal_handle):
        """start or defer execution of an accepted goal"""
        #we always execute the new goal and abort the previously running goal
        self.get_logger().info(f'accepting: {goal_handle}')
        self.current_goal = goal_handle
        self.trajectory = goal_handle.request.path
            
    def goal_cb(self, goal_request):
        """ Accept or reject client action request """
        # TODO: Check if start(?) pose and time are valid
        #self.get_logger().info(f'got new goal request: {goal_request}')
        self.get_logger().info(f'got new goal request.')
        return GoalResponse.ACCEPT
    
    def cancel_cb(self, goal_handle):
        if goal_handle.goal_id == self.current_goal.goal_id:
            self.current_goal = None
            self.trajectory = None
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        goal_handle.succeed()
        return FollowPath.Result()
    
    def timer_cb(self):
        if self.trajectory is None:
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
            return

        ego_pose = self.get_current_ego_pose() 
        desired_pose, desired_vel = self.get_current_desired_pose_vel()
        x_diff = desired_pose.x - ego_pose.x
        y_diff = desired_pose.y - ego_pose.y 
        if x_diff**2 + y_diff**2 > self.fail_radius**2:
            self.get_logger().info('canceling goal because to far from planned pose')
            self.trajectory = None
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
        theta_diff = desired_pose.theta - ego_pose.theta
        
        px = 1.0
        py = 1.0
        p_theta = 1.0
        
        self.cmd_vel.linear.x = desired_vel.linear.x + x_diff * px
        self.cmd_vel.angular.z = desired_vel.angular.z + y_diff * py + theta_diff * p_theta

        #self.get_logger().info(f'x={self.cmd_vel.linear.x}, theta={self.cmd_vel.angular.z}')

        if self.trajectory is None:
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
            return
        else:
            self.desired_pose_publisher.publish(self.pose2D_to_PoseStamped(desired_pose))
            self.path_publisher.publish(self.trajectory)
            self.cmd_publisher.publish(self.cmd_vel)

    def get_current_ego_pose(self):
        # this works because reference frame == ego frame
        return Pose2D(x=0.0, y=0.0, theta=0.0)

    def get_current_desired_pose_vel(self):
        # get last pose, next pose and dt
        NANO = 0.001 ** 3
        current_stamp = self.get_clock().now()
        trajectory_start = rclpy.time.Time.from_msg(self.trajectory.header.stamp)
        rate = .3
        t = (current_stamp - trajectory_start).nanoseconds * NANO * rate
        if t < 0.0:
            self.get_logger().warn('trajectory start is in the future')
            t = 0.0
        
        last_index = int(np.floor(t))
        if last_index+2 >= len(self.trajectory.poses):
            self.trajectory = None
            return Pose2D(), Twist()
        dt = (t - np.floor(t))
        #self.get_logger().info(f't={t / rate}, dt={dt}, index: {last_index}')
        
        last_pose = self.poseStamped_to_Pose2D(self.trajectory.poses[last_index])
        next_pose = self.poseStamped_to_Pose2D(self.trajectory.poses[last_index + 1])
        #self.get_logger().info(f'last: {last_pose}')
        #self.get_logger().info(f'next: {next_pose}')
        
        x = (1 - dt) * last_pose.x + dt * next_pose.x
        y = (1 - dt) * last_pose.y + dt * next_pose.y
        theta = (1 - dt) * last_pose.theta + dt * next_pose.theta

        vel = Twist()
        vel.linear.x = (next_pose.x - last_pose.x) * rate * 2
        vel.angular.z = (next_pose.theta - last_pose.theta) * rate
        #self.get_logger().info(f'linear vel setpoint={vel}')
        return Pose2D(x=x, y=y, theta=theta), vel

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
