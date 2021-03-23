#!/usr/bin/env python

import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from std_msgs.msg import String
from trajectory_generator.utils import *
from trajectory_generator.vehicle_model_node import TrajectoryGenerator, Vehicle
from driving_swarm_messages.srv import UpdateTrajectory


class DirectPlanner(Node):
    def __init__(self):
        super().__init__('direct_planner')
        self.get_logger().info('Starting')
        self.own_frame = 'base_link'
        self.reference_frame = 'map'
        self.goal = None
        self.started = False

        self.declare_parameter('vehicle_model')
        self.declare_parameter('step_size')
        self.declare_parameter('turn_radius')
        self.vm = TrajectoryGenerator(
            model = Vehicle(self.get_parameter('vehicle_model').get_parameter_value().integer_value),
            step = self.get_parameter('step_size').get_parameter_value().double_value,
            r = self.get_parameter('turn_radius').get_parameter_value().double_value
        )

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        qos_profile = rclpy.qos.qos_profile_system_default
        qos_profile.reliability = rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        qos_profile.durability = rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, 'status', qos_profile)

        self.create_subscription(String, '/command', self.command_cb, qos_profile)
        self.follow_action_client = self.create_client(UpdateTrajectory, 'nav/follow_trajectory')
        self.follow_action_client.wait_for_service()
        self.get_logger().info('connected to trajectory follower service')
        f = self.tfBuffer.wait_for_transform_async(self.own_frame, self.reference_frame, rclpy.time.Time().to_msg())
        self.get_logger().info('waiting for transform map -> baselink')
        rclpy.spin_until_future_complete(self, f)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 1)
        self.create_timer(0.1, self.timer_cb)
    
    def command_cb(self, msg):
        if msg.data == "go":
            self.get_logger().info("going")
            self.started = True
            self.status_pub.publish(String(data="running"))
        
    def set_goal(self, goal):
        self.goal_started = False
        self.goal = goal

    def goal_cb(self, msg):
        if self.goal is None:
            self.set_goal(msg)
            self.get_logger().info(f'got goal')#: {msg}')
            self.status_pub.publish(String(data="ready"))
            self.get_logger().info("ready")
        elif self.goal.pose != msg.pose:
            self.get_logger().info('got new goal')
            self.set_goal(msg)

    def timer_cb(self):
        if self.started and not self.goal_started:
            path = self.create_path()
            self.send_path(path)
            self.goal_started = True
            
    def create_path(self):
        waypoint_tuples = self.get_waypoints()
        if waypoint_tuples[0] is None:
            return
        
        return self.vm.tuples_to_path(waypoint_tuples)
        
        

    def send_path(self, trajectory):
        # convert trajectory to correct space
        path = Path()
        path.header.frame_id = self.reference_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in trajectory:
            pose3d = PoseStamped()
            pose3d.header.frame_id = self.reference_frame
            pose3d.header.stamp = self.get_clock().now().to_msg()
            pose3d.pose.position.x = pose[0]
            pose3d.pose.position.y = pose[1]
            pose3d.pose.position.z = 0.0
            pose3d.pose.orientation = yaw_to_orientation(pose[2]) 
            path.poses.append(pose3d)
        
        # create follow trajectory action goal
        self.get_logger().info('sending path to action server')
        ti = 0
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=ti)
        self.follow_action_client.call_async(request)
    
    def get_waypoints(self):
        start = None
        try:
            trans = self.tfBuffer.lookup_transform(self.reference_frame, self.own_frame, rclpy.time.Time().to_msg(), rclpy.time.Duration())
            frame = tf2_kdl.transform_to_kdl(trans)
            start = (frame.p.x(), frame.p.y(), frame.M.GetRPY()[2])
        except Exception as e: 
            self.get_logger().info(f"Exception in tf transformations\n{e}")
            
        goal = (
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            yaw_from_orientation(self.goal.pose.orientation)
        )
        return [start, goal]    

def main():
    rclpy.init()
    node = DirectPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting Down')
        node.destroy_node()

if __name__ == '__main__':
    main()