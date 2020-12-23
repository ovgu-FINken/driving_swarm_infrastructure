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
from driving_swarm_messages.srv import VehicleModel
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path


class DirectPlanner(Node):
    def __init__(self):
        super().__init__('direct_planner')
        self.get_logger().info('Starting')
        self.own_frame = 'base_link'
        self.reference_frame = 'map'

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.path_publisher = self.create_publisher(Path, 'nav/path', 9)
        self.follow_action_client = ActionClient(self, FollowPath, 'nav/follow_path')
        self.client_futures = []
        self.client = self.create_client(VehicleModel, 'nav/vehicle_model')
        self.client.wait_for_service()
        self.get_logger().info('connected to VM service')
        self.follow_action_client.wait_for_server()
        self.get_logger().info('connected to trajectory follower service')
        self.goal = None
        self.create_timer(0.1, self.timer_cb)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 9)
        
    def goal_cb(self, msg):
        self.get_logger().info(f'got new goal')#: {msg}')
        if self.goal is None or self.goal != msg:
            self.goal = msg
            self.create_path_request()

    # TODO: it would be much nicer to create a callback attached to the future, however this is not as simple (?) 
    # probably needs a reentrant callback
    def timer_cb(self):
        for future in self.client_futures:
            if future.done():
                self.get_logger().debug(f'finished {future.result()}')
                self.client_futures.remove(future)
                self.send_path(future.result().trajectory)
            
    def create_path_request(self):
        request = VehicleModel.Request()
        request.waypoints = self.get_waypoints()
        request.speeds = [1.0, 1.0]
        self.get_logger().debug(f"sending request")
        vm_future = self.client.call_async(request)
        self.client_futures.append(vm_future)

    def send_path(self, trajectory):
        # convert trajectory to correct space
        path = Path()
        path.header.frame_id = self.reference_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in trajectory:
            pose3d = PoseStamped()
            pose3d.header.frame_id = self.reference_frame
            pose3d.header.stamp = self.get_clock().now().to_msg()
            pose3d.pose.position.x = pose.x
            pose3d.pose.position.y = pose.y
            pose3d.pose.position.z = 0.0
            pose3d.pose.orientation = yaw_to_orientation(pose.theta) 
            
            path.poses.append(pose3d)

        # publish trajectory
        self.path_publisher.publish(path) 

        # create follow trajectory action goal
        
        action_goal = FollowPath.Goal(path=path)
        
        self.follow_action_client.send_goal_async(action_goal)
    
    def get_waypoints(self):
        start = Pose2D()
        try:
            trans = self.tfBuffer.lookup_transform(self.own_frame, self.reference_frame, rclpy.time.Time())
            frame = tf2_kdl.transform_to_kdl(trans)
            start.theta = frame.M.GetRPY()[2]
            start.x = frame.p.x()
            start.y = frame.p.y()
            #self.get_logger().debug(f"got the following for ego position: {start}")
        except: 
            self.get_logger().info("Exception in tf transformations")
            
        goal = Pose2D()
        try:
            x = self.tfBuffer.transform(self.goal, self.reference_frame)
            goal.x = x.pose.position.x
            goal.y = x.pose.position.y
            goal.theta = yaw_from_orientation(x.pose.orientation)

            self.get_logger().debug(f"got the following for goal position: {goal}")
        except Exception as e: 
            self.get_logger().info(f"Exception in goal transformation {e}")
        
        return [start, goal]    

def yaw_from_orientation(orientation):
    rot = PyKDL.Rotation.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
    return rot.GetRPY()[2]

def yaw_to_orientation(yaw):
    q = PyKDL.Rotation.RPY(0,0,yaw).GetQuaternion()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    

def main():
    rclpy.init()
    node = DirectPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == '__main__':
    main()