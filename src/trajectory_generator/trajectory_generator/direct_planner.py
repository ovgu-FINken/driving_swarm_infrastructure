#!/usr/bin/env python

import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from driving_swarm_messages.srv import VehicleModel
from nav_msgs.msg import Path


#def pose_stamped_to_transform(pose_stamped):
#    transform = PyKDL.TransformStamped()
#    transform = TransformStamped()
#    transform.header = pose_stamped.header
#    transform.transform.rotation = pose_stamped.pose.orientation
#    transform.transform.translation.x = pose_stamped.pose.position.x
#    transform.transform.translation.y = pose_stamped.pose.position.y
#    transform.transform.translation.z = pose_stamped.pose.position.z
#    return transform

class DirectPlanner(Node):
    def __init__(self):
        super().__init__('direct_planner')
        self.get_logger().info('Starting')
        self.own_frame = 'base_link'
        self.reference_frame = 'map'

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.client_futures = []
        self.client = self.create_client(VehicleModel, 'nav/vehicle_model')
        self.client.wait_for_service()
        self.get_logger().info('connected to VM service')
        self.goal = None
        self.create_timer(0.1, self.timer_cb)
        self.path_publisher = self.create_publisher(Path, 'nav/path', 9)
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
        # publish trajectory
        # create follow trajectory action goal
        pass 
    
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

def main():
    rclpy.init()
    node = DirectPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == '__main__':
    main()