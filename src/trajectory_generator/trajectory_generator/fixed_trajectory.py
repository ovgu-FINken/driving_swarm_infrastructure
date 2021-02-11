#!/usr/bin/env python

import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
import numpy as np
import yaml
from scipy import optimize
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from driving_swarm_messages.srv import VehicleModel, UpdateTrajectory
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, OccupancyGrid
from std_srvs.srv import Empty
from trajectory_generator.vehicle_model_node import TrajectoryGenerator, Vehicle
from time import time


class FixedTrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_provider')
        self.get_logger().info('Starting')
        self.own_frame = 'base_link'
        self.reference_frame = 'map'
        self.client_futures = []

        self.declare_parameter('vehicle_model')
        self.declare_parameter('step_size')
        self.declare_parameter('turn_radius')
        self.declare_parameter('trajectory_file')
        self.vm = TrajectoryGenerator(
            model = Vehicle(self.get_parameter('vehicle_model').get_parameter_value().integer_value),
            step = self.get_parameter('step_size').get_parameter_value().double_value,
            r = self.get_parameter('turn_radius').get_parameter_value().double_value
        )

        self.trajectory = None
        with open(self.get_parameter('trajectory_file').get_parameter_value().string_value, 'r') as stream:
            trajectories = yaml.safe_load(stream)
            self.trajectory = trajectories[self.get_namespace()]

        self.tfBuffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=10))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        self.follow_action_client = self.create_client(UpdateTrajectory, f'nav/follow_trajectory')
        self.follow_action_client.wait_for_service()
        self.get_logger().info('connected to trajectory follower service')
        self.goal = None
        f = self.tfBuffer.wait_for_transform_async(self.own_frame, self.reference_frame, rclpy.time.Time().to_msg())
        self.get_logger().info('waiting for transform map -> baselink')
        rclpy.spin_until_future_complete(self, f)
        
        self.create_service(Empty, 'nav/start', self.start_cb)

        self.get_logger().info('setup done')
        
    def start_cb(self, req, res):
        traj_request = UpdateTrajectory.Request()
        traj_request.trajectory = self.follow_action_client.call_async(traj_request)
        
        return res        

def yaw_from_orientation(orientation):
    rot = PyKDL.Rotation.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
    return rot.GetRPY()[2]

def yaw_to_orientation(yaw):
    q = PyKDL.Rotation.RPY(0,0,yaw).GetQuaternion()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    

def main():
    rclpy.init()
    node = FixedTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting Down')
        node.destroy_node()

if __name__ == '__main__':
    main()