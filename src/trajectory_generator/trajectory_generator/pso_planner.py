#!/usr/bin/env python

import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
import numpy as np
from scipy import optimize
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from driving_swarm_messages.srv import VehicleModel
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, OccupancyGrid
from trajectory_generator.vehicle_model_node import TrajectoryGenerator, Vehicle


class DirectPlanner(Node):
    def __init__(self):
        super().__init__('pso_planner')
        self.get_logger().info('Starting')
        self.own_frame = 'base_link'
        self.reference_frame = 'map'
        self.current_gh = None

        self.w_time = 1.0
        self.w_length = 0.0
        self.w_danger = 1.0

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

        self.costmap_initalised = False
        self.create_subscription(OccupancyGrid, 'nav/obstacles', self.obstacle_cb, 9)

        self.follow_action_client = ActionClient(self, FollowPath, 'nav/follow_path')
        self.vm_client_futures = []
        self.action_client_futures = []
        self.action_result_futures = []
        self.client = self.create_client(VehicleModel, 'nav/vehicle_model')
        self.client.wait_for_service()
        self.get_logger().info('connected to VM service')
        self.follow_action_client.wait_for_server()
        self.get_logger().info('connected to trajectory follower service')
        self.goal = None
        f = self.tfBuffer.wait_for_transform_async(self.own_frame, self.reference_frame, rclpy.time.Time().to_msg())
        self.get_logger().info('waiting for transform map -> baselink')
        rclpy.spin_until_future_complete(self, f)
        while not self.costmap_initalised:
            rclpy.spin_once(self)

        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 9)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('setup done')
        
    def goal_cb(self, msg):
        if self.goal is None:
            self.get_logger().info(f'got goal')#: {msg}')
            self.goal = msg
            self.compute_path()
        elif self.goal.pose != msg.pose:
            self.get_logger().info('got new goal')
            self.goal = msg
            self.cancel_all_current_actions()
            self.compute_path()
        else:
            self.get_logger().debug('got old goal')
            
    def obstacle_cb(self, msg):
        self.costmap_initalised = True
        self.vm.set_costmap(msg)

    def cancel_all_current_actions(self):
        self.get_logger().info('cancel current actions')
        self.vm_client_futures = []
        self.action_client_futures = []
        if self.current_gh is not None:
            self.current_gh.cancel_goal_async()
        self.action_result_futures = []

    # TODO: it would be much nicer to create a callback attached to the future, however this is not as simple (?) 
    # probably needs a reentrant callback
    def timer_cb(self):
        #self.get_logger().info('timer cb')
        for future in self.action_client_futures:
            if future.done():
                self.get_logger().info('action submit complete')
                if self.current_gh is not None:
                    self.get_logger().info('Cancelling old goal')
                    self.current_gh.cancel_goal_async()
                self.action_client_futures.remove(future)
                self.current_gh = future.result()
                self.get_logger().info(f'goal accepted: {self.current_gh.accepted}')
                self.action_result_futures.append(self.current_gh.get_result_async())
        for future in self.action_result_futures:
            if future.done():
                self.get_logger().info('action complete')
                self.action_result_futures.remove(future)
                self.current_gh = None
                self.compute_path()
            
    def weighted_sum(self, traj):
        objectives = self.vm.compute_trajectory_objectives( traj )
        return self.w_time * objectives['time'] +\
            self.w_length * objectives['length']+ \
            self.w_danger * objectives['danger']

    def evaluate_solution(self, wp):
        trajectory = self.vm.tuples_to_path([self.start_wp, wp, self.goal_wp])
        return self.weighted_sum( trajectory )

    def compute_path(self):
        waypoints = self.get_waypoints()
        waypoint_tuples = [(p.x, p.y, p.theta, 1.0) for p in waypoints]
        trivial = self.vm.tuples_to_path(waypoint_tuples)
        self.start_wp = waypoint_tuples[0]
        self.goal_wp = waypoint_tuples[1]
        
        opt = optimize.minimize(self.evaluate_solution, [.0, .0, .0, 1.0],method='Nelder-Mead')
        self.get_logger().info(f'best is: {opt.x}')
        best = self.vm.tuples_to_path([self.start_wp, opt, self.goal_wp])
        # check if better than trivial solution
        if self.weighted_sum(trivial) < self.weighted_sum(best): 
            best = trivial
            self.get_logger().info(f'could not find better solution than trivial.')
        self.send_path([Pose2D(x=x, y=y, theta=theta) for x,y,theta in best])
        #self.vm.plot_trajectory_in_costmap(traj)

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

        
        # create follow trajectory action goal
        action_goal = FollowPath.Goal(path=path)
        self.get_logger().info('sending path to action server')
        self.action_client_futures.append(self.follow_action_client.send_goal_async(action_goal))
    
    def get_waypoints(self):
        start = Pose2D()
        try:
            trans = self.tfBuffer.lookup_transform(self.reference_frame, self.own_frame, rclpy.time.Time().to_msg(), timeout=rclpy.time.Duration(seconds=3.0))
            frame = tf2_kdl.transform_to_kdl(trans)
            start.theta = frame.M.GetRPY()[2]
            start.x = frame.p.x()
            start.y = frame.p.y()
            #self.get_logger().debug(f"got the following for ego position: {start}")
        except Exception as e: 
            self.get_logger().info(f"Exception in tf transformations\n{e}")
            
        goal = Pose2D()
        goal.x = self.goal.pose.position.x
        goal.y = self.goal.pose.position.y
        goal.theta = yaw_from_orientation(self.goal.pose.orientation)
        
        return [start, goal]    

    def convert_map_data_to_image(self, map_data):
        image = np.array(map_data.data, dtype=int)
        image = np.reshape(image, (map_data.info.width, map_data.info.height))
        image[image < 50] = 0
        image[image >= 50] = 100
        return image

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
        node.get_logger().info('Shutting Down')
        node.destroy_node()

if __name__ == '__main__':
    main()