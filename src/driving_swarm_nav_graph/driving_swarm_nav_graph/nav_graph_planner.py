import rclpy

from driving_swarm_nav_graph.nav_graph import NavGraphNode
from trajectory_generator.vehicle_model_node import (
    TrajectoryGenerator,
    Vehicle,
)

import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
import numpy as np
import traceback
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32
from driving_swarm_messages.srv import UpdateTrajectory
from std_srvs.srv import Empty
from trajectory_generator.utils import yaw_from_orientation, yaw_to_orientation
from driving_swarm_nav_graph.utils import *


class NavGraphLocalPlanner(NavGraphNode):
    def __init__(self):
        super().__init__()
        self.get_logger().info("Starting")
        self.own_frame = "base_link"
        self.reference_frame = "map"
        self.plan = None

        self.started = False
        self.current_trajectory = None

        self.declare_parameter("vehicle_model")
        self.declare_parameter("step_size")
        self.declare_parameter("turn_radius")
        self.declare_parameter("turn_speed")
        self.vm = TrajectoryGenerator(
            model=Vehicle(
                self.get_parameter("vehicle_model")
                .get_parameter_value()
                .integer_value
            ),
            step=self.get_parameter("step_size")
            .get_parameter_value()
            .double_value,
            r=self.get_parameter("turn_radius")
            .get_parameter_value()
            .double_value,
            r_step=self.get_parameter("turn_speed")
            .get_parameter_value()
            .double_value,
        )

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        qos_profile = rclpy.qos.qos_profile_system_default
        qos_profile.reliability = (
            rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )
        qos_profile.durability = (
            rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )
        self.status_pub = self.create_publisher(String, "status", qos_profile)

        self.create_subscription(
            String, "/command", self.command_cb, qos_profile
        )
        self.follow_client = self.create_client(
            UpdateTrajectory, "nav/follow_trajectory"
        )
        self.create_service(Empty, "nav/replan", self.replan_callback)
        self.follow_client.wait_for_service()
        self.get_logger().info("connected to trajectory follower service")
        f = self.tfBuffer.wait_for_transform_async(
            self.own_frame, self.reference_frame, rclpy.time.Time().to_msg()
        )
        self.get_logger().info("waiting for transform map -> baselink")
        self.cell_publisher = self.create_publisher(Int32, "nav/cell", 1)
        rclpy.spin_until_future_complete(self, f)
        self.status_pub.publish(String(data="ready"))
        self.create_subscription(String, "nav/plan", self.plan_cb, 1)
        self.create_timer(1.0, self.timer_cb)
    
    
    def timer_cb(self):
        try:
            trans = self.tfBuffer.lookup_transform(
                self.reference_frame,
                self.own_frame,
                rclpy.time.Time().to_msg(),
            )
            frame = tf2_kdl.transform_to_kdl(trans)
            pose = (frame.p.x(), frame.p.y(), frame.M.GetRPY()[2])

        except Exception as e:
            self.get_logger().warn(f"Exception in tf transformations\n{e}")
            return
        
        node = find_nearest_node(self.g, (pose[0], pose[1]))
        if not node:
            self.get_logger().info(f'node: {node}')
        self.cell_publisher.publish(Int32(data=int(node)))
    
    def plan_cb(self, msg):
        plan = yaml.safe_load(msg.data)
        if self.plan is None and self.started and plan:
            self.status_pub.publish(String(data="running"))
        if self.plan != plan:
            self.get_logger().info(f'plan:{self.plan}')
            self.go_to_goal(plan) 

    def command_cb(self, msg):
        if msg.data == "go":
            self.get_logger().info("going")
            self.started = True

    def send_path(self, trajectory, ti=0):
        # convert trajectory to correct space
        if trajectory is None:
            return
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

        self.get_logger().info("sending path")
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=ti)
        self.follow_client.call_async(request)

    def go_to_goal(self, plan):
        if self.plan is not None:
            if plan[-1] == self.plan[-1]:
                self.plan == plan
                return
        self.plan = plan
        self.execute_plan()
        
    def execute_plan(self):
        try:
            trans = self.tfBuffer.lookup_transform(
                self.reference_frame,
                self.own_frame,
                rclpy.time.Time().to_msg(),
            )
            frame = tf2_kdl.transform_to_kdl(trans)
            start = (frame.p.x(), frame.p.y(), frame.M.GetRPY()[2])

        except Exception as e:
            self.get_logger().warn(f"Exception in tf transformations\n{e}")
            return
        waypoints = self.gen_rtr_path(start)

        self.get_logger().info(f'wps: {waypoints}')
        trajectory = self.vm.tuples_to_path(waypoints)
        self.send_path(trajectory)
    
    def replan_callback(self, _, res):
        if self.started:
            self.get_logger().info('got replanning request, which we will do')
        self.execute_plan()
        return res
    
    def gen_rtr_path(self, start):
        if start is None:
            return
        poly = poly_from_path(self.g, self.plan) 
        # set goal from final node in path
        goal = self.g.vp['geometry'][self.plan[-1]].inner.centroid.coords[0]
        path = waypoints_through_poly(self.g, poly, start[:2], goal, eps=0.01)
        pose_path = [[coords[0], coords[1], np.nan, 1.0] for coords in path.coords]
        pose_path = [start] + pose_path + [(*goal, np.nan)]
        return pose_path


def main():
    rclpy.init()
    node = NavGraphLocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()