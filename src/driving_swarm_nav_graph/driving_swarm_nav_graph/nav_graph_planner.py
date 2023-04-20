import rclpy

from driving_swarm_nav_graph.nav_graph import NavGraphNode
from trajectory_generator.vehicle_model_node import (
    TrajectoryGenerator,
    Vehicle,
)

import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32
from driving_swarm_messages.srv import UpdateTrajectory
from std_srvs.srv import Empty
import polygonal_roadmaps as poro
import yaml


class NavGraphLocalPlanner(NavGraphNode):
    def __init__(self):
        super().__init__('local_planner')
        self.get_logger().info("Starting")
        self.get_frames()
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
            step=0.1,
            r=self.get_parameter("turn_radius")
            .get_parameter_value()
            .double_value,
            r_step=1.0,
        )

        self.setup_tf()

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
        self.cell_publisher = self.create_publisher(Int32, "nav/cell", 1)
        self.create_service(Empty, "nav/replan", self.replan_callback)
        self.create_subscription(String, "nav/plan", self.plan_cb, 1)
        self.follow_client.wait_for_service()
        self.get_logger().info("connected to trajectory follower service")
        self.wait_for_tf()

        self.status_pub.publish(String(data="ready"))
        self.create_timer(1.0, self.timer_cb)
    
    
    def timer_cb(self):
        pose = self.get_pose()
        if pose is None:
            return
        
        node = poro.geometry.find_nearest_node(self.env.g, (pose[0], pose[1]))
        if not node or node == 0:
            self.get_logger().warn(f'node: {node}')
            self.get_logger().warn(f'node found is {self.env.g.nodes()[0]["geometry"].center}')
            self.get_logger().warn(f'robot is located at {pose[0]}, {pose[1]}')
            return
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
            path.poses.append(self.tuple_to_pose_stamped_msg(pose))

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
        pose = self.get_pose()
        if pose is None:
            return

        waypoints = self.gen_rtr_path(pose)

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
        poly = poro.geometry.poly_from_path(self.env.g, self.plan) 
        # set goal from final node in path
        goal = self.env.g.nodes()[self.plan[-1]]['geometry'].inner.centroid.coords[0]
        path = poro.geometry.waypoints_through_poly(self.env.g, poly, start[:2], goal, eps=0.01)
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
