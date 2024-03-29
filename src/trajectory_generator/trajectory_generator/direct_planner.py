#!/usr/bin/env python

import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from trajectory_generator.utils import yaw_from_orientation
from trajectory_generator.vehicle_model_node import (
    TrajectoryGenerator,
    Vehicle,
)
from driving_swarm_messages.srv import UpdateTrajectory
from std_srvs.srv import Empty
from driving_swarm_utils.node import DrivingSwarmNode
from termcolor import colored


class DirectPlanner(DrivingSwarmNode):
    def __init__(self):
        super().__init__("direct_planner")
        self.goal = None
        self.started = False
        self.current_trajectory = None

        self.declare_parameter("vehicle_model", 3)
        self.declare_parameter("step_size", 0.1)
        self.declare_parameter("turn_radius", 0.3)
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
        )

        self.get_frames()
        self.setup_tf()
        self.setup_command_interface()

        self.follow_action_client = self.create_client(
            UpdateTrajectory, "nav/follow_trajectory"
        )
        self.create_service(Empty, "nav/replan", self.replan_callback)
        self.follow_action_client.wait_for_service()
        self.get_logger().info("connected to trajectory follower service")
        self.wait_for_tf()
        self.set_state_ready()
        self.create_subscription(PoseStamped, "nav/goal", self.goal_cb, 1)
        self.create_timer(0.1, self.timer_cb)

    def set_goal(self, goal):
        self.goal_started = False
        self.goal = goal

    def goal_cb(self, msg):
        if self.goal is None:
            self.set_goal(msg)
            self.get_logger().info("got goal")
            self.status_pub.publish(String(data="ready"))
            self.get_logger().info("ready")
        elif self.goal.pose != msg.pose:
            self.get_logger().info("got new goal")
            self.set_goal(msg)

    def timer_cb(self):
        if not self.started:
            return
        if self.goal is None:
            return
        if not self.goal_started:
            ti, path = self.create_path()
            self.send_path(path, ti=ti)
            self.goal_started = True

    def replan_callback(self, _, response):
        self.current_trajectory = None
        self.set_goal(self.goal)
        return response

    def create_path(self):
        waypoint_tuples, ti = self.get_waypoints()
        if waypoint_tuples[0] is None:
            return

        return ti, self.vm.tuples_to_path(waypoint_tuples)

    def send_path(self, trajectory, ti=0):
        # convert trajectory to correct space
        if not trajectory:
            self.get_logger().warn(colored("empty trajectory", "red"))
        path = Path()
        path.header.frame_id = self.reference_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.tuple_to_pose_stamped_msg(*pose) for pose in trajectory]
        
        self.get_logger().info(colored("sending path", "green"))
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=ti)
        self.follow_action_client.call_async(request)

    def get_waypoints(self):
        start = None
        ti = 0
        if self.current_trajectory is not None:
            ti = self.get_trajectory_index(self.current_trajectory, offset=0.3)

        if ti is None or ti == 0:
            ti = 0
            start = self.get_tf_pose()
        else:
            start = (
                self.current_trajectory.poses[ti].pose.position.x,
                self.current_trajectory.poses[ti].pose.position.y,
                yaw_from_orientation(
                    self.current_trajectory.poses[ti].pose.orientation
                ),
            )

        goal = (
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            yaw_from_orientation(self.goal.pose.orientation),
        )

        return [start, goal], ti

    def get_trajectory_index(self, trajectory, offset=0.0):
        NANO = 0.001 ** 3
        current_stamp = self.get_clock().now() + rclpy.time.Duration(
            seconds=offset
        )
        trajectory_start = rclpy.time.Time.from_msg(
            self.current_trajectory.header.stamp
        )
        rate = 1.0
        t = ((current_stamp - trajectory_start).nanoseconds * NANO) * rate
        if t < 0.0:
            self.get_logger().warn("trajectory start is in the future")
            t = 0.0

        last_index = int(np.floor(t))
        if last_index + 2 >= len(self.current_trajectory.poses):
            return None
        return last_index


def main():
    rclpy.init()
    node = DirectPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting Down")
        node.destroy_node()


if __name__ == "__main__":
    main()
