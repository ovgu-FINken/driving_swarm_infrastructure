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
from driving_swarm_messages.srv import UpdateTrajectory
from std_srvs.srv import Empty


# for the action server seehttps://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_queue_goals.py
# as a reference.


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__("trajectory_follower")
        self.get_logger().info("Starting trajectory follower")
        self.reference_frame = "base_link"
        self.trajectory = None
        self.current_goal = None
        self.ix = 0
        self.iy = 0
        self.itheta = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.cmd_vel = Twist()

        # todo: eventually make this a configurable parameter
        self.fail_radius = 0.3

        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 9)
        self.desired_pose_publisher = self.create_publisher(
            PoseStamped, "nav/desired", 9
        )
        self.path_publisher = self.create_publisher(Path, "nav/trajectory", 9)
        self.create_service(
            UpdateTrajectory,
            "nav/follow_trajectory",
            self.update_trajectory_cb,
        )
        self.replan_client = self.create_client(Empty, "nav/replan")

        self.create_timer(0.1, self.timer_cb)

    def update_trajectory_cb(self, request, response):
        # todo: check if trajectories submitted are valid
        if self.trajectory is None or request.update_index == 0:
            self.trajectory = request.trajectory
        else:
            self.trajectory.poses = (
                self.trajectory.poses[: request.update_index]
                + request.trajectory.poses
            )
        response.accepted = True
        response.trajectory = self.trajectory
        return response

    def pose2D_to_PoseStamped(self, pose2d):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.reference_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = pose2d.x
        pose_stamped.pose.position.y = pose2d.y
        pose_stamped.pose.position.z = 0.0
        q = Quaternion()
        q.x, q.y, q.z, q.w = PyKDL.Rotation.RPY(
            0.0, 0.0, pose2d.theta
        ).GetQuaternion()
        pose_stamped.pose.orientation = q
        return pose_stamped

    def poseStamped_to_Pose2D(self, pose_stamped):
        pose2d = Pose2D()
        try:
            # get the transform so we can read at wich time it was performed
            t = self.tfBuffer.lookup_transform(
                pose_stamped.header.frame_id,
                self.reference_frame,
                rclpy.time.Time().to_msg(),
            )
            # set the time to the most recent transform
            pose_stamped.header.stamp = t.header.stamp
            pose3d = self.tfBuffer.transform(
                pose_stamped, self.reference_frame
            )
            pose2d.x = pose3d.pose.position.x
            pose2d.y = pose3d.pose.position.y
            q = pose3d.pose.orientation
            pose2d.theta = PyKDL.Rotation.Quaternion(
                q.x, q.y, q.z, q.w
            ).GetRPY()[2]

        except Exception as e:
            self.get_logger().warn(
                f"could not transform pose to reference frame \n {e}"
            )
        return pose2d

    def timer_cb(self):
        if self.trajectory is None:
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
            return

        ego_pose = self.get_current_ego_pose()
        desired_pose, desired_vel = self.get_current_desired_pose_vel()
        x_diff = desired_pose.x - ego_pose.x
        y_diff = desired_pose.y - ego_pose.y
        theta_diff = desired_pose.theta - ego_pose.theta
        if x_diff ** 2 + y_diff ** 2 > self.fail_radius ** 2:
            self.get_logger().warn(
                "-----------------------------------------------------"
            )
            self.get_logger().warn(
                "canceling trajectory because to far from planned pose"
            )
            self.get_logger().warn(
                "-----------------------------------------------------"
            )
            self.trajectory = None
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
            request = Empty.Request()
            self.replan_client.call_async(request)

        self.ix += x_diff
        self.iy += y_diff
        self.itheta = theta_diff

        px = 1.5
        py = 1.5
        p_theta = 1.5

        pi_x = 0.1
        pi_y = 0.1
        pi_theta = 0.1

        self.cmd_vel.linear.x = (
            desired_vel.linear.x + x_diff * px + pi_x * self.ix
        )
        self.cmd_vel.angular.z = (
            desired_vel.angular.z
            + y_diff * py
            + pi_y * self.iy
            + theta_diff * p_theta
            + pi_theta * self.itheta
        )

        if self.trajectory is None:
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
            return
        else:
            self.desired_pose_publisher.publish(
                self.pose2D_to_PoseStamped(desired_pose)
            )
            self.path_publisher.publish(self.trajectory)
            self.cmd_publisher.publish(self.cmd_vel)

    def get_current_ego_pose(self):
        # this works because reference frame == ego frame
        return Pose2D(x=0.0, y=0.0, theta=0.0)

    def get_current_desired_pose_vel(self):
        # get last pose, next pose and dt
        NANO = 0.001 ** 3
        current_stamp = self.get_clock().now()
        trajectory_start = rclpy.time.Time.from_msg(
            self.trajectory.header.stamp
        )
        rate = 1.0
        t = (current_stamp - trajectory_start).nanoseconds * NANO * rate
        if t < 0.0:
            self.get_logger().warn("trajectory start is in the future")
            t = 0.0

        last_index = int(np.floor(t))
        if last_index + 2 >= len(self.trajectory.poses):
            self.trajectory = None
            return Pose2D(), Twist()
        dt = t - np.floor(t)
        # self.get_logger().info(f't={t / rate}, dt={dt}, index: {last_index}')

        last_pose = self.poseStamped_to_Pose2D(
            self.trajectory.poses[last_index]
        )
        next_pose = self.poseStamped_to_Pose2D(
            self.trajectory.poses[last_index + 1]
        )
        # self.get_logger().info(f'last: {last_pose}')
        # self.get_logger().info(f'next: {next_pose}')

        x = (1 - dt) * last_pose.x + dt * next_pose.x
        y = (1 - dt) * last_pose.y + dt * next_pose.y
        theta = (1 - dt) * last_pose.theta + dt * next_pose.theta

        vel = Twist()
        vel.linear.x = (next_pose.x - last_pose.x) * rate * 2
        vel.angular.z = (next_pose.theta - last_pose.theta) * rate
        # self.get_logger().info(f'linear vel setpoint={vel}')
        return Pose2D(x=x, y=y, theta=theta), vel


def main():
    rclpy.init()
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down after user interrupt")
    node.destroy_node()


if __name__ == "__main__":
    main()
