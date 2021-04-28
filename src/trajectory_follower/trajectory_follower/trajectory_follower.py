#!/usr/bin/env python
import rclpy
import PyKDL

# modules need to be imported as plugins for tf2
import tf2_ros
import tf2_kdl # noqa F401
import tf2_py # noqa F401
import tf2_geometry_msgs # noqa F401
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, Quaternion
from nav_msgs.msg import Path
from driving_swarm_messages.srv import UpdateTrajectory
from std_srvs.srv import Empty


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.get_logger().info('Starting trajectory follower')
        self.reference_frame = 'base_link'
        self.declare_parameter("dt")
        self.dt = self.get_parameter("dt").get_parameter_value().double_value
        self.declare_parameter("w1")
        self.w1 = self.get_parameter("w1").get_parameter_value().double_value
        self.declare_parameter("w2")
        self.w2 = self.get_parameter("w2").get_parameter_value().double_value
        self.declare_parameter("fail_radius")
        self.fail_radius = self.get_parameter("fail_radius") \
                               .get_parameter_value().double_value
        self.trajectory = None
        self.current_goal = None
        self.ix = 0
        self.iy = 0
        self.itheta = 0
        self.name = self.get_namespace()[1:]

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.cmd_vel = Twist()
        self.max_accel_x = 0.025
        self.max_accel_z = 0.032
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 9)
        self.desired_pose_publisher = \
            self.create_publisher(PoseStamped, 'nav/desired', 9)
        self.co0 = self.create_publisher(PoseStamped, 'nav/cutoff_0', 9)
        self.co1 = self.create_publisher(PoseStamped, 'nav/cutoff_1', 9)
        self.path_publisher = self.create_publisher(Path, 'nav/trajectory', 9)
        self.create_service(
            UpdateTrajectory,
            'nav/follow_trajectory',
            self.update_trajectory_cb
        )
        self.replan_client = self.create_client(Empty, 'nav/replan')

        self.create_timer(0.1, self.timer_cb)

    def update_trajectory_cb(self, request, response):
        # todo: check if trajectories submitted are valid
        if self.trajectory is None or request.update_index == 0:
            self.trajectory = request.trajectory
        else:
            self.co0.publish(self.trajectory.poses[request.update_index - 1])
            self.co1.publish(request.trajectory.poses[0])
            self.trajectory.poses = \
                self.trajectory.poses[:request.update_index] + \
                request.trajectory.poses
        response.accepted = True
        response.trajectory = self.trajectory
        return response

    def pose2D_to_PoseStamped(self, pose2d, header_id=None):
        pose_stamped = PoseStamped()
        if header_id is None:
            pose_stamped.header.frame_id = self.reference_frame
        else:
            pose_stamped.header.frame_id = header_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = pose2d.x
        pose_stamped.pose.position.y = pose2d.y
        pose_stamped.pose.position.z = 0.0
        q = Quaternion()
        q.x, q.y, q.z, q.w = \
            PyKDL.Rotation.RPY(.0, .0, pose2d.theta).GetQuaternion()
        pose_stamped.pose.orientation = q
        return pose_stamped

    def poseStamped_to_Pose2D(self, pose_stamped):
        pose2d = Pose2D()
        try:
            # get the transform so we can read at wich time it was performed
            t = self.tfBuffer.lookup_transform(
                pose_stamped.header.frame_id,
                self.reference_frame,
                rclpy.time.Time().to_msg()
            )
            # set the time to the most recent transform
            pose_stamped.header.stamp = t.header.stamp
            pose3d = self.tfBuffer.transform(
                pose_stamped,
                self.reference_frame
            )
            pose2d.x = pose3d.pose.position.x
            pose2d.y = pose3d.pose.position.y
            q = pose3d.pose.orientation
            pose2d.theta = \
                PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w).GetRPY()[2]
        except Exception as e:
            self.get_logger().warn(
                'could not transform pose' +
                f' in frame {pose_stamped.header.frame_id}' +
                f' to reference frame {self.reference_frame} \n {e}'
            )
        return pose2d
 
    def get_target_pose(self, offset: float = 0.0) -> Pose2D:
        """ get the target poset at time t + offset in ego coordinates
        """
        return self.get_desired_pose(offset=offset)
    
    def set_trajectory_fail(self):
        self.get_logger().info(
            'canceling goal because to far from planned pose'
        )
        self.trajectory = None
        request = Empty.Request()
        self.replan_client.call_async(request)
        self.cmd_vel = Twist()
        self.cmd_publisher.publish(self.cmd_vel)
        
    def compute_pure_pursuit_output(self) -> Twist:
        if self.trajectory is None:
            return None
        # get the target pose at the correct time in ego_coordinates
        diff_pose = self.get_target_pose(offset=self.dt)
        vel = Twist()
        vel.linear.x = diff_pose.x / self.dt
        # dy is for path deviation
        dy = 0
        # no division by 0.0
        if diff_pose.x != 0.0:
            dy = np.arctan(diff_pose.y / diff_pose.x) / self.dt
        # dtheta is for angular deviation
        dtheta = diff_pose.theta / self.dt
        vel.angular.z = self.w1 * dy + self.w2 * dtheta
        return vel
    
    def set_cmd_vel(self, control):
        # todo: respect limits (acceleration + max vel)
        # self.cmd_vel = control
        self.cmd_vel.linear.x = np.clip(
            control.linear.x,
            self.cmd_vel.linear.x - self.max_accel_x,
            self.cmd_vel.linear.x + self.max_accel_x
        )
        self.cmd_vel.angular.z = np.clip(
            control.angular.z,
            self.cmd_vel.angular.z - self.max_accel_z,
            self.cmd_vel.angular.z + self.max_accel_z
        )
    
    def timer_cb(self):
        # when no trajectory is given, don't do anything
        if self.trajectory is None:
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
            return

        diff_pose = self.get_target_pose(offset=0.0)
        if diff_pose.x**2 + diff_pose.y**2 > self.fail_radius**2:
            self.set_trajectory_fail()
            return

        # compute and publish control output
        control = self.compute_pure_pursuit_output()
        if control is None:
            self.set_trajectory_fail()
            return
        
        self.set_cmd_vel(control)
        self.cmd_publisher.publish(self.cmd_vel)

        # publish topics for vizual debugging and analysis
        if self.trajectory is None:
            return None
        self.desired_pose_publisher.publish(
            self.pose2D_to_PoseStamped(
                self.get_desired_pose(), header_id=self.name
            )
        )
        self.path_publisher.publish(self.trajectory)

    def get_desired_pose(self, offset: float = 0.0) -> Pose2D:
        # get last pose, next pose and dt
        current_stamp = self.get_clock().now()
        trajectory_start = \
            rclpy.time.Time.from_msg(self.trajectory.header.stamp)
        rate = 1.0
        t = ((current_stamp - trajectory_start).nanoseconds * 1e-9 + offset)
        t *= rate
        if t < 0.0:
            self.get_logger().warn('trajectory start is in the future')
            t = 0.0
        last_index = int(np.floor(t))
        if last_index+2 >= len(self.trajectory.poses):
            return self.poseStamped_to_Pose2D(self.trajectory.poses[-1])
        dt = (t - np.floor(t))
        last_pose = self.poseStamped_to_Pose2D(
            self.trajectory.poses[last_index]
        )
        next_pose = self.poseStamped_to_Pose2D(
            self.trajectory.poses[last_index + 1]
        )
        x = (1 - dt) * last_pose.x + dt * next_pose.x
        y = (1 - dt) * last_pose.y + dt * next_pose.y
        theta = (1 - dt) * last_pose.theta + dt * next_pose.theta
        return Pose2D(x=x, y=y, theta=theta)


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
