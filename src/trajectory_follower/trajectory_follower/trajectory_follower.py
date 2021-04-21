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
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, Quaternion
from nav_msgs.msg import Path
from driving_swarm_messages.srv import UpdateTrajectory
from std_srvs.srv import Empty


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.get_logger().info('Starting trajectory follower')
        self.reference_frame = 'base_link'
        self.trajectory = None
        self.current_goal = None
        self.ix = 0
        self.iy = 0
        self.itheta = 0
        self.name = self.get_namespace()[1:]

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.cmd_vel = Twist()

        # todo: make this a configurable parameter
        self.fail_radius = 0.3
        
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 9)
        self.desired_pose_publisher = self.create_publisher(PoseStamped, 'nav/desired', 9)
        self.co0 = self.create_publisher(PoseStamped, 'nav/cutoff_0', 9)
        self.co1 = self.create_publisher(PoseStamped, 'nav/cutoff_1', 9)
        self.path_publisher = self.create_publisher(Path, 'nav/trajectory', 9)
        self.create_service(UpdateTrajectory, 'nav/follow_trajectory', self.update_trajectory_cb)
        self.replan_client = self.create_client(Empty, 'nav/replan')

        self.create_timer(0.1, self.timer_cb)

    def update_trajectory_cb(self, request, response):
        # todo: check if trajectories submitted are valid
        if self.trajectory is None or request.update_index == 0:
            self.trajectory = request.trajectory
        else:
            self.co0.publish(self.trajectory.poses[request.update_index - 1])
            self.co1.publish(request.trajectory.poses[0])
            self.trajectory.poses = self.trajectory.poses[:request.update_index] + request.trajectory.poses
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
        q.x, q.y, q.z, q.w = PyKDL.Rotation.RPY(.0, .0, pose2d.theta).GetQuaternion()
        pose_stamped.pose.orientation = q
        return pose_stamped

    def poseStamped_to_Pose2D(self, pose_stamped):
        pose2d = Pose2D()
        try:
            # get the transform so we can read at wich time it was performed
            t = self.tfBuffer.lookup_transform(pose_stamped.header.frame_id, self.reference_frame, rclpy.time.Time().to_msg())
            # set the time to the most recent transform
            pose_stamped.header.stamp = t.header.stamp
            pose3d = self.tfBuffer.transform(pose_stamped, self.reference_frame)
            pose2d.x = pose3d.pose.position.x
            pose2d.y = pose3d.pose.position.y
            q = pose3d.pose.orientation
            pose2d.theta = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w).GetRPY()[2]
            
        except Exception as e:
            self.get_logger().warn(f'could not transform pose to reference frame \n {e}')
        return pose2d
 
    def get_diff(self, offset=0):
        ego_pose = self.get_current_ego_pose() 
        desired_pose, desired_vel = self.get_desired_pose_vel(offset=offset)
        x_diff = desired_pose.x - ego_pose.x
        y_diff = desired_pose.y - ego_pose.y 
        theta_diff = desired_pose.theta - ego_pose.theta
        return Pose2D(x=x_diff, y=y_diff, theta=theta_diff), desired_vel
    
    def set_trajectory_fail(self):
        self.get_logger().info('canceling goal because to far from planned pose')
        self.trajectory = None
        request = Empty.Request()
        self.replan_client.call_async(request)
        self.cmd_vel = Twist()
        self.cmd_publisher.publish(self.cmd_vel)
        
    def compute_lookahead_output(self, dt=0.5):
        diff_pose, desired_vel = self.get_diff(offset=0)
        if diff_pose.x**2 + diff_pose.y**2 > self.fail_radius**2:
            return None
        diff_pose, desired_vel = self.get_diff(offset=dt)
        
        vel = Twist()
        vel.angular.z = diff_pose.theta / dt
        # linear approximation
        vel.linear.x = np.linalg.norm(np.array([diff_pose.x, diff_pose.y]))
        vel.linear.x /= dt * np.sign(diff_pose.x)
        
        # todo: x ist approximiert durch die gerade, sollte aber eigentlich länge des kreisbogen sein
        # d = np.linalg.norm(np.array([diff_pose.x, diff_pose.y]))
        # r = 0.5 * d / (np.sin(diff_pose.theta / 2))
        # vel.linear.x = (0.5 * r * diff_pose.theta / np.pi) / dt
        return vel

    def compute_control_output(self):
        diff_pose, desired_vel = self.get_diff(offset=0)
        if diff_pose.x**2 + diff_pose.y**2 > self.fail_radius**2:
            return None
        self.ix += diff_pose.x
        self.iy += diff_pose.y
        self.itheta = diff_pose.theta
        
        px = 1.5
        py = 1.5
        p_theta = 1.5
        
        pi_x = 0.1
        pi_y = 0.1
        pi_theta = 0.1
        
        output = Twist()
        output.linear.x = desired_vel.linear.x + diff_pose.x * px + pi_x * self.ix

        output.angular.z = desired_vel.angular.z
        output.angular.z += diff_pose.y * py
        output.angular.z += pi_y * self.iy
        output.angular.z += diff_pose.theta * p_theta
        output.angular.z += pi_theta * self.itheta
        return output
    
    def set_cmd_vel(self, control):
        # todo: respect limits (acceleration + max vel)
        self.cmd_vel = control
    
    def timer_cb(self):
        # when no trajectory is given, don't do anything
        if self.trajectory is None:
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
            return

        # compute and publish control output
        control = self.compute_lookahead_output(dt=1.0)
        if control is None:
            self.set_trajectory_fail()
            return
        
        self.set_cmd_vel(control)
        self.cmd_publisher.publish(self.cmd_vel)

        # publish debug info 
        self.desired_pose_publisher.publish(self.pose2D_to_PoseStamped(
            self.get_desired_pose_vel()[0], header_id=self.name)
        )
        self.path_publisher.publish(self.trajectory)

    def get_current_ego_pose(self):
        # this works because reference frame == ego frame
        return Pose2D(x=0.0, y=0.0, theta=0.0)

    def get_desired_pose_vel(self, offset=0):
        # get last pose, next pose and dt
        NANO = 0.001 ** 3
        current_stamp = self.get_clock().now()
        trajectory_start = rclpy.time.Time.from_msg(self.trajectory.header.stamp)
        rate = 1.0
        t = ((current_stamp - trajectory_start).nanoseconds * NANO + offset) * rate
        if t < 0.0:
            self.get_logger().warn('trajectory start is in the future')
            t = 0.0
        last_index = int(np.floor(t))
        if last_index+2 >= len(self.trajectory.poses):
            self.trajectory = None
            return Pose2D(), Twist()
        dt = (t - np.floor(t))
        last_pose = self.poseStamped_to_Pose2D(self.trajectory.poses[last_index])
        next_pose = self.poseStamped_to_Pose2D(self.trajectory.poses[last_index + 1])
        x = (1 - dt) * last_pose.x + dt * next_pose.x
        y = (1 - dt) * last_pose.y + dt * next_pose.y
        theta = (1 - dt) * last_pose.theta + dt * next_pose.theta
        vel = Twist()
        vel.linear.x = (next_pose.x - last_pose.x) * rate
        vel.angular.z = (next_pose.theta - last_pose.theta) * rate
        return Pose2D(x=x, y=y, theta=theta), vel


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
