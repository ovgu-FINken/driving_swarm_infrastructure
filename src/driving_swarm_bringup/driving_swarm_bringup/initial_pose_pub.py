# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Script used to spawn a robot in a generic position."""

import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_srvs.srv import Empty
from lifecycle_msgs.srv import GetState
import tf_transformations
import time


class Spawner(Node):
    def __init__(self, args):
        super().__init__(f'spawner_{args.robot_name}')
        self.args = args
        topic = f'{args.robot_namespace}/initialpose'
        self.pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.srv = self.create_client(Empty, f'{args.robot_namespace}/request_nomotion_update')

        self.initial_pose = Pose()
        self.initial_pose.position.x = args.x
        self.initial_pose.position.y = args.y
        self.initial_pose.position.z = args.z
        rot = tf_transformations.quaternion_from_euler(.0, .0, args.yaw)
        self.initial_pose.orientation.x = rot[0]
        self.initial_pose.orientation.y = rot[1]
        self.initial_pose.orientation.z = rot[2]
        self.initial_pose.orientation.w = rot[3]

    def wait_for_localization(self):
        request = GetState.Request()
        topic = f'/{self.args.robot_name}/amcl/get_state'
        client = self.create_client(GetState, topic)
        if not client.service_is_ready():
            self.get_logger().info(f'waiting for service {topic}')
            client.wait_for_service()
            self.get_logger().info(f'connected to state service')

        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                print('response: %r' % future.result())
                self.get_logger().info(f'{future.result()}')
                if future.result().current_state.id == 3:
                    break
            else:
                raise RuntimeError(
                    'exception while calling service: %r' % future.exception())
        self.send_initial_pose()
        self.request_no_motion_update()
        rclpy.shutdown()

    def request_no_motion_update(self):
        self.get_logger().info("sending nomotion update request")
        self.srv.wait_for_service()
        future = self.srv.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("nomotion update request completed")
        


    def send_initial_pose(self):
        # Send initial pose
        time.sleep(1.0)
        self.get_logger().info('Sending initial pose')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose = self.initial_pose
        self.pub.publish(pose)

        self.get_logger().info('Done! Shutting down self.')


def main():
    parser = argparse.ArgumentParser(
        description='Spawn Robot into Gazebo with navigation2')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0.0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0.0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0.0,
                        help='the z component of the initial position [meters]')

    parser.add_argument('-yaw', type=float, default=0.0,
                        help='rotation of the initial position in [rad]')

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-t', '--turtlebot_type', type=str,
                       choices=['waffle', 'burger'])
    group.add_argument('-s', '--sdf', type=str,
                       help="the path to the robot's model file (sdf)")

    args, _ = parser.parse_known_args()
    print("-----------------LAUNCH ARGUMENTS--------------")
    print(args)
    print("-------------------------------")

    rclpy.init()
    node = Spawner(args)
    node.wait_for_localization()
    node.destroy_node()


if __name__ == '__main__':
    main()
