# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
from std_msgs.msg import String
import tf2_ros
# from tf2_py import *
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from driving_swarm_utils.node import DrivingSwarmNode


class LocalTFPub(DrivingSwarmNode):
    def __init__(self):
        # node
        super().__init__('local_tf_pub')

        # params
        self.declare_parameter('robot_name', "invalid_robot_name")
        self.robot_name = self.get_parameter('robot_name').value
        if self.robot_name == "invalid_robot_name":
            self.get_logger().warn("robot_name is not set")
        self.get_own_frame()
        self.reference_frame = 'world'
        # we do not need the reference frame, as we assume world -> map -> ... -> own_frame exists
        # but we want to reference to world in this case

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.publisher = self.create_publisher(TFMessage, 'tf_global', 100)
        f = self.tfBuffer.wait_for_transform_async(
            self.reference_frame, self.own_frame, rclpy.time.Time().to_msg()
        )
        rclpy.spin_until_future_complete(self, f)
        TIMER_PERIOD = 0.1  # seconds
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        #self.br = tf2_ros.TransformBroadcaster(self)

    def timer_callback(self):
        tf2Msg = TransformStamped()

        try:
            tf2Msg = self.tfBuffer.lookup_transform(
                "world", self.base_frame, rclpy.time.Time())
            tf2Msg.child_frame_id = self.robot_name
            msg = TFMessage()
            msg.transforms = [tf2Msg]
            self.publisher.publish(msg)
            #self.br.sendTransform(tf2Msg)
        except Exception as e:
            self.get_logger().info(str(type(e)), once=True)


def main(args=None):
    rclpy.init(args=args)
    node = LocalTFPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'got keyboard interrupt, shutting down')
        node.destroy_node()


if __name__ == '__main__':
    main()
