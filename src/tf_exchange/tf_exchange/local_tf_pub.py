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


class LocalTFPub(Node):
    def __init__(self):
        # node
        super().__init__('local_tf_pub')

        # params
        self.declare_parameter('robot_name')
        self.robot_name = self.get_parameter('robot_name').value
        self.declare_parameter('base_frame')
        self.base_frame = self.get_parameter('base_frame').value

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        TIMER_PERIOD = 0.1  # seconds
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def timer_callback(self):
        br = tf2_ros.TransformBroadcaster(self)
        tf2Msg = TransformStamped()

        try:
            tf2Msg = self.tfBuffer.lookup_transform(
                "world", self.base_frame, rclpy.time.Time())
            tf2Msg.child_frame_id = self.robot_name
            br.sendTransform(tf2Msg)

        # except (LookupException, ExtrapolationException, ConnectivityException) as e:
        #     self.get_logger().info(str(e))
        except Exception as e:
            # self.get_logger().info(str(e))
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
