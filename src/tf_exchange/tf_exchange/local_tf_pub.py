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
import subprocess
import re
from datetime import datetime
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile


def get_ip_name():
    ipa = subprocess.check_output(['ip', 'a']).decode('ascii')
    ips = re.findall(r'inet\s[0-9.]+', ipa)
    ips = [ip[5:] for ip in ips]
    x = [ip.split('.') for ip in ips]
    name = 'T'
    for ip in x:
        if ip[0] != '127':
            name += ip[-1]
    return name


class LocalTFPub(Node):
    def __init__(self):
        # node
        super().__init__('local_tf_pub')

        # params
        # self.declare_parameter('robot_name', get_ip_name())
        self.declare_parameter('robot_name')
        self.robot_name = self.get_parameter('robot_name').value

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        # qos = QoSProfile(depth=100)
        # self.pub_tf = self.create_publisher(TFMessage, "/tf", qos)
        TIMER_PERIOD = 0.5  # seconds
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def timer_callback(self):
        br = tf2_ros.TransformBroadcaster(self)
        tf2Msg = TransformStamped()

        try:
            tf2Msg = self.tfBuffer.lookup_transform(
                "world", "base_link", rclpy.time.Time())
            tf2Msg.child_frame_id = self.robot_name
            br.sendTransform(tf2Msg)

            # # maybe the tranformation can directly be published to /tf?
            # if not isinstance(tf2Msg, list):
            #     if hasattr(tf2Msg, '__iter__'):
            #         tf2Msg = list(tf2Msg)
            #     else:
            #         tf2Msg = [tf2Msg]
            # self.pub_tf.publish(TFMessage(transforms=tf2Msg))

        # except (LookupException, ExtrapolationException, ConnectivityException) as e:
        #     self.get_logger().info(str(e))
        except Exception as e:
            self.get_logger().info(str(e))
            self.get_logger().info(str(type(e)))


def main(args=None):
    rclpy.init(args=args)
    node = LocalTFPub()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
