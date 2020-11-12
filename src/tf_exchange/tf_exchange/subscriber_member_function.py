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
from tf2_msgs.msg import TFMessage
import subprocess
import re


def get_ip():
    ipa = subprocess.check_output(['ip', 'a']).decode('ascii')
    ips = re.findall(r'inet\s[0-9.]+', ipa)
    ips = [ip[5:] for ip in ips]
    # TODO get only the last 3 digits of the ip
    return ", ".join(ips)


class LocalToGlobalTFPub(Node):
    def __init__(self):
        super().__init__('local_to_global_tf_pub')
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, Node)
        self.robotName = "TB" + get_ip()
        self.get_logger().log(self.robotName)
        tfTopic = self.robotName + "/tf"
        self.publisher_ = self.create_publisher(TFMessage, tfTopic, 10)
        TIMER_PERIOD = 0.5  # seconds
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def timer_callback(self):
        tf2Msg = TFMessage()
        try:
            tf2Msg = self.tfBuffer.lookup_transform(
                "world", "base_link", self.get_clock().now())
        except:  # TODO where are the exception objects
            Rate.sleep(self)
        tf2Msg.transforms.child_frame_id = "TB" + self.robotName
        self.publisher_.publish(tf2Msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = LocalToGlobalTFPub()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
