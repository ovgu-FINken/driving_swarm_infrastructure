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
import tf2_ros # noqa
from tf2_msgs.msg import TFMessage


class LocalToGlobalTFPub(Node):
    def __init__(self):
        # node
        super().__init__('local_to_global_tf_pub')

        # params
        self.declare_parameter('robot_name')
        self.robot_name = self.get_parameter('robot_name').value
        self.publisher_ = self.create_publisher(TFMessage, "/tf", 100)
        self.create_subscription(
            TFMessage, 'tf', self.listener_callback, 100)

    def listener_callback(self, msg):
        if msg.transforms[0].child_frame_id == self.robot_name:
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalToGlobalTFPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'got keyboard interrupt, shutting down')
        node.destroy_node()


if __name__ == '__main__':
    main()
