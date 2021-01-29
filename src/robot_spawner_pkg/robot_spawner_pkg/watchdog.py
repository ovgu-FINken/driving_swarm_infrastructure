import rclpy
from rclpy.node import Node
from driving_swarm_messages.msg import SystemStatus
from datetime import datetime
import psutil
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos


class Watchdog(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.store_vel_callback, qos.qos_profile_system_default)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        WATCH_RATE = 0.2  # s
        self.timer = self.create_timer(WATCH_RATE, self.watch_callback)
        self.latest_timestamp = 0
        self.previous_timestamp = 0
        self.first_cb = True

    def store_vel_callback(self, msg):
        if self.first_cb:
            self.previous_timestamp = self.get_clock().now().nanoseconds
            self.first_cb = False
        else:
            self.latest_timestamp = self.get_clock().now().nanoseconds

    def watch_callback(self):
        if (not self.latest_timestamp == 0) and (not self.previous_timestamp == 0):
            if self.latest_timestamp == self.previous_timestamp:
                self.publish_stop_cmd()

    def publish_stop_cmd(self):
        angular = Vector3(x=0.0, y=0.0, z=0.0)
        linear = Vector3(x=0.0, y=0.0, z=0.0)
        msg = Twist(linear=linear, angular=angular)
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    # main loop
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
