import rclpy
from rclpy.node import Node
from driving_swarm_messages.msg import SystemStatus
import psutil
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos


class Watchdog(Node):
    def __init__(self):
        super().__init__("watchdog")
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.vel_callback, qos.qos_profile_system_default
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        WATCH_RATE = 0.3  # s
        self.timer = self.create_timer(WATCH_RATE, self.watch_callback)
        self.latest_timestamp_vel = 0
        self.latest_timestamp_wd = 0
        self.first_cb = True

    def vel_callback(self, msg):
        if self.first_cb:
            self.latest_timestamp_wd = self.get_clock().now().nanoseconds
            self.first_cb = False
        else:
            self.latest_timestamp_vel = self.get_clock().now().nanoseconds

    def watch_callback(self):
        # TODO: cover the case where the cmd_vel was only send once, i.e. latest_timestamp_vel is still 0 --> maybe with a timer
        if (not self.latest_timestamp_vel == 0) and (
            not self.latest_timestamp_wd == 0
        ):
            if self.latest_timestamp_vel == self.latest_timestamp_wd:
                self.publish_stop_cmd()
            self.latest_timestamp_wd = self.latest_timestamp_vel

    def publish_stop_cmd(self):
        angular = Vector3(x=0.0, y=0.0, z=0.0)
        linear = Vector3(x=0.0, y=0.0, z=0.0)
        msg = Twist(linear=linear, angular=angular)
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Stopping robot due to missed control loop.")


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    # main loop
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
