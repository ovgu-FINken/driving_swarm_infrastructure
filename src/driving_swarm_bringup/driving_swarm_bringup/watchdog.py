import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos

class Watchdog(Node):
    def __init__(self):
        super().__init__('watchdog')
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.vel_callback, qos.qos_profile_system_default)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos.qos_profile_system_default)
        WATCH_RATE = 2.0
        self.timer = self.create_timer(WATCH_RATE, self.watch_callback)
        self.cmd_vel_received = False

    def vel_callback(self, msg):
        self.cmd_vel_received = True

    def watch_callback(self):
        if not self.cmd_vel_received:
            self.publish_stop_cmd()
        self.cmd_vel_received = False

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


if __name__ == '__main__':
    main()
