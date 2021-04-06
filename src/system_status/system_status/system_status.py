import rclpy
from rclpy.node import Node
from driving_swarm_messages.msg import SystemStatus
from datetime import datetime
import psutil
import subprocess
import re
from system_status import utils


class StatusPublisher(Node):
    def __init__(self):
        super().__init__("status_publisher")
        self.pub = self.create_publisher(SystemStatus, "/system_status", 10)
        self.timer = self.create_timer(10.0, self.timer_cb)
        self.ip = utils.get_ip()

    def timer_cb(self):
        msg = SystemStatus()
        msg.ip = self.ip
        msg.time = str(datetime.now())
        msg.load = psutil.cpu_percent()
        msg.memory = psutil.virtual_memory().percent
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    # main loop
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
