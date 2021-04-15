import rclpy
from rclpy.node import Node
from driving_swarm_messages.msg import SystemStatus
from datetime import datetime
import psutil
import subprocess
import re
from rclpy import qos
import yaml
import os
from glob import glob
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from system_status import utils
from driving_swarm_messages.srv import GetActiveRobots

class IPLogger(Node):
    def __init__(self):
        super().__init__('ip_logger')
        self.sub = self.create_subscription(
            SystemStatus, '/system_status',  self.system_status_callback, qos.qos_profile_system_default)
        self.local_ips_ = []
        self.srv = self.create_service(GetActiveRobots, 'get_active_robots', self.store_ips)

    def system_status_callback(self, msg):
        local_ip = utils.get_robot_name_from_msg(msg.ip)
        self.local_ips_.append('robot' + local_ip)

    def store_ips(self, request, response):
        if request.timer > 0.0:
            self.create_timer(request.timer, self.send_response(request, response))
        else:
            self.send_response(request, response)

    def send_response(self, request, response):
        robot_ips = list(set(self.local_ips_))
        response.active_robots = robot_ips

def main(args=None):
    rclpy.init(args=args)
    node = IPLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
