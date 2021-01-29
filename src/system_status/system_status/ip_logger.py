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


def extract_local_ip(full_ip):
    x = [full_ip.split('.')]
    local_ip = ''
    for ip in x:
        # print(ip) # e.g. ['127', '0', '0', '1, 10', '61', '10', '245']
        if ip[0] == '127':
            local_ip += ip[-1]
    return local_ip


class IPLogger(Node):
    def __init__(self):
        super().__init__('ip_logger')
        self.sub = self.create_subscription(
            SystemStatus, '/system_status',  self.system_status_callback, qos.qos_profile_system_default)
        self.local_ips_ = []
        self.done_ = False

    def store_ips(self):
        dict_file = [{'local_ips': list(set(self.local_ips_))}]
        pkg_share_dir = get_package_share_directory('system_status')
        with open(os.path.join(pkg_share_dir, 'local_ips.yaml'), 'w+') as file:
            yaml.dump(dict_file, file)

    def system_status_callback(self, msg):
        local_ip = extract_local_ip(msg.ip)
        self.local_ips_.append(local_ip)
        # self.local_ips_.add(local_ip)

    def break_loop(self):
        self.done_ = True


def main(args=None):
    rclpy.init(args=args)
    node = IPLogger()
    # spin for a certain time
    node.create_timer(20.0, node.break_loop)
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.done_:
            break

    node.store_ips()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
