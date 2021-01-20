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

    def store_ips(self):
        dict_file = [{'local_ips': self.local_ips_}]
        # base_path = Path(__file__).parent
        # file_path = (base_path / "../log/local_ips.yaml").resolve()
        cwd = os.getcwd()
        with open(os.path.join(cwd, 'local_ips.yaml'), 'w+') as file:
            documents = yaml.dump(dict_file, file)

    def system_status_callback(self, msg):
        local_ip = extract_local_ip(msg.ip)
        self.local_ips_.append(local_ip)


def main(args=None):
    rclpy.init(args=args)
    node = IPLogger()
    # main loop
    # rclpy.spin(node)
    rclpy.spin_once(node, timeout_sec=20)
    node.store_ips()
    node.destroy_node()
    rclpy.shutdown()

    # example from tf2_tool view_frames.py
    #  rospy.init_node('view_frames')
    #  # listen to tf for 5 seconds
    #  rospy.loginfo('Listening to tf data during 5 seconds...')
    #  rospy.sleep(0.00001)
    #  buffer = tf2_ros.Buffer()
    #  listener = tf2_ros.TransformListener(buffer)
    #  rospy.sleep(5.0)

    #  rospy.loginfo('Generating graph in frames.pdf file...')
    #  rospy.wait_for_service('~tf2_frames')
    #  srv = rospy.ServiceProxy('~tf2_frames', FrameGraph)
    #  data = yaml.safe_load(srv().frame_yaml)
    #  with open('frames.gv', 'w') as f:
    #      f.write(generate_dot(data))
    #  subprocess.Popen('dot -Tpdf frames.gv -o frames.pdf'.split(' ')).communicate()


if __name__ == '__main__':
    main()
