#!/bin/env/python

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class ScanDelayNode(Node):
    def __init__(self):
        super().__init__('scan_delay_node')
        self.get_logger().info('Starting')
        self.create_subscription(
            LaserScan,
            'scan',
            self.scan_cb,
            rclpy.qos.qos_profile_sensor_data
        )
        self.time_pub = self.create_publisher(Float32, 'scan_delay', 9)
        # self.drop_pub = self.create_publisher(Int32, 'scan_drop', 9)
        # self.seq = None

    def scan_cb(self, msg):
        # check if messages were dropped
        # if self.seq is None:
        #    self.seq = msg.header.seq
        # else:
        #    self.seq += 1
        #    if self.seq < msg.header.seq:
        #        self.drop_pub.publish(self.seq - msg.header.seq)
        #    self.seq = msg.header.seq

        # send delay
        dt = self.get_clock().now() - Time.from_msg(msg.header.stamp)
        self.time_pub.publish(Float32(data=10e-9*dt.nanoseconds))


def main():
    rclpy.init()
    node = ScanDelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting Down')
        node.destroy_node()


if __name__ == "__main__":
    main()
