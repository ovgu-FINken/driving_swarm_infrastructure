#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from driving_swarm_messages.srv import VehicleModel

class VehicleModelNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.get_logger().info("Starting")
        self.create_service(VehicleModel, 'nav/vehicle_model', self.vm_callback)
        
    def vm_callback(self, request):
        self.get_logger().info(f'got request {request}')


def main():
    rclpy.init()
    node = VehicleModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'got keyboard interrupt, shutting down')
        node.destroy_node()


if __name__ == '__main__':
    main()
