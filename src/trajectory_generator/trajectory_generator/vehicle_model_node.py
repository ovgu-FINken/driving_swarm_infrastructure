#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from driving_swarm_messages.srv import VehicleModel

class VehicleModelNode(Node):
    def __init__(self):
        super().__init__('vehicle_model_node')
        self.get_logger().info("Starting")
        self.create_service(VehicleModel, 'nav/vehicle_model', self.vm_callback)
        
    def vm_callback(self, request, response):
        self.get_logger().info(f'got request {request}')

        return response


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
