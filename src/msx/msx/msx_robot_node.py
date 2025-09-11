#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from std_msgs.msg import Float64MultiArray
import numpy as np

class MSXRobotNode(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/roofcam/data',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg: Float64MultiArray):
        dims = msg.layout.dim
        num_robots = dims[0].size
        max_neighbors = dims[1].size
        feature_size = dims[2].size

        # --- reshape flat data back into 3D array ---
        full_array = np.array(msg.data).reshape((num_robots, max_neighbors, feature_size))

        # --- separate into distances and angles arrays for convenience ---
        distances = full_array[:, :, 0]  # first feature is distance
        angles = full_array[:, :, 1]     # second feature is angle

        # --- Debugging output ---
        self.get_logger().info(f"Full array (robots x neighbors x features):\n{full_array}")
        self.get_logger().info(f"Distances matrix:\n{distances}")
        self.get_logger().info(f"Angles matrix:\n{angles}")

        # Optional: If you want a list of neighbors per robot
        neighbor_list = []
        for i in range(num_robots):
            neighbors = full_array[i, :, :]
            neighbor_list.append(neighbors)
        self.get_logger().info(f"Neighbor list per robot:\n{neighbor_list}")

def main():
    main_fn('MSXRobotNode', MSXRobotNode)

if __name__ == '__main__':
    main()
