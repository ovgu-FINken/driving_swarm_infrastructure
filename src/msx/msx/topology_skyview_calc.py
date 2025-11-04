from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

import numpy as np

class TopologySkyviewCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.waldo_pos = None

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/robotPos',
            self.listener_callback,
            10
        )

        self.sub_waldo_pos = self.create_subscription(
            Point,
            '/waldoPos',
            self.waldo_cb,
            10
        )

        self.pub = self.create_publisher(Float64MultiArray, "/topologySkyview/data", 100)

        self.pub_waldo = self.create_publisher(Float64MultiArray, "/topologySkyview/waldo", 100)


    def waldo_cb(self, msg: Point):
        self.waldo_pos = (msg.x, msg.y)
        # print(f"recieved waldo pos : {self.waldo_pos}")

    def listener_callback(self, msg: Float64MultiArray):
        # --- reconstruct Nx2 position matrix from Float64MultiArray ---
        data = np.array(msg.data, dtype=np.float64)
        if data.size == 0 or self.waldo_pos == None:
            return

        # Each row: [x, y] for one robot
        num_robots = msg.layout.dim[0].size if msg.layout.dim else data.size // 2
        robot_positions = data.reshape((num_robots, 2))

        # Robot naming convention (for internal consistency)
        robot_names = [f"robot_{i}" for i in range(num_robots)]

        all_results = {}

        # for i, main_name in enumerate(robot_names):
        #     main_pos = robot_positions[i]
        #
        #     other_indices = [j for j in range(num_robots) if j != i]
        #
        #     main_results = []
        #
        #     for k, idx in enumerate(other_indices):
        #         if k == 0:
        #             prev_idx = other_indices[0]
        #         else:
        #             prev_idx = other_indices[k - 1]
        #
        #         next_idx = other_indices[(k + 1) % len(other_indices)]
        #
        #         prev_vec = robot_positions[prev_idx] - main_pos
        #         next_vec = robot_positions[next_idx] - main_pos
        #
        #         dist = np.linalg.norm(prev_vec)
        #         angle_rad = np.arctan2(
        #             np.cross(prev_vec, next_vec),
        #             np.dot(prev_vec, next_vec)
        #         )
        #         angle_deg = np.degrees(angle_rad)
        #
        #         main_results.append([dist, angle_rad])
        #
        #         print(f"{main_name} -> {robot_names[idx]}: Distanz = {dist:.3f}, Winkel = {angle_deg:.2f}°")
        #
        #     all_results[main_name] = main_results

def main():
    main_fn('TopologySkyviewCalc', TopologySkyviewCalc)

if __name__ == '__main__':
    main()
