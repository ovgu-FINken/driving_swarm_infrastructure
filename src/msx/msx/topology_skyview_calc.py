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

        all_data = []
        robot_labels = {}
        robot_distances = {}

        waldo_data = []

        for i, name in enumerate(robot_names):
            own_pos = robot_positions[i]

            # compute vectors, distances, and angles to all other robots
            others = []
            for j, other_name in enumerate(robot_names):
                if i == j:
                    continue
                vec = robot_positions[j] - own_pos
                dist = np.linalg.norm(vec)
                angle = np.arctan2(vec[1], vec[0])  # angle relative to cameras x-axis
                others.append((other_name, dist, angle))
        

def main():
    main_fn('TopologySkyviewCalc', TopologySkyviewCalc)

if __name__ == '__main__':
    main()
