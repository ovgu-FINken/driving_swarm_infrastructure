from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

import numpy as np

class SunburstSkyviewCalc(DrivingSwarmNode):
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

        self.pub = self.create_publisher(Float64MultiArray, "/sunburstSkyview", 100)

        self.pub_waldo = self.create_publisher(Float64MultiArray, "/sunburstSkyview/waldo", 100)


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

            if not others:
                robot_labels[name] = {}
                robot_distances[name] = {}
                continue

            # --- 1. find nearest neighbor (gets label 0) ---
            nearest = min(others, key=lambda x: x[1])  
            nearest_name, _, ref_angle = nearest

            # --- 2. shift all other angles so that nearest neighbor is at 0 ---
            shifted = []
            for other_name, dist, angle in others:
                rel_angle = angle - ref_angle
                if rel_angle < 0:
                    rel_angle += 2 * np.pi  # normalize into [0, 2π)
                shifted.append((other_name, dist, rel_angle))


            # --- 3. sort by angle (counter-clockwise) ---
            shifted.sort(key=lambda x: x[2])
            labels = {other_name: lbl for lbl, (other_name, _, _) in enumerate(shifted)}

            # --- 4. assign labels (0 for nearest, then increasing CCW) ---
            labels = {}
            for lbl, (other_name, _, _) in enumerate(shifted):
                labels[other_name] = lbl

            robot_labels[name] = labels
            robot_distances[name] = [(other_name, dist) for other_name, dist, _ in shifted]

            # --- 5. pack into numpy array [neighbors × features] ---
            data_mat = np.array([[dist, angle] for _, dist, angle in shifted])
            all_data.append(data_mat)

            # --- 6. compute Waldo distance and relative angle ---
            vec_w = self.waldo_pos - own_pos
            dist_w = np.linalg.norm(vec_w)
            angle_w = np.arctan2(vec_w[1], vec_w[0])
            rel_angle_w = angle_w - ref_angle
            if rel_angle_w < 0:
                rel_angle_w += 2 * np.pi

            waldo_data.append([dist_w, rel_angle_w])

        self.robot_labels = robot_labels

        if not all_data:
            return

        # --- align sizes (pad with -1 if different neighbor counts) ---
        max_neighbors = max(arr.shape[0] for arr in all_data)
        num_robots = len(all_data)
        feature_size = 2

        full_array = -1.0 * np.ones((num_robots, max_neighbors, feature_size))  # padding with -1
        for i, arr in enumerate(all_data):
            n = arr.shape[0]
            full_array[i, :n, :] = arr

        # --- pack into Float64MultiArray ---
        msg_out = Float64MultiArray()
        msg_out.layout.dim = [
            MultiArrayDimension(label="robots", size=num_robots, stride=num_robots * max_neighbors * feature_size),
            MultiArrayDimension(label="neighbors", size=max_neighbors, stride=max_neighbors * feature_size),
            MultiArrayDimension(label="features", size=feature_size, stride=feature_size)
        ]
        msg_out.data = full_array.flatten().tolist()

        # --- publish once for all robots ---
        self.pub.publish(msg_out)

        # --- publish Waldo data ---
        waldo_arr = np.array(waldo_data, dtype=np.float64)
        waldo_msg = Float64MultiArray()
        waldo_msg.layout.dim = [
            MultiArrayDimension(label="waldo", size=num_robots, stride=num_robots * 2),
            MultiArrayDimension(label="features", size=2, stride=2)  # [distance, angle]
        ]
        waldo_msg.data = waldo_arr.flatten().tolist()
        self.pub_waldo.publish(waldo_msg)
        #print(f"Published waldo data : {waldo_msg}")

        # log robot positions
        #self.get_logger().info(f"Robot positions:\n{robot_positions}")
        # log distances of each robot to all others
        #for robot, dists in robot_distances.items():
        #    dist_str = ", ".join([f"{other}: {dist:.2f}" for other, dist in dists])
        #    self.get_logger().info(f"{robot} distances -> {dist_str}")
        # log robot labeling from camera
        #for robot, labels in robot_labels.items():
        #    self.get_logger().info(f"{robot}: {labels}")

def main():
    main_fn('SunburstSkyviewCalc', SunburstSkyviewCalc)

if __name__ == '__main__':
    main()
