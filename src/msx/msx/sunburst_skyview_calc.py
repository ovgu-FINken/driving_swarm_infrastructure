from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

import numpy as np

class SunburstSkyviewCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.listener_callback,
            10
        )

        self.pub = self.create_publisher(Float64MultiArray, "/sunburstSkyview/data", 100)

    def listener_callback(self, msg: ModelStates):
        robot_names = []
        robot_positions = []
        all_data = []

        # --- collect all robot positions ---
        for idx, name in enumerate(msg.name):
            if "robot" in name: 
                pose = msg.pose[idx]
                robot_positions.append(np.array([pose.position.x, pose.position.y]))
                robot_names.append(name)

        robot_positions = np.stack(robot_positions) if robot_positions else np.empty((0, 2))

        # --- labeling for each robot ---
        robot_labels = {}
        robot_distances = {}

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

            # --- 4. assign labels (0 for nearest, then increasing CCW) ---
            labels = {}
            for lbl, (other_name, _, _) in enumerate(shifted):
                labels[other_name] = lbl

            robot_labels[name] = labels
            robot_distances[name] = [(other_name, dist) for other_name, dist, _ in shifted]

            # --- 5. pack into numpy array [neighbors × features] ---
            data_mat = np.array([[dist, angle] for _, dist, angle in shifted])
            all_data.append(data_mat)

        self.robot_labels = robot_labels

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
        msg_out.layout.data_offset = 0
        msg_out.data = full_array.flatten().tolist()

        # --- publish once for all robots ---
        self.pub.publish(msg_out)

        # log robot positions
        self.get_logger().info(f"Robot positions:\n{robot_positions}")
        # log distances of each robot to all others
        for robot, dists in robot_distances.items():
            dist_str = ", ".join([f"{other}: {dist:.2f}" for other, dist in dists])
            self.get_logger().info(f"{robot} distances -> {dist_str}")
        # log robot labeling from camera
        for robot, labels in robot_labels.items():
            self.get_logger().info(f"{robot}: {labels}")

def main():
    main_fn('SunburstSkyviewCalc', SunburstSkyviewCalc)

if __name__ == '__main__':
    main()
