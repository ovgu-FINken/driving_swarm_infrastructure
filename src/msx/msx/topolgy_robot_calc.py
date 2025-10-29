import itertools

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle
from math import atan2
import numpy as np
from numpy.linalg import norm
from numpy.ma.core import shape
from matplotlib.axes._axes import Axes


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.utils import detect_tb_from_ranges
import time, math

class Triangle:
    def __init__(self, pos1, pos2, pos3, lbl1, lbl2, lbl3):
        self.positions = [pos1, pos2, pos3]
        self.labels = [lbl1, lbl2, lbl3]


class TopologyRobotCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.robot_triangles = None
        self.lidar_data = None

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/topologySkyview/data',
            self.listener_callback,
            10)

        self.subscription2 = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)

        '''
        self.pub_marker = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        time.sleep(10)
        self.create_timer(10.0, self.calc_timer)

        # please use this to publish estimated waldo position
        self.pub_error = self.create_publisher(Float64MultiArray, "/TopologyRobotCalc/waldoPosition", 100)
        '''




    def calculate_angle_between_vectors(self, vector1, vector2):
        return np.degrees(np.arctan2(vector1[0] * vector2[1] - vector1[1] * vector2[0],
                                    np.dot(vector1, vector2)))


    def make_nice(self, eval):
        d = val % np.degrees((2 * np.pi))  # -> [0, 360)
        return min(d, np.degrees(2 * np.pi - d))  # -> [0, 180)


    def close_angle(self, value, target, tolerance):
        return abs(value - target) < tolerance


    def determine_label(self,angles, topology):
        results = []
        for tri_idx, triangle in enumerate(topology):
            for vertex in range(3):
                vec1 = triangle.positions[(vertex + 1) % 3] - triangle.positions[vertex]
                vec2 = triangle.positions[(vertex + 2) % 3] - triangle.positions[vertex]
                angle_between_vecs = self.make_nice(self.calculate_angle_between_vectors(vec1, vec2))

                for angle, robots in angles:
                    if self.close_angle(angle, angle_between_vecs, 1):
                        results.append({
                            'identity': robots[0],
                            'triangle': robots,
                            'tri_idx': tri_idx
                        })
        return results




    def do_math(self):
        self.lidar_angles = []
        self.lidar_distances = []

        for loc in self.lidar_data:
            angle = math.atan2(loc[1], loc[0])
            if angle < 0:
                angle += 2 * math.pi
                pass
            self.lidar_angles.append(angle)
            self.lidar_distances.append(math.sqrt(loc[0]**2 + loc[1]**2))
            pass

        
        # it will then generate a list of angles between each pair of these angles
        # these will hopefully match with one from one of the triangles in the topology
        A_angles_between_robots = []
        for a, b in itertools.combinations(self.lidar_angles, 2):
            # the make_nice(...) method ensures the angle returned is in range [0, pi], as we would expect for a triangle
            A_angles_between_robots.append([self.make_nice(b[0] - a[0]), ("0", a[1], b[1])])
            pass
        results = self.determine_label(A_angles_between_robots, self.robot_triangles)

        pass
    
    

    def calc_timer(self):
        # if len(self.lidar_data) > 0:
        #     self.get_logger().info(f"Robot Positions: {self.lidar_data}")
        #     pass

        # To ensure the math won't have a division by zero error!
        if self.topology_angles is not None:
            if len(self.lidar_data) > 1:
                self.do_math()
            else:
                self.get_logger().info(f"DATA MISSING")
            pass
        pass

    def laser_callback(self, msg):
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        self.lidar_data = detect_tb_from_ranges(r, 0.0, 0.0, 0.0, msg.angle_min, msg.angle_increment, cluster_range_threshold=3.5, cluster_size_threshold=20)
        # The output of the positions is relative to the current position of the laser scanner
        # Using the x-axis of the robot (aka the first value from the msg.ranges)

        # Params
        # Cluster linkage threshold -> How far the cluster has to be from another cluster (closeness of points to be considered in the same cluster) this happens first
        # Cluster range threshold -> Points larger than this value are omitted from clusters
        # Cluster size threshold -> any cluster with less than this number of points CAN be still consdiered a turtle bot
        # self.get_logger().info(f"Robots: {self.lidar_data}")
        # Change range threshold to 3.5
        # Look at what the RoLo guys did ...

        marker_array = MarkerArray()

        for i, (x, y) in enumerate(self.lidar_data):
            marker = Marker()
            marker.header.frame_id = "base_scan"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cluster_positions"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 1  # 0 = forever
            marker_array.markers.append(marker)

        self.pub_marker.publish(marker_array)


        pass

    def listener_callback(self, msg: Float64MultiArray):
        '''
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
        # self.get_logger().info(f"Full array (robots x neighbors x features):\n{full_array}")
        # self.get_logger().info(f"Distances matrix:\n{distances}")
        # self.get_logger().info(f"Angles matrix:\n{angles}")

        # Optional: If you want a list of neighbors per robot
        neighbor_list = []
        for i in range(num_robots):
            neighbors = full_array[i, :, :]
            neighbor_list.append(neighbors)
        # self.get_logger().info(f"Neighbor list per robot:\n{neighbor_list}")

        self.skyview_distances = distances
        self.skyview_angles = angles
        '''
        pass

def main():
    main_fn('TopologyRobotCalc', TopologyRobotCalc)

if __name__ == '__main__':
    main()
