#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.utils import detect_tb_from_ranges
import numpy as np
import time

class SunburstRobotCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.skyview_distances = None
        self.skyview_angles = None
        self.lidar_data = None

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/sunburstSkyview/data',
            self.listener_callback,
            10)

        self.subscription2 = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.pub_marker = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        time.sleep(10)
        self.create_timer(10.0, self.calc_timer)

        # please use this to publish estimated waldo position
        self.pub_error = self.create_publisher(Float64MultiArray, "/sunburstRobotCalc/waldoPosition", 100)


    def do_math(self):
        # self.get_logger().info('do_math')
        if len(self.lidar_data) > 0:
            self.get_logger().info(f"Robot Positions: {self.lidar_data}")
            pass
        # self.get_logger().info(self.skyview_distances)
        # self.get_logger().info(self.skyview_angles)

        # The skyview distances
        # 

        # for main_idx in range(len(self.lidar_data)):

        pass

    def calc_timer(self):
        self.do_math()
        pass

    def laser_callback(self, msg):
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        self.lidar_data = detect_tb_from_ranges(r, 0.0, 0.0, 0.0, msg.angle_min, msg.angle_increment, cluster_size_threshold=20)
        # self.get_logger().info(f"Robots: {self.lidar_data}")

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

        self.publisher.publish(marker_array)


        pass

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
        pass

def main():
    main_fn('SunburstRobotCalc', SunburstRobotCalc)

if __name__ == '__main__':
    main()
