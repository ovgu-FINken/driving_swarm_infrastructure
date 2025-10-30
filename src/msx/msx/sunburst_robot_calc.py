#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.utils import detect_tb_from_ranges
import numpy as np
import time, math

class SunburstRobotCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.skyview_distances = None
        self.skyview_angles = None
        self.lidar_data = None

        # TODO: Make sure data is matched between SkyView message and waldo data
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/sunburstSkyview',
            self.listener_callback,
            10)

        # self.subscription = self.create_subscription(
        #     Float64MultiArray,
        #     'robotA/sunburstSkyviewCalc/data',
        #     self.listener_callback,
        #     10)

        self.subscription2 = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.sub_waldo_pos = self.create_subscription(
            Float64MultiArray,
            "/sunburstSkyview/waldo",
            self.waldo_cb,
            10)

        self.pub_marker = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        time.sleep(10)
        self.create_timer(10.0, self.calc_timer)

        # please use this to publish estimated waldo position
        self.pub_error = self.create_publisher(Float64MultiArray, "/sunburstRobotCalc/waldoPosition", 100)

    def waldo_cb(self, msg: Float64MultiArray):
        #print(f"WALDO : {msg}")

        data = np.array(msg.data).reshape((-1, 2))
        distances = data[:, 0]
        angles = data[:, 1]

        #print(f"WALDO dist : {distances} WALDO angle : {angles}")

    def calc_angle_error(self, alpha, sigmas, thetas):
        sum = 0
        for idx in range(len(sigmas)):
            sum += (alpha + sigmas[idx] - thetas[idx])**2
            pass
        return sum


    def calc_distance_error(self, scale, Ls, ls):
        sum = 0
        for idx in range(len(ls)):
            sum += (scale*ls[idx] - Ls[idx])**2
            pass
        return sum


    def do_math(self):
        # Okay at this point I have self.skyview_distances, self.skyview_angles, self.lidar_data
        # Assume N robots
        # self.skyview_angles[robot 0-(N-1)][0] = the angle between the x-axis of the indexed robot and the closest robot (this should be 0 as the closest robot determines the x-axis)
        # self.skyview_angles[robot 0-(N-1)][1-(N-2)] = the angle between the x-axis of the indexed robot and the next N-2 robots (in clockwise order from the x-axis)

        # self.skyview_distances[robot 0-(N-1)][0] = the distance to the closest robot (which forms the x-axis)
        # self.skyview_distances[robot 0-(N-1)][1-(N-2)] = the distance to the next N-2 robots (sorted according to the angles in self.skyview_angles)

        # self.lidar_data[robot 0-(N-2)][...] = a list of np arrays each with the x and y positions (respectively) of a robot relative to the position of the robot

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

        # For the data from each of the N robots
        min_scales = []
        min_angles = []
        for rbt_idx, _ in enumerate(self.skyview_angles):
            divisor = 1
            the_sum = 0
            for other_idx in range(len(self.skyview_angles[rbt_idx])):
                the_sum -= 2*self.lidar_angles[other_idx]
                the_sum += 2*self.skyview_angles[rbt_idx][other_idx]
                divisor += 2
                pass
            angle = the_sum/divisor
            min_angles.append(angle)

            divisor = 0
            the_sum = 0
            for other_idx, _ in enumerate(self.skyview_distances[rbt_idx]):
                the_sum += 2*self.skyview_distances[rbt_idx][other_idx]*self.lidar_distances[other_idx]
                divisor += 2*self.skyview_distances[rbt_idx][other_idx]**2
                pass
            scale = the_sum/divisor
            min_scales.append(scale)
            pass

        angle_errors = []
        scale_errors = []
        for rbt_idx in range(len(self.skyview_angles)):
            angle_errors.append(self.calc_angle_error(min_angles[rbt_idx], self.lidar_angles, self.skyview_angles[rbt_idx]))
            scale_errors.append(self.calc_distance_error(min_scales[rbt_idx], self.lidar_distances, self.skyview_distances[rbt_idx]))
            pass

        angle_idx = angle_errors.index(min(angle_errors))
        scale_idx = scale_errors.index(min(scale_errors))

        self.get_logger().info(f"I AM {angle_idx} I AM {scale_idx}")

        # self.get_logger().info(self.skyview_distances)
        # self.get_logger().info(self.skyview_angles)

        # The skyview distances
        # 

        # for main_idx in range(len(self.lidar_data)):

        pass


    def calc_timer(self):
        # if len(self.lidar_data) > 0:
        #     self.get_logger().info(f"Robot Positions: {self.lidar_data}")
        #     pass

        # To ensure the math won't have a division by zero error!
        if self.skyview_angles is not None and self.skyview_distances is not None:
            if len(self.lidar_data) > 1 and len(self.skyview_angles) > 1 and len(self.skyview_distances) > 1:
                self.do_math()
                pass
            else:
                self.get_logger().info(f"DATA MISSING")
            pass
        pass

    def laser_callback(self, msg):
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        self.lidar_data = detect_tb_from_ranges(r, 0.0, 0.0, 0.0, msg.angle_min, msg.angle_increment)
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
