#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.utils import detect_tb_from_ranges
from geometry_msgs.msg import Point
import numpy as np
import time, math


class SunburstRobotCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.skyview_distances = []
        self.skyview_angles = []
        self.lidar_data = []
        self.scale_error_weight = 1
        self.angle_error_weight = 1

        # TODO: thresholds are way to high, so we do not skip too much for now
        self.dist_threshold = 1.5
        self.angle_threshold = np.pi / 6

        # TODO: Make sure data is matched between SkyView message and waldo data
        self.skyview_sub = self.create_subscription(
            Float64MultiArray,
            '/sunburstSkyview/data',
            self.skyview_cb,
            10)

        self.waldo_pos_sub = self.create_subscription(
            Float64MultiArray,
            "/sunburstSkyview/waldo",
            self.waldo_cb,
            10)

        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_cb,
            rclpy.qos.qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        self.debug_pub = self.create_publisher(MarkerArray, 'visualization_of_sunburst', 10)

        # please use this to publish estimated waldo position
        self.waldo_pub = self.create_publisher(Float64MultiArray, "sunburstRobotCalc/waldoPosition", 10)

        # time.sleep(10)
        self.create_timer(1.0, self.calc_timer)

    def waldo_cb(self, msg: Float64MultiArray):
        # print(f"WALDO : {msg}")

        data = np.array(msg.data).reshape((-1, 2))

        self.skyview_waldo_distances = data[:, 0]
        self.skyview_waldo_angles = data[:, 1]

        # print(f"WALDO dist : {distances} WALDO angle : {angles}")

    def calc_angle_error(self, alpha, sigmas, thetas):
        sum = 0
        for idx in range(len(sigmas)):
            sum += (alpha + sigmas[idx] - thetas[idx]) ** 2
            pass
        return sum

    def calc_distance_error(self, scale, Ls, ls):
        sum = 0
        for idx in range(len(ls)):
            sum += (scale * ls[idx] - Ls[idx]) ** 2
            pass
        return sum

    def sunburst_single_robot(self, sky_dist, sky_angle, lidar_dist, lidar_angle):
        assignments = set()
        for sky_idx, sky_dist_x in enumerate(sky_dist):
            for sky_idy, sky_dist_y in enumerate(sky_dist):
                if sky_idx == sky_idy:
                    continue
                for lidar_idx, lidar_dist_x in enumerate(lidar_dist):
                    for lidar_idy, lidar_dist_y in enumerate(lidar_dist):
                        if lidar_idx == lidar_idy:
                            continue
                        # threshold check dist
                        s_sky = sky_dist_x / sky_dist_y
                        s_lidar = lidar_dist_x / lidar_dist_y
                        s_discrepancy = max(s_sky, s_lidar) / min(s_sky, s_lidar)

                        # self.get_logger().info(f"dist threshold: sky = ({sky_dist_x:.2f}, {sky_dist_y:.2f}), lidar = ({lidar_dist_x:.2f}, {lidar_dist_y:.2f})")
                        # self.get_logger().info(f"dist threshold: s_sky ={s_sky :.2f}, s_lidar={s_lidar :.2f}")
                        # self.get_logger().info(f"dist_threshold: s_discrepancy = {s_discrepancy:.2f} < {self.dist_threshold}")

                        if s_discrepancy > self.dist_threshold:
                            continue

                        # threshold check angles
                        # TODO: angles seem wrong!
                        angle_diff_sky = (sky_angle[sky_idx] % (np.pi * 2) - sky_angle[sky_idy] % (np.pi * 2)) % (
                                np.pi * 2)
                        angle_diff_lidar = (lidar_angle[lidar_idx] % (np.pi * 2) - lidar_angle[lidar_idy] % (
                                np.pi * 2)) % (np.pi * 2)
                        # self.get_logger().info(
                        #     f"angle_threshold: sky_diff = {angle_diff_sky % (np.pi * 2):.2f}, lidar_diff = {angle_diff_lidar % (np.pi * 2):.2f}")
                        a1 = np.abs(angle_diff_sky - angle_diff_lidar)
                        # if we have 1deg or 359 deg, take 1 deg
                        angle_diff = min(a1, -a1 % (2 * np.pi))
                        # self.get_logger().info(
                        #     f"angle_threshold: angle_diff = {angle_diff : .2f} < {self.angle_threshold : .2f}")
                        if angle_diff > self.angle_threshold:
                            continue

                        assignments.add((sky_idx, lidar_idx))
                        assignments.add((sky_idy, lidar_idy))
                        pass
                    pass
                pass
            pass
        if not len(assignments):
            return np.inf, np.inf, np.inf, np.inf
        # self.get_logger().info(f"assignments: {assignments}")

        sky_dist_subset = [sky_dist[sky_idx] for sky_idx, lidar_idx in assignments]
        sky_angle_subset = [sky_angle[sky_idx] for sky_idx, lidar_idx in assignments]
        lidar_dist_subset = [lidar_dist[lidar_idx] for sky_idx, lidar_idx in assignments]
        lidar_angle_subset = [lidar_angle[lidar_idx] for sky_idx, lidar_idx in assignments]

        angle_error, scale_error, angle, scale = self.do_optimization(sky_dist_subset, sky_angle_subset, lidar_dist_subset, lidar_angle_subset)
        return angle_error, scale_error, angle, scale

    def do_optimization(self, sky_dists, sky_angles, lidar_dists, lidar_angles):
        scale_sum = 0
        scale_divisor = 1
        angle_sum = 0
        angle_divisor = 0
        for sky_dist, sky_angle, lidar_dist, lidar_angle in zip(sky_dists, sky_angles, lidar_dists, lidar_angles):  # For each assignment
            angle_sum -= 2 * lidar_angle
            angle_sum += 2 * sky_angle
            angle_divisor += 2

            scale_sum += 2 * sky_dist * lidar_dist
            scale_divisor += 2 * sky_dist ** 2
            pass
        best_angle = angle_sum / angle_divisor
        best_scale = scale_sum / scale_divisor

        angle_error = self.calc_angle_error(best_angle, lidar_angles, sky_angles)
        scale_error = self.calc_distance_error(best_scale, lidar_dists, sky_dists)

        return angle_error, scale_error, best_angle, best_scale

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
            self.lidar_distances.append(math.sqrt(loc[0] ** 2 + loc[1] ** 2))
            pass

        # For the data from each of the N robots
        min_scales = []
        min_angles = []
        errors = {}
        vals = {}
        for rbt_idx, _ in enumerate(self.skyview_angles):
            angle_error, scale_error, angle, scale = self.sunburst_single_robot(self.skyview_distances[rbt_idx], self.skyview_angles[rbt_idx],
                                                        self.lidar_distances, self.lidar_angles)
            if angle_error != np.inf and scale_error != np.inf:
                errors[rbt_idx] = self.angle_error_weight * angle_error + self.scale_error_weight * scale_error
                vals[rbt_idx] = [angle, scale]
                pass
            pass

        if len(errors):
            my_rbt_idx = min(errors, key=errors.get)

            waldo_angle = self.skyview_waldo_angles[my_rbt_idx] - vals[my_rbt_idx][0]
            waldo_distance = self.skyview_waldo_distances[my_rbt_idx] * vals[my_rbt_idx][1]

            waldo_x = waldo_distance * math.cos(waldo_angle)
            waldo_y = waldo_distance * math.sin(waldo_angle)

            self.get_logger().info(f"idx, angle, dist: {my_rbt_idx}, {self.skyview_waldo_angles[my_rbt_idx]}, {self.skyview_waldo_distances[my_rbt_idx]}")
            to_send = Float64MultiArray()
            to_send.data = [waldo_x, waldo_y]
            self.waldo_pub.publish(to_send)

            marker_array = MarkerArray()

            for idx, dat in enumerate(self.skyview_angles[my_rbt_idx]):
                marker = Marker()
                marker.header.frame_id = "base_scan"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"sunburst{idx}"
                marker.id = idx
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.scale.x = 0.02  # arrow shaft length scale
                marker.scale.y = 0.002  # shaft thickness
                marker.scale.z = 0.002
                marker.color.a = 1.0
                marker.color.b = 0.5
                marker.color.r = 0.5

                robot_angle = (dat - vals[my_rbt_idx][0]) % (2 * math.pi)
                robot_distance = self.skyview_distances[my_rbt_idx][idx] * vals[my_rbt_idx][1]

                robot_x = robot_distance * math.cos(robot_angle)
                robot_y = robot_distance * math.sin(robot_angle)

                marker.points = [
                    Point(x=0.0, y=0.0, z=0.0),
                    Point(x=float(robot_x), y=float(robot_y), z=0.0)
                ]
                marker_array.markers.append(marker)

            self.debug_pub.publish(marker_array)
            pass

        else:
            pass
        pass
        # self.get_logger().info(f"matching errors: {errors}")
        #    divisor = 1
        #    the_sum = 0
        #    for other_idx in range(len(self.skyview_angles[rbt_idx])):
        #        the_sum -= 2*self.lidar_angles[other_idx]
        #        the_sum += 2*self.skyview_angles[rbt_idx][other_idx]
        #        divisor += 2
        #        pass
        #    angle = the_sum/divisor
        #    min_angles.append(angle)

        #    divisor = 0
        #    the_sum = 0
        #    for other_idx, _ in enumerate(self.skyview_distances[rbt_idx]):
        #        the_sum += 2*self.skyview_distances[rbt_idx][other_idx]*self.lidar_distances[other_idx]
        #        divisor += 2*self.skyview_distances[rbt_idx][other_idx]**2
        #        pass
        #    scale = the_sum/divisor
        #    min_scales.append(scale)
        #    pass

        # angle_errors = []
        # scale_errors = []
        # for rbt_idx in range(len(self.skyview_angles)):
        #    angle_errors.append(self.calc_angle_error(min_angles[rbt_idx], self.lidar_angles, self.skyview_angles[rbt_idx]))
        #    scale_errors.append(self.calc_distance_error(min_scales[rbt_idx], self.lidar_distances, self.skyview_distances[rbt_idx]))
        #    pass

        # angle_idx = angle_errors.index(min(angle_errors))
        # scale_idx = scale_errors.index(min(scale_errors))

        # self.get_logger().info(f"I AM {angle_idx} I AM {scale_idx}")

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
                pass
                # self.get_logger().info(f"DATA MISSING")
            pass
        pass

    def laser_cb(self, msg):
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

        self.marker_pub.publish(marker_array)

        pass

    def skyview_cb(self, msg: Float64MultiArray):
        dims = msg.layout.dim
        num_robots = dims[0].size
        max_neighbors = dims[1].size
        feature_size = dims[2].size

        # --- reshape flat data back into 3D array ---
        full_array = np.array(msg.data).reshape((num_robots, max_neighbors, feature_size))

        # --- separate into distances and angles arrays for convenience ---
        distances = full_array[:, :, 0]  # first feature is distance
        angles = full_array[:, :, 1]  # second feature is angle

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
