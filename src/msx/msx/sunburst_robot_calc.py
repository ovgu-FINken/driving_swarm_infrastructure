#!/usr/bin/env python3
import rclpy
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from std_msgs.msg import Float64MultiArray, String, Int32
from sensor_msgs.msg import LaserScan, PointCloud
from driving_swarm_utils.utils import detect_tb_from_ranges
from geometry_msgs.msg import Point32
import numpy as np
import math


class SunburstRobotCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        # ===== Parameters =====
        self.DEBUG = True
        self.USE_BAYES_FILTER = True
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        # TODO: thresholds are way to high, so we do not skip too much for now
        self.dist_threshold = 1.05
        self.angle_threshold = np.pi / 12
        # weights for errors weighted sum calculaiton
        self.scale_error_weight = 1
        self.angle_error_weight = 1
        self.alpha = 0.1


        # ===== Variables =====
        self.DEBUG_cur_robot_heading = None

        # input data for identification estimate
        self.skyview_distances = []
        self.skyview_angles = []
        self.lidar_data = []
        # current and last probabilities for identification
        self.cur_probs = {}
        self.last_probs = {}
        self.bayes_errors = {}
        self.bayes_vals = {}

        self.get_list_of_robot_names()

        # ===== Subscriptions ===== 
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

        # DEBUG
        if self.DEBUG:
            self.groundtruth_pos_sub = self.create_subscription(
                Float64MultiArray,
                "/robotPos",
                self.groundtruth_pos_callback,
                100
            )

            self.groundtruth_heading_sub = self.create_subscription(
                Float64MultiArray,
                "/robotHeadings",
                self.groundtruth_heading_callback,
                100
            )

            ns = self.get_namespace()
            self.own_name = ns.strip("/")
            self.cur_robot_idx = self.robots.index(self.own_name)

        # ===== Publishers =====
        self.waldo_pub = self.create_publisher(Float64MultiArray, "sunburstRobotCalc/waldoPosition", 10)

        self.pub_lidar_data = self.create_publisher(PointCloud, "lidarData", 10)
        self.lidar_detection_count_pub = self.create_publisher(Int32, "num_detections", 10)
        self.identity_pub = self.create_publisher(String, "identify_as", 10)

        # time.sleep(10)
        self.create_timer(0.1, self.calc_timer)


    # =============================================
    # ================= Callbacks =================
    # =============================================

    def waldo_cb(self, msg: Float64MultiArray):
        data = np.array(msg.data).reshape((-1, 2))

        self.skyview_waldo_distances = data[:, 0]
        self.skyview_waldo_angles = data[:, 1]

        # print(f"WALDO dist : {distances} WALDO angle : {angles}")


    def laser_cb(self, msg):
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]

        # Using the x-axis of the robot (aka the first value from the msg.ranges)
        # The output of the positions is relative to the current position of the laser scanner
        # TODO: at this point a laser scan position filter could be implemented

        # ===== Params =====
        # Cluster linkage threshold -> How far the cluster has to be from another cluster (closeness of points to be considered in the same cluster) this happens first
        # Cluster range threshold -> Points larger than this value are omitted from clusters
        # Cluster size threshold -> any cluster with less than this number of points CAN be still consdiered a turtle bot
        # ranges, px=0.0, py=0.0, pt=0.0, angle_min=0.0, angle_increment=1.0,cluster_linkage_threshold=0.15,
        # cluster_range_threshold=1.5, cluster_size_threshold=30
        if self.DEBUG == False:
            self.lidar_data = detect_tb_from_ranges(r, 0.0, 0.0, 0.0, msg.angle_min, msg.angle_increment)

        #self.get_logger().info(f"Lidar data : {self.lidar_data}")

        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_frame"

        for x, y in self.lidar_data:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            msg.points.append(p)

        # Publish for visualization
        self.pub_lidar_data.publish(msg)

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


    def groundtruth_pos_callback(self, msg):
        if self.DEBUG_cur_robot_heading is None or self.DEBUG == False:
            return

        # self.get_logger().info(f"groundtruth_pos_callback, msg: {msg}")

        flat = msg.data
        pairs = [[flat[i], flat[i+1]] for i in range(0, len(flat), 2)]

        cur_robot_position = np.array(pairs[self.cur_robot_idx])

        self.lidar_data = [];
        for i in range(len(self.robots)):
            if i != self.cur_robot_idx:
                R = self.rotation_matrix_2d(-self.DEBUG_cur_robot_heading)
                self.lidar_data.append(R @ (pairs[i] - cur_robot_position))

        #self.get_logger().info(f"I am {cur_robot_idx} and am at {cur_robot_position} facing {self.DEBUG_cur_robot_heading}; all robots are at {pairs}; in my frame they are at: {self.lidar_data}")
        return

    def groundtruth_heading_callback(self, msg):
        if self.DEBUG == False:
            return

        flat = msg.data

        self.DEBUG_cur_robot_heading = flat[self.cur_robot_idx]
        return

    # =============================================
    # ================== Helpers ==================
    # =============================================

    def rotation_matrix_2d(self, angle):
        return np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle),  np.cos(angle)]])

    def angle_diff(self, a, b):
        diff = abs(a - b) % (2 * math.pi)
        return min(diff, 2 * math.pi - diff)

    def calc_angle_error(self, alpha, sigmas, thetas):
        sum = 0
        for idx in range(len(sigmas)):
            sum += (alpha + sigmas[idx] - thetas[idx]) ** 2
        return sum

    def calc_distance_error(self, scale, Ls, ls):
        sum = 0
        for idx in range(len(ls)):
            sum += (scale * ls[idx] - Ls[idx]) ** 2
        return sum

    def one_likelihood(self, one_error):
        return np.exp(-self.alpha*one_error)

    def calc_likelihood(self):
        vals = np.array([])
        for id, err in self.bayes_errors.items():
            val = self.one_likelihood(err)*self.last_probs[id]
            vals = np.append(vals, max(val, 0.01))
            # vals = np.append(vals, val)
        return vals/sum(vals)

    # =============================================
    # =============== Calculations ================
    # =============================================
    def calc_timer(self):
        # Wait for data and to ensure the math won't have a division by zero error
        if self.skyview_angles is None and self.skyview_distances is None:
            if len(self.lidar_data) < 2 and len(self.skyview_angles) < 2 and len(self.skyview_distances) < 2:
                return

        # Okay at this point I have self.skyview_distances, self.skyview_angles, self.lidar_data
        # Assume N robots
        # self.skyview_angles[robot 0-(N-1)][0] = the angle between the x-axis of the indexed robot and the closest robot (this should be 0 as the closest robot determines the x-axis)
        # self.skyview_angles[robot 0-(N-1)][1-(N-2)] = the angle between the x-axis of the indexed robot and the next N-2 robots (in clockwise order from the x-axis)

        # self.skyview_distances[robot 0-(N-1)][0] = the distance to the closest robot (which forms the x-axis)
        # self.skyview_distances[robot 0-(N-1)][1-(N-2)] = the distance to the next N-2 robots (sorted according to the angles in self.skyview_angles)

        # self.lidar_data[robot 0-(N-2)][...] = a list of np arrays each with the x and y positions (respectively) of a robot relative to the position of the robot

        lidar_angles = []
        lidar_distances = []

        for loc in self.lidar_data:
            angle = math.atan2(loc[1], loc[0])
            if angle < 0:
                angle += 2 * math.pi
            lidar_angles.append(angle)
            lidar_distances.append(math.sqrt(loc[0] ** 2 + loc[1] ** 2))

        # For the data from each of the N robots
        errors = {}
        vals = {}

        # If this is the first time we calculate errors, initialize the bayes_errors to infinity
        if len(self.bayes_errors) == 0:
            self.bayes_errors = {k: np.inf for k in range(len(self.skyview_angles))}

        for rbt_idx, _ in enumerate(self.skyview_angles):
            angle_error, scale_error, angle, scale = self.sunburst_single_robot(self.skyview_distances[rbt_idx], self.skyview_angles[rbt_idx],
                                                        lidar_distances, lidar_angles)
            if angle_error != np.inf and scale_error != np.inf:
                errors[rbt_idx] = self.angle_error_weight * angle_error + self.scale_error_weight * scale_error
                vals[rbt_idx] = [angle, scale]
            # elif self.USE_BAYES_FILTER:
                # errors[rbt_idx] = np.inf
                # vals[rbt_idx] = [angle, scale]

        # choose min error as current estimate
        # TODO: think about what we need to do, if anything, if errors is empty
        if len(errors):
            if self.USE_BAYES_FILTER:
                self.get_logger().info(f"Bayes filter error: {errors}")

                # If this is the first time we calculate probabilities, initialize last_probs uniformly
                if len(self.cur_probs) == 0:
                    self.last_probs = {k: 1/len(self.skyview_angles) for k in range(len(self.skyview_angles))}

                # Grab the new errors calculated and save them
                for id, err in errors.items():
                    self.bayes_errors[id] = err

                new_probs = self.calc_likelihood()
                for id, prob in enumerate(new_probs):
                    self.cur_probs[id] = prob


                self.last_probs = self.cur_probs.copy()
                my_rbt_idx = max(self.cur_probs, key=self.cur_probs.get)
                self.get_logger().info(f"Probs: {self.bayes_errors} -> {self.cur_probs} -> {my_rbt_idx}")
            else:
                my_rbt_idx = min(errors, key=errors.get)

            # if my_rbt_idx not in vals:
            #     waldo_angle = self.skyview_waldo_angles[my_rbt_idx] - vals[my_rbt_idx][0]
            #     waldo_distance = self.skyview_waldo_distances[my_rbt_idx] * vals[my_rbt_idx][1]
            #
            #     waldo_x = waldo_distance * math.cos(waldo_angle)
            #     waldo_y = waldo_distance * math.sin(waldo_angle)
            #
            #     self.get_logger().info(f"idx, angle, dist: {my_rbt_idx}, {self.skyview_waldo_angles[my_rbt_idx]}, {self.skyview_waldo_distances[my_rbt_idx]}")
            #
            #     # publish waldos min error position in the robot's reference frame
            #     min_error_waldo_pos = Float64MultiArray()
            #     min_error_waldo_pos.data = [waldo_x, waldo_y]
            #     self.waldo_pub.publish(min_error_waldo_pos)
            #
            #     # publish estimated robot identity with min error
            #     min_error_robot_id = String()
            #     min_error_robot_id.data = str(self.robots[my_rbt_idx])
            #     self.identity_pub.publish(min_error_robot_id)
            #
            #     # publish lidar detection count
            #     self.lidar_detection_count_pub.publish(Int32(data=int(len(lidar_angles))))

    
    # filters out every comparison that is above a given threshold
    def sunburst_single_robot(self, sky_dist, sky_angle, lidar_dist, lidar_angle):
            assignments = set()
            # TODO: remove double comparisons (x,y) <=> (y,x) for optimization
            for sky_id_a, sky_dist_a in enumerate(sky_dist):
                for sky_id_b, sky_dist_b in enumerate(sky_dist):

                    # skip if we compare same robot
                    if sky_id_a == sky_id_b or sky_id_a < sky_id_b:
                        continue

                    for lidar_id_a, lidar_dist_a in enumerate(lidar_dist):
                        for lidar_id_b, lidar_dist_b in enumerate(lidar_dist):

                            # skip if we compare same robot
                            if lidar_id_a == lidar_id_b or lidar_id_a < lidar_id_b:
                                continue

                            # filter out robot if distance discrepancy is too high
                            distance_diff_sky = sky_dist_a / sky_dist_b
                            distance_diff_lidar = lidar_dist_a / lidar_dist_b
                            distance_diff = max(distance_diff_sky, distance_diff_lidar) / min(distance_diff_sky, distance_diff_lidar)

                            if distance_diff > self.dist_threshold:
                                continue

                            # filter out robot if angle discrepancy is too high
                            # TODO: try figuring out whats about the angle errors
                            angle_diff_sky = abs(sky_angle[sky_id_a] % (np.pi * 2) - sky_angle[sky_id_b] % (np.pi * 2))
                            angle_diff_lidar = abs(lidar_angle[lidar_id_a] % (np.pi * 2) - lidar_angle[lidar_id_b] % (np.pi * 2))
                        
                            angle_diff = np.abs(angle_diff_sky - angle_diff_lidar)
                            angle_diff = min(angle_diff, 2 * np.pi - angle_diff)

                            #angle_diff_sky = self.angle_diff(sky_angle[sky_id_a], sky_angle[sky_id_b])
                            #angle_diff_lidar = self.angle_diff(lidar_angle[lidar_id_a], lidar_angle[lidar_id_b])

                            #angle_diff = abs(angle_diff_sky - angle_diff_lidar)

                            if angle_diff > self.angle_threshold:
                                continue

                            # add valid comparisons to assignment list
                            assignments.add((sky_id_a, lidar_id_a))
                            assignments.add((sky_id_b, lidar_id_b))


            # if no assignments => return infinity
            if not len(assignments):
                return np.inf, np.inf, np.inf, np.inf
            # self.get_logger().info(f"assignments: {assignments}")

            sky_dist_subset = [sky_dist[sky_id_a] for sky_id_a, _ in assignments]
            sky_angle_subset = [sky_angle[sky_id_a] for sky_id_a, _ in assignments]
        
            lidar_dist_subset = [lidar_dist[lidar_id_a] for _, lidar_id_a in assignments]
            lidar_angle_subset = [lidar_angle[lidar_id_a] for _, lidar_id_a in assignments]

            angle_error, scale_error, angle, scale = self.do_registration(sky_dist_subset, sky_angle_subset, lidar_dist_subset, lidar_angle_subset)

            return angle_error, scale_error, angle, scale

    # transformation gets estimated with least squares (point set registration like ICP)
    def do_registration(self, sky_dists, sky_angles, lidar_dists, lidar_angles):
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

        best_angle = angle_sum / angle_divisor
        best_scale = scale_sum / scale_divisor

        angle_error = self.calc_angle_error(best_angle, lidar_angles, sky_angles)
        scale_error = self.calc_distance_error(best_scale, lidar_dists, sky_dists)

        return angle_error, scale_error, best_angle, best_scale


def main():
    main_fn('SunburstRobotCalc', SunburstRobotCalc)


if __name__ == '__main__':
    main()
