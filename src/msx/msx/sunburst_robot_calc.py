#!/usr/bin/env python3
import rclpy
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from std_msgs.msg import Float64MultiArray, String, Int32
from sensor_msgs.msg import LaserScan, PointCloud
from driving_swarm_utils.utils import detect_tb_from_ranges
from geometry_msgs.msg import Point32

from enum import Enum
import numpy as np
import math


class LikelihoodFunction(Enum):
    LINEAR_CLAMPED = 1
    NEGATIVE_LOG_CLAMPED = 2
    GAUSS_CLAMPED = 3


class SunburstRobotCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        # Set the logger level
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # ===== Declare parameters with default values =====
        self.declare_parameter("debug", False)
        self.declare_parameter("use_bayes_filter", True)

        # Threshold values
        self.declare_parameter("dist_threshold", 1.05)
        self.declare_parameter("angle_threshold", np.pi / 12)

        # Weights for weighted error sum calculation
        self.declare_parameter("scale_error_weight", 1.0)
        self.declare_parameter("angle_error_weight", 1.0)

        # Bayes filter likelihood settings
        self.declare_parameter("likelihood_function", "GAUSS_CLAMPED")
        self.declare_parameter("likelihood_clamp", 0.01)

        # Linear clamped
        self.declare_parameter("a_1", 1.0)

        # Negative log clamped
        self.declare_parameter("a_2", 3.0)

        # Gaussian clamped
        self.declare_parameter("a_3", 1.0)
        self.declare_parameter("m", 0.0)
        self.declare_parameter("s", 0.05)

        # Penalty scale for stale error values
        self.declare_parameter("old_error_penalty_scale", 1.05)

        # ===== Load parameter values =====
        self.DEBUG = self.get_parameter("debug").value
        self.USE_BAYES_FILTER = self.get_parameter("use_bayes_filter").value

        self.dist_threshold = self.get_parameter("dist_threshold").value
        self.angle_threshold = self.get_parameter("angle_threshold").value

        self.scale_error_weight = self.get_parameter("scale_error_weight").value
        self.angle_error_weight = self.get_parameter("angle_error_weight").value

        self.LIKELIHOOD_FUNCTION = LikelihoodFunction[
            self.get_parameter("likelihood_function").value
        ]

        self.likelihood_clamp = self.get_parameter("likelihood_clamp").value

        self.a_1 = self.get_parameter("a_1").value
        self.a_2 = self.get_parameter("a_2").value
        self.a_3 = self.get_parameter("a_3").value

        self.m = self.get_parameter("m").value
        self.s = self.get_parameter("s").value

        self.old_error_penalty_scale = self.get_parameter(
            "old_error_penalty_scale"
        ).value

        # print to validate parameters set successfully
        self.get_logger().info(
            "\n"
            "========== Parameter Configuration ==========\n"
            f"debug                    : {self.DEBUG}\n"
            f"use_bayes_filter         : {self.USE_BAYES_FILTER}\n"
            "\n"
            "Thresholds:\n"
            f"  dist_threshold         : {self.dist_threshold:.1f}\n"
            f"  angle_threshold        : {self.angle_threshold:.6f}\n"
            "\n"
            "Weights:\n"
            f"  scale_error_weight     : {self.scale_error_weight:.1f}\n"
            f"  angle_error_weight     : {self.angle_error_weight:.1f}\n"
            "\n"
            "Likelihood:\n"
            f"  function               : {self.LIKELIHOOD_FUNCTION.name}\n"
            f"  likelihood_clamp       : {self.likelihood_clamp:.3f}\n"
            f"  a_1                    : {self.a_1:.2f}\n"
            f"  a_2                    : {self.a_2:.2f}\n"
            f"  a_3                    : {self.a_3:.2f}\n"
            f"  m                      : {self.m:.2f}\n"
            f"  s                      : {self.s:.2f}\n"
            "\n"
            "Penalty:\n"
            f"  old_error_penalty_scale: {self.old_error_penalty_scale:.6f}\n"
            "============================================"
        )

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

        # This method populates the list self.robots with the names of the robots.
        self.get_list_of_robot_names()

        # ===== Subscriptions =====
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
            self.DEBUG_cur_robot_idx = self.robots.index(self.own_name)

        # ===== Publishers =====
        self.waldo_pub = self.create_publisher(Float64MultiArray, "sunburstRobotCalc/waldoPosition", 10)

        self.pub_lidar_data = self.create_publisher(PointCloud, "lidarData", 10)
        self.lidar_detection_count_pub = self.create_publisher(Int32, "num_detections", 10)
        self.identity_pub = self.create_publisher(String, "identify_as", 10)

        self.posterior_pub = self.create_publisher(Float64MultiArray, "posterior", 10)

        self.likelihoods_pub = self.create_publisher(Float64MultiArray, "likelihoods", 10)

        self.create_timer(0.1, self.calc_timer)

    # =============================================
    # ================= Callbacks =================
    # =============================================

    def waldo_cb(self, msg: Float64MultiArray):
        data = np.array(msg.data).reshape((-1, 2))

        self.skyview_waldo_distances = data[:, 0]
        self.skyview_waldo_angles = data[:, 1]

    def laser_cb(self, msg):
        ranges = msg.ranges
        ranges = [x if x > msg.range_min and x < msg.range_max else float('inf') for x in ranges]

        # Using the x-axis of the robot (aka the first value from the msg.ranges)
        # The output of the positions is relative to the current position of the laser scanner
        # TODO: at this point a laser scan position filter could be implemented

        # ===== Params =====
        #
        # ranges, px=0.0, py=0.0, pt=0.0, angle_min=0.0, angle_increment=1.0, cluster_linkage_threshold=0.15,
        # cluster_range_threshold=1.5, cluster_size_threshold=30
        if self.DEBUG == False:
            self.lidar_data = detect_tb_from_ranges(ranges, 0.0, 0.0, 0.0, msg.angle_min, msg.angle_increment)

        # self.get_logger().info(f"Lidar data : {self.lidar_data}")

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
        # We don't have a value for the current robot headings or aren't debugging
        if self.DEBUG_cur_robot_heading is None or self.DEBUG == False:
            return

        # self.get_logger().info(f"groundtruth_pos_callback, msg: {msg}")

        flat = msg.data
        pairs = [[flat[i], flat[i + 1]] for i in range(0, len(flat), 2)]

        cur_robot_position = np.array(pairs[self.DEBUG_cur_robot_idx])

        # Rotating global groundtruth data to be in the robot's reference frame
        self.lidar_data = []
        for i in range(len(self.robots)):
            if i != self.DEBUG_cur_robot_idx:
                R = self.rotation_matrix_2d(-self.DEBUG_cur_robot_heading)
                self.lidar_data.append(R @ (pairs[i] - cur_robot_position))

        # self.get_logger().info(f"I am {cur_robot_idx} and am at {cur_robot_position} facing {self.DEBUG_cur_robot_heading}; all robots are at {pairs}; in my frame they are at: {self.lidar_data}")
        return

    # Get the robots heading
    def groundtruth_heading_callback(self, msg):
        if self.DEBUG == False:
            return

        flat = msg.data

        self.DEBUG_cur_robot_heading = flat[self.DEBUG_cur_robot_idx]
        return

    # =============================================
    # ================== Helpers ==================
    # =============================================

    def rotation_matrix_2d(self, angle):
        return np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]])

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

    def negative_log(self, one_error):
        return max(np.exp(-self.a_2 * one_error), self.likelihood_clamp)

    def linear_clamped(self, one_error):
        return max(-self.a_1 * one_error + 1, self.likelihood_clamp)

    def gauss_clamped(self, one_error):
        return max(self.a_3 * math.exp(-((one_error - self.m) ** 2) / (2 * self.s ** 2)), self.likelihood_clamp)

    def calc_likelihood(self):
        vals = np.array([])
        likelihoods = np.array([])
        val = 0

        for id, err in self.bayes_errors.items():
            if self.LIKELIHOOD_FUNCTION == LikelihoodFunction.NEGATIVE_LOG_CLAMPED:
                likelihood = self.negative_log(err)
                val = likelihood * self.last_probs[id]
            elif self.LIKELIHOOD_FUNCTION == LikelihoodFunction.LINEAR_CLAMPED:
                likelihood = self.linear_clamped(err)
                val = likelihood * self.last_probs[id]
            elif self.LIKELIHOOD_FUNCTION == LikelihoodFunction.GAUSS_CLAMPED:
                likelihood = self.gauss_clamped(err)
                val = likelihood * self.last_probs[id]

            vals = np.append(vals, val)
            likelihoods = np.append(likelihoods, likelihood)
            # vals = np.append(vals, val)

        return vals / sum(vals), likelihoods

    # =============================================
    # =============== Calculations ================
    # =============================================

    def calc_timer(self):
        # Wait for data and to ensure the math won't have a division by zero error
        if len(self.skyview_angles) == 0 or len(self.skyview_distances) == 0:
            return
        # Check if we have at least three robots in roofcam sunburst
        if len(self.skyview_angles) < 2 or len(self.skyview_distances) < 2:
            self.get_logger().info("ERROR: skipping\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
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
            self.bayes_vals = {k: [0, 0] for k in range(len(self.skyview_angles))}

        # For every sunburst sent from the drone (roof cam)
        for rbt_idx, _ in enumerate(self.skyview_angles):
            # This returns np.inf for both errors if there are less than 2 skyview detections, less than 2 lidar detections, or no assignments were made.
            angle_error, scale_error, angle, scale = self.sunburst_single_robot(self.skyview_distances[rbt_idx],
                                                                                self.skyview_angles[rbt_idx],
                                                                                lidar_distances, lidar_angles)
            if angle_error != np.inf and scale_error != np.inf:
                errors[rbt_idx] = self.angle_error_weight * angle_error + self.scale_error_weight * scale_error
                vals[rbt_idx] = [angle, scale]
            # elif self.USE_BAYES_FILTER:
            # errors[rbt_idx] = np.inf
            # vals[rbt_idx] = [angle, scale]

        waldo_angle = None

        # If we had at least one identification pass the thresholds for any of the skyview sunbursts
        if len(errors):
            if self.USE_BAYES_FILTER:
                self.get_logger().info(f"Bayes filter error: {errors}")

                # If this is the first time we calculate probabilities, initialize last_probs uniformly
                if len(self.cur_probs) == 0:
                    self.last_probs = {k: 1 / len(self.skyview_angles) for k in range(len(self.skyview_angles))}

                # Grab the new errors calculated and save them
                for id, err in errors.items():
                    self.bayes_errors[id] = err
                    self.bayes_vals[id] = [vals[id][0], vals[id][1]]

                # Penalize the error values that haven't been updated this iteration
                for id in range(len(self.cur_probs)):
                    if id not in errors:
                        self.bayes_errors[id] *= self.old_error_penalty_scale

                new_probs, cur_likelihoods = self.calc_likelihood()
                for id, prob in enumerate(new_probs):
                    self.cur_probs[id] = prob

                self.last_probs = self.cur_probs.copy()

                posterior_probs = Float64MultiArray()
                posterior_probs.data = [self.cur_probs[id] for id in range(len(self.cur_probs))]
                self.posterior_pub.publish(posterior_probs)

                current_likelihoods = Float64MultiArray()
                current_likelihoods.data = [cur_likelihoods[id] for id in range(len(self.cur_probs))]
                self.likelihoods_pub.publish(current_likelihoods)

                my_rbt_idx = max(self.cur_probs, key=self.cur_probs.get)
                self.get_logger().info(f"Probs: {self.bayes_errors} -> {self.cur_probs} -> {my_rbt_idx}")

                waldo_angle = self.skyview_waldo_angles[my_rbt_idx] - self.bayes_vals[my_rbt_idx][0]
                waldo_distance = self.skyview_waldo_distances[my_rbt_idx] * self.bayes_vals[my_rbt_idx][1]
            # Not using a Bayesian filter
            else:
                my_rbt_idx = min(errors, key=errors.get)

                waldo_angle = self.skyview_waldo_angles[my_rbt_idx] - vals[my_rbt_idx][0]
                waldo_distance = self.skyview_waldo_distances[my_rbt_idx] * vals[my_rbt_idx][1]

                posterior_probs = Float64MultiArray()
                tmp = [0.0 for _ in range(5)]
                tmp[my_rbt_idx] = 1.0
                posterior_probs.data = tmp
                self.posterior_pub.publish(posterior_probs)

        # Publish
        if waldo_angle is not None:
            waldo_x = waldo_distance * math.cos(waldo_angle)
            waldo_y = waldo_distance * math.sin(waldo_angle)

            self.get_logger().info(
                f"idx, angle, dist: {my_rbt_idx}, {self.skyview_waldo_angles[my_rbt_idx]}, {self.skyview_waldo_distances[my_rbt_idx]}")

            # publish waldos min error position in the robot's reference frame
            min_error_waldo_pos = Float64MultiArray()
            min_error_waldo_pos.data = [waldo_x, waldo_y]
            self.waldo_pub.publish(min_error_waldo_pos)

            # publish estimated robot identity with min error
            min_error_robot_id = String()
            min_error_robot_id.data = str(self.robots[my_rbt_idx])
            self.identity_pub.publish(min_error_robot_id)

            # publish lidar detection count
            self.lidar_detection_count_pub.publish(Int32(data=int(len(lidar_angles))))

    # Filters out every comparison that is above a given threshold
    def sunburst_single_robot(self, sky_dist, sky_angle, lidar_dist, lidar_angle):
        assignments = set()
        # For every pair of sky_ids
        for sky_id_a, sky_dist_a in enumerate(sky_dist):
            for sky_id_b, sky_dist_b in enumerate(sky_dist):

                # Skip if we compare same skyview values and make sure each pairing is only checked once
                if sky_id_a <= sky_id_b:
                    continue

                # For every pair of lidar_ids
                for lidar_id_a, lidar_dist_a in enumerate(lidar_dist):
                    for lidar_id_b, lidar_dist_b in enumerate(lidar_dist):

                        # Skip if we compare same lidar values and make sure each pairing is only checked once
                        if lidar_id_a <= lidar_id_b:
                            continue

                        # filter out robot if distance discrepancy is too high
                        distance_diff_sky = sky_dist_a / sky_dist_b
                        distance_diff_lidar = lidar_dist_a / lidar_dist_b
                        distance_diff = max(distance_diff_sky, distance_diff_lidar) / min(distance_diff_sky,
                                                                                          distance_diff_lidar)

                        if distance_diff > self.dist_threshold:
                            continue

                        # filter out robot if angle discrepancy is too high
                        diff = (sky_angle[sky_id_a] - sky_angle[sky_id_b]) % (np.pi * 2)
                        angle_diff_sky = min(diff, np.pi * 2 - diff)
                        diff = (lidar_angle[lidar_id_a] - lidar_angle[lidar_id_b]) % (np.pi * 2)
                        angle_diff_lidar = min(diff, np.pi * 2 - diff)

                        angle_diff = np.abs(angle_diff_sky - angle_diff_lidar)

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

        angle_error, scale_error, angle, scale = self.do_registration(sky_dist_subset, sky_angle_subset,
                                                                      lidar_dist_subset, lidar_angle_subset)

        return angle_error, scale_error, angle, scale

    # transformation gets estimated with least squares (point set registration like ICP)
    def do_registration(self, sky_dists, sky_angles, lidar_dists, lidar_angles):
        scale_sum = 0
        scale_divisor = 0

        angle_sum = 0
        angle_divisor = 0

        for sky_dist, sky_angle, lidar_dist, lidar_angle in zip(sky_dists, sky_angles, lidar_dists,
                                                                lidar_angles):  # For each assignment
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
