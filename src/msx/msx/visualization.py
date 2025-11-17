from driving_swarm_utils.node import DrivingSwarmNode, main_fn
import functools
import numpy as np

import rclpy
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.msg import ModelStates

def rotation_matrix_2d(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]])


class Visualization(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.get_list_of_robot_names()

        self.real_robot_pos = []
        self.real_waldo_pos = None
        self.est_waldo_pos = {}
        self.robot_headings = []
        self.lidar_data = {}
        self.skyview_distances = []
        self.skyview_angles = []

        self.create_timer(1.0, self.publish_markers)

        # Subscriptions for each robot
        for robot in self.robots:
            self.create_subscription(
                Float64MultiArray,
                f"/{robot}/sunburstRobotCalc/waldoPosition",
                functools.partial(self.estimated_waldo_pos_cb, robot),
                10
            )
            self.create_subscription(
                PointCloud,
                f"/{robot}/lidarData",
                functools.partial(self.lidar_data_cb, robot),
                10
            )
        
        self.create_subscription(
            Float64MultiArray,
            '/sunburstSkyview/data',
            self.skyview_cb,
            10)
        self.create_subscription(Float64MultiArray, "/robotPos", self.robot_pos_cb, 10)
        self.create_subscription(Point, "/waldoPos", self.waldo_pos_cb, 10)
        self.create_subscription(Float64MultiArray, "/robotHeadings", self.robot_head_cb, 10)

        # Marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, "/visualization/markers", 10)

    # --------------------------- Callbacks ---------------------------

    def estimated_waldo_pos_cb(self, robot, msg):
        """Store each robot's estimated Waldo position."""
        self.est_waldo_pos[robot] = np.array(msg.data)

    def lidar_data_cb(self, robot, msg):
        self.lidar_data[robot] = np.array([[p.x, p.y] for p in msg.points])

    def robot_pos_cb(self, msg):
        """Store real robot positions as Nx2 matrix."""
        data = np.array(msg.data)
        if len(data) % 2 != 0:
            self.get_logger().warn("Robot position array length is not even.")
            return
        self.real_robot_pos = data.reshape((-1, 2))

    def waldo_pos_cb(self, msg):
        """Store Waldo's real position."""
        self.real_waldo_pos = np.array([msg.x, msg.y])

    def robot_head_cb(self, msg):
        """Store robot heading angles (in radians)."""
        self.robot_headings = np.array(msg.data)

    def skyview_cb(self, msg: Float64MultiArray):
        dims = msg.layout.dim
        num_robots = dims[0].size
        max_neighbors = dims[1].size
        feature_size = dims[2].size

        # --- reshape flat data back into 3D array ---
        full_array = np.array(msg.data).reshape((num_robots, max_neighbors, feature_size))

        # --- separate into distances and angles arrays for convenience ---
        self.skyview_distances = full_array[:, :, 0]  # first feature is distance
        self.skyview_angles = full_array[:, :, 1]  # second feature is angle

    # --------------------------- Visualization ---------------------------
    # Visualizations summary:
    # ---------------------------------------------------------------------
    # 🟢 Green Arrow   → Ground truth vector from robot to Waldo
    # 🔵 Blue Arrow    → Estimated Waldo vector transformed into world frame
    # 🔵 Blue Sphere   → Estimated Waldo position in world frame
    # 🟢 Green Sphere  → Ground truth robot position
    # 🔴 Red Sphere    → Estimated other robots' positions from lidar data
    # These markers are used to verify whether the Waldo estimation and
    # frame transformations are correct.
    # ---------------------------------------------------------------------

    def publish_markers(self):
        marker_array = MarkerArray()
        t = self.get_clock().now().to_msg()

        # Ensure required data is available
        if len(self.robots) != len(self.real_robot_pos):
            self.get_logger().info("Not all robot positions available yet...")
            return

        if len(self.robots) != len(self.est_waldo_pos):
            self.get_logger().info("Not all robots have Waldo estimates yet...")
            return

        for i, robot in enumerate(self.robots):
            x, y = self.real_robot_pos[i]

            # Ground truth vector from robot to Waldo
            real_vec = self.real_waldo_pos - self.real_robot_pos[i]

            # Estimated Waldo vector in robot frame
            est_vec_robot = self.est_waldo_pos[robot]

            # Rotate estimate into world frame
            R = rotation_matrix_2d(-self.robot_headings[i])
            est_vec_world = R @ np.array([est_vec_robot[0], est_vec_robot[1]])

            # --- Ground truth Waldo vector (green arrow) ---
            true_mark = Marker()
            true_mark.header.frame_id = "map"
            true_mark.header.stamp = t
            true_mark.ns = "true_waldo"
            true_mark.id = i
            true_mark.type = Marker.ARROW
            true_mark.action = Marker.ADD
            true_mark.scale.x = 0.02
            true_mark.scale.y = 0.002
            true_mark.scale.z = 0.002
            true_mark.color.a = 1.0
            true_mark.color.g = 1.0
            true_mark.points = [
                Point(x=x, y=y, z=0.0),
                Point(x=x + real_vec[0], y=y + real_vec[1], z=0.0)
            ]
            marker_array.markers.append(true_mark)

            # --- Estimated Waldo vector (blue arrow) ---
            est_mark = Marker()
            est_mark.header.frame_id = "map"
            est_mark.header.stamp = t
            est_mark.ns = f"estimated_waldo_{robot}"
            est_mark.id = 100 + i
            est_mark.type = Marker.ARROW
            est_mark.action = Marker.ADD
            est_mark.scale.x = 0.02
            est_mark.scale.y = 0.002
            est_mark.scale.z = 0.002
            est_mark.color.a = 1.0
            est_mark.color.b = 1.0
            est_mark.points = [
                Point(x=x, y=y, z=0.0),
                Point(x=x + est_vec_world[0], y=y + est_vec_world[1], z=0.0)
            ]
            marker_array.markers.append(est_mark)

            # --- Estimated Waldo position (blue sphere) ---
            est_pos = Marker()
            est_pos.header.frame_id = "map"
            est_pos.header.stamp = t
            est_pos.ns = f"{robot}-waldo-pos"
            est_pos.id = 400 + i
            est_pos.type = Marker.SPHERE
            est_pos.action = Marker.ADD
            est_pos.scale.x = est_pos.scale.y = est_pos.scale.z = 0.1
            est_pos.color.a = 1.0
            est_pos.color.b = 1.0
            est_pos.pose.position.x = x + est_vec_world[0]
            est_pos.pose.position.y = y + est_vec_world[1]
            est_pos.pose.position.z = 0.0
            marker_array.markers.append(est_pos)

            # --- Ground truth robot position (green sphere) ---
            real_robot = Marker()
            real_robot.header.frame_id = "map"
            real_robot.header.stamp = t
            real_robot.ns = f"{robot}-real-pos"
            real_robot.id = 500 + i
            real_robot.type = Marker.SPHERE
            real_robot.action = Marker.ADD
            real_robot.scale.x = real_robot.scale.y = real_robot.scale.z = 0.1
            real_robot.color.a = 1.0
            real_robot.color.g = 1.0
            real_robot.pose.position.x = x
            real_robot.pose.position.y = y
            real_robot.pose.position.z = 0.0
            marker_array.markers.append(real_robot)

            # --- Estimated other robots from lidar (red spheres) ---
            for j, pos in enumerate(self.lidar_data[robot]):
                pos_world = R @ np.array([pos[0], pos[1]])

                lidar_mark = Marker()
                lidar_mark.header.frame_id = "map"
                lidar_mark.header.stamp = t
                lidar_mark.ns = f"lidar_clusters_{robot}"
                lidar_mark.id = 600 + i * 10 + j
                lidar_mark.type = Marker.SPHERE
                lidar_mark.action = Marker.ADD
                lidar_mark.scale.x = lidar_mark.scale.y = lidar_mark.scale.z = 0.2
                lidar_mark.color.a = 1.0
                lidar_mark.color.r = 1.0
                lidar_mark.color.g = 0.2
                lidar_mark.color.b = 0.2
                lidar_mark.pose.position.x = x + pos_world[0]
                lidar_mark.pose.position.y = y + pos_world[1]
                lidar_mark.pose.position.z = 0.0
                marker_array.markers.append(lidar_mark)

        # Publish all markers
        self.marker_pub.publish(marker_array)

def main():
    main_fn("Visualization", Visualization)


if __name__ == "__main__":
    main()
