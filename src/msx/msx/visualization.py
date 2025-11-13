from driving_swarm_utils.node import DrivingSwarmNode, main_fn
import functools
import numpy as np

import rclpy
from geometry_msgs.msg import Point
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

        self.create_timer(1.0, self.publish_markers)

        # Subscriptions for each robot
        for robot in self.robots:
            self.create_subscription(
                Float64MultiArray,
                f"/{robot}/sunburstRobotCalc/waldoPosition",
                functools.partial(self.estimated_waldo_pos_cb, robot),
                10
            )

        self.create_subscription(Float64MultiArray, "/robotPos", self.robot_pos_cb, 10)
        self.create_subscription(Point, "/waldoPos", self.waldo_pos_cb, 10)
        self.create_subscription(Float64MultiArray, "/robotHeadings", self.robot_head_cb, 10)

        # Marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, "/visualization/markers", 10)

    # --------------------------- Callbacks ---------------------------

    def estimated_waldo_pos_cb(self, robot, msg):
        """Store each robot's estimated Waldo position."""
        self.est_waldo_pos[robot] = np.array(msg.data)

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

    # --------------------------- Visualization ---------------------------
    # Visualizations summary:
    # ---------------------------------------------------------------------
    # 🟢 Green Arrow   → True vector from each robot to the real Waldo (ground truth)
    # 🔴 Red Arrow     → Estimated Waldo vector rotated into world frame
    # 🔵 Blue Arrow    → Robot heading from simulation (ground truth)
    # 🔵 Blue Sphere   → Estimated Waldo position in the world frame
    #
    # These markers help verify whether the estimated Waldo position
    # and rotation transformations (based on robot heading) are correct.
    # The blue sphere should align closely with the real Waldo position
    # if both estimation and rotation are accurate.
    # ---------------------------------------------------------------------

    def publish_markers(self):
        marker_array = MarkerArray()
        t = self.get_clock().now().to_msg()

        if len(self.robots) != len(self.real_robot_pos):
            self.get_logger().info(f"Not all robot positions available yet...")
            return

        if len(self.robots) != len(self.est_waldo_pos):
            self.get_logger().info(f"Not all robots have estimates for waldo yet...")
            return

        for i, robot in enumerate(self.robots):
            x = self.real_robot_pos[i][0]
            y = self.real_robot_pos[i][1]
            real_waldo_vec_world =  self.real_waldo_pos - self.real_robot_pos[i]
            est_waldo_vec_rob = self.est_waldo_pos[robot] 

            # Rotate Waldo vector into robot frame using robot heading
            R = rotation_matrix_2d(-self.robot_headings[i])
            est_waldo_vec_world = (R @ np.array([est_waldo_vec_rob[0], est_waldo_vec_rob[1]]))


            # --- True Waldo vector (green arrow) ---
            true_marker = Marker()
            true_marker.header.frame_id = "map"
            true_marker.header.stamp = t
            true_marker.ns = "true_waldo"
            true_marker.id = i
            true_marker.type = Marker.ARROW
            true_marker.action = Marker.ADD
            true_marker.scale.x = 0.02  # arrow shaft length scale
            true_marker.scale.y = 0.002  # shaft thickness
            true_marker.scale.z = 0.002
            true_marker.color.a = 1.0
            true_marker.color.g = 1.0

            true_marker.points = [
                Point(x=x, y=y, z=0.0),
                Point(x=x + real_waldo_vec_world[0], y=y + real_waldo_vec_world[1], z=0.0)
            ]
            marker_array.markers.append(true_marker)
            
            # --- Estimated Waldo vector (red arrow) ---
            est_marker = Marker()
            est_marker.header.frame_id = "map"
            est_marker.header.stamp = t
            est_marker.ns = f"estimated_waldo_{robot}"
            est_marker.id = 100 + i
            est_marker.type = Marker.ARROW
            est_marker.action = Marker.ADD
            est_marker.scale.x = 0.02
            est_marker.scale.y = 0.002
            est_marker.scale.z = 0.002
            est_marker.color.a = 1.0
            est_marker.color.r = 1.0

            est_marker.points = [
                Point(x=x, y=y, z=0.0),
                Point(x=x + est_waldo_vec_world[0], y=y + est_waldo_vec_world[1], z=0.0)
            ]
            marker_array.markers.append(est_marker)
            
            
            # --- Estimated Waldo position in world frame (blue sphere) ---
            est_pos_marker = Marker()
            est_pos_marker.header.frame_id = "map"
            est_pos_marker.header.stamp = t
            est_pos_marker.ns = f"{robot}-waldo-pose"
            est_pos_marker.id = 400 + i
            est_pos_marker.type = Marker.SPHERE
            est_pos_marker.action = Marker.ADD
            est_pos_marker.scale.x = 0.1
            est_pos_marker.scale.y = 0.1
            est_pos_marker.scale.z = 0.1
            est_pos_marker.color.a = 1.0
            est_pos_marker.color.r = 0.0
            est_pos_marker.color.g = 0.0
            est_pos_marker.color.b = 1.0
            est_pos_marker.pose.position.x = x + est_waldo_vec_world[0]
            est_pos_marker.pose.position.y = y + est_waldo_vec_world[1]
            est_pos_marker.pose.position.z = 0.0

            marker_array.markers.append(est_pos_marker)
            #
            # # --- Robot heading vector (blue arrow) ---
            # heading_marker = Marker()
            # heading_marker.header.frame_id = "map"
            # heading_marker.header.stamp = t
            # heading_marker.ns = "robot_heading"
            # heading_marker.id = 200 + i
            # heading_marker.type = Marker.ARROW
            # heading_marker.action = Marker.ADD
            # heading_marker.scale.x = 0.05   # arrow length
            # heading_marker.scale.y = 0.003  # arrow thickness
            # heading_marker.scale.z = 0.003
            # heading_marker.color.a = 1.0
            # heading_marker.color.b = 1.0    # blue arrow
            #
            # # Arrow from robot position in heading direction
            # heading_marker.points = [
            #     Point(x=x, y=y, z=0.0),
            #     Point(x=x + np.cos(self.robot_headings[i]) * 0.5,
            #           y=y + np.sin(self.robot_headings[i]) * 0.5,
            #           z=0.0)
            # ]
            # #marker_array.markers.append(heading_marker)

        # Publish all markers
        self.marker_pub.publish(marker_array)


def main():
    main_fn("Visualization", Visualization)


if __name__ == "__main__":
    main()
