from driving_swarm_utils.node import DrivingSwarmNode, main_fn
import functools
import numpy as np

import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


def rotation_matrix(angle):
    """2D rotation matrix (homogeneous 3x3)."""
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])


class ErrorCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.get_list_of_robot_names()

        self.real_robot_pos = []
        self.real_waldo_pos = None
        self.waldo_pos = {}
        self.robot_headings = []

        self.error_pub = {}
        self.last_errors = {}  # Store last computed errors for visualization

        self.create_timer(1.0, self.compute_waldo_errors)

        # Subscriptions for each robot
        for robot in self.robots:
            self.create_subscription(
                Float64MultiArray,
                f"/{robot}/sunburstRobotCalc/waldoPosition",
                functools.partial(self.estimated_waldo_pos_cb, robot),
                10
            )
            self.error_pub[robot] = self.create_publisher(
                Float64MultiArray,
                f"/{robot}/sunburstError",
                10
            )

        self.create_subscription(Float64MultiArray, "/robotPos", self.robot_pos_cb, 10)
        self.create_subscription(Point, "/waldoPos", self.waldo_pos_cb, 10)
        self.create_subscription(Float64MultiArray, "/robotHeadings", self.robot_head_cb, 10)

        # Marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, "/error_calc/markers", 10)

    # --------------------------- Callbacks ---------------------------

    def estimated_waldo_pos_cb(self, robot, msg):
        """Store each robot's estimated Waldo position."""
        self.waldo_pos[robot] = np.array(msg.data)

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

    # --------------------------- Main Computation ---------------------------

    def compute_waldo_errors(self):
        """Compute and visualize Waldo position estimation errors."""
        if (
            self.real_waldo_pos is None
            or len(self.real_robot_pos) == 0
            or len(self.robot_headings) == 0
        ):
            self.get_logger().warn("Missing data: cannot compute errors yet.")
            return

        if len(self.real_robot_pos) != len(self.robots):
            self.get_logger().warn("Robot count mismatch between positions and robot list.")
            return

        for i, robot in enumerate(self.robots):
            if robot not in self.waldo_pos:
                continue

            waldo_true_rel = self.real_waldo_pos - self.real_robot_pos[i]
            waldo_est_rel = self.waldo_pos[robot]

            # Rotate Waldo vector into robot frame using robot heading
            R = rotation_matrix(self.robot_headings[i])

            waldo_est_rel_world = (R @ np.array([waldo_est_rel[0], waldo_est_rel[1], 1]))[:2]


            self.get_logger().info(
                f"[{robot}] heading={np.degrees(self.robot_headings[i]):.1f}°, "
                f"local_est={waldo_est_rel}, "
                f"rotated_est_world={waldo_est_rel_world}, "
                f"true_vec={waldo_true_rel}"
            )

            # Compute Euclidean error
            error = np.linalg.norm(waldo_est_rel_world - waldo_true_rel)

            # Store for visualization
            self.last_errors[robot] = {
                "pos": self.real_robot_pos[i],
                "true_vec": waldo_true_rel,
                "waldo_est_rel_world": waldo_est_rel_world,
                "error": error
            }

            # Publish numeric error
            msg = Float64MultiArray()
            msg.data = [error]
            self.error_pub[robot].publish(msg)

        # Visualize everything
        self.publish_markers()

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
        """Visualize true vs. estimated Waldo direction for each robot."""
        if not self.last_errors:
            return

        marker_array = MarkerArray()
        t = self.get_clock().now().to_msg()

        for i, (robot, info) in enumerate(self.last_errors.items()):
            x, y = info["pos"]
            true_vec = info["true_vec"]
            waldo_est_rel_world = info["waldo_est_rel_world"]

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
                Point(x=x + true_vec[0], y=y + true_vec[1], z=0.0)
            ]
            marker_array.markers.append(true_marker)

            # --- Estimated Waldo vector (red arrow) ---
            est_marker = Marker()
            est_marker.header.frame_id = "map"
            est_marker.header.stamp = t
            est_marker.ns = "estimated_waldo"
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
                Point(x=x + waldo_est_rel_world[0], y=y + waldo_est_rel_world[1], z=0.0)
            ]
            marker_array.markers.append(est_marker)


            # --- Estimated Waldo position in world frame (blue sphere) ---
            est_pos_marker = Marker()
            est_pos_marker.header.frame_id = "map"
            est_pos_marker.header.stamp = t
            est_pos_marker.ns = "estimated_waldo_position"
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
            est_pos_marker.pose.position.x = x + waldo_est_rel_world[0]
            est_pos_marker.pose.position.y = y + waldo_est_rel_world[1]
            est_pos_marker.pose.position.z = 0.0

            marker_array.markers.append(est_pos_marker)

            # --- Robot heading vector (blue arrow) ---
            heading_marker = Marker()
            heading_marker.header.frame_id = "map"
            heading_marker.header.stamp = t
            heading_marker.ns = "robot_heading"
            heading_marker.id = 200 + i
            heading_marker.type = Marker.ARROW
            heading_marker.action = Marker.ADD
            heading_marker.scale.x = 0.05   # arrow length
            heading_marker.scale.y = 0.003  # arrow thickness
            heading_marker.scale.z = 0.003
            heading_marker.color.a = 1.0
            heading_marker.color.b = 1.0    # blue arrow

            # Arrow from robot position in heading direction
            heading_marker.points = [
                Point(x=x, y=y, z=0.0),
                Point(x=x + np.cos(self.robot_headings[i]) * 0.5,
                      y=y + np.sin(self.robot_headings[i]) * 0.5,
                      z=0.0)
            ]
            #marker_array.markers.append(heading_marker)

        # Publish all markers
        self.marker_pub.publish(marker_array)


def main():
    main_fn("ErrorCalc", ErrorCalc)


if __name__ == "__main__":
    main()
