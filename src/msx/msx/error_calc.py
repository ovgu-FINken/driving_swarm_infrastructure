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

def rotation_matrix_2d(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]])


class ErrorCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.get_list_of_robot_names()

        self.real_robot_pos = []
        self.real_waldo_pos = None
        self.waldo_pos = {}
        self.robot_headings = []

        self.error_pub = {}

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

            real_waldo_vec_world = self.real_waldo_pos - self.real_robot_pos[i]
            est_waldo_vec_rob = self.waldo_pos[robot]

            # Rotate Waldo vector into robot frame using robot heading
            #R = rotation_matrix(self.robot_headings[i])
            R = rotation_matrix_2d(-self.robot_headings[i])
            #est_waldo_vec_world = (R @ np.array([est_waldo_vec_rob[0], est_waldo_vec_rob[1], 1]))[:2]
            est_waldo_vec_world = (R @ np.array([est_waldo_vec_rob[0], est_waldo_vec_rob[1]]))

            # Compute Euclidean error
            error = np.linalg.norm(est_waldo_vec_world - real_waldo_vec_world)

            self.get_logger().info(
                f"[{robot}] heading={np.degrees(self.robot_headings[i]):.1f}°, "
                f"local_est={est_waldo_vec_rob}, "
                f"rotated_est_world={est_waldo_vec_world}, "
                f"real_waldo_vec_world={real_waldo_vec_world}, "
                f"error={error}"
            )

            # Publish numeric error
            msg = Float64MultiArray()
            msg.data = [error]
            self.error_pub[robot].publish(msg)


def main():
    main_fn("ErrorCalc", ErrorCalc)


if __name__ == "__main__":
    main()
