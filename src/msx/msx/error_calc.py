
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
import functools
import numpy as np

class ErrorCalc(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.get_list_of_robot_names()

        # simulated real positions
        self.real_robot_pos = [] 
        self.real_waldo_pos = None

        # estimated positions out of every robots view as dictionary
        # waldo_pos[robotA] = (x, y)
        self.waldo_pos = {} 

        self.create_timer(1.0, self.compute_waldo_errors)

        for robot in self.robots:
            self.create_subscription(Float64MultiArray, f"/{robot}/sunburstRobotCalc/waldoPosition", functools.partial(self.estimated_waldo_pos_cb, robot), 10)

        self.robot_pos_sub = self.create_subscription(
            Float64MultiArray, "/robotPos", self.robot_pos_cb, 10
        )
        self.waldo_pos_sub = self.create_subscription(
            Point, "/waldoPos", self.waldo_pos_cb, 10
        )

    def estimated_waldo_pos_cb(self, robot, msg):
        self.get_logger().info(f'got msg for {robot}: {msg}')
        data = np.array(msg.data)
        self.waldo_pos[robot] = data

    def robot_pos_cb(self, msg):
        # Convert Float64MultiArray back to Nx2 matrix
        data = np.array(msg.data)
        if len(data) % 2 != 0:
            self.get_logger().warn("Robot position data size is not even — ignoring message")
            return

        self.real_robot_pos = data.reshape((-1, 2))

    def waldo_pos_cb(self, msg):
        # msg is geometry_msgs/Point
        self.real_waldo_pos = np.array([msg.x, msg.y])

    def compute_waldo_errors(self):
        """
        Compute and log the distance error between each robot's estimated Waldo position
        and the true relative Waldo position seen from that robot.

        The true relative Waldo position is defined as:
            waldo_true_rel = real_waldo_pos - real_robot_pos[i]

        Each robot's estimate (stored in self.waldo_pos[robot]) is compared to this true
        relative position. The Euclidean distance between the two is logged for analysis.
        """

        # Check that all necessary data is available
        if self.real_waldo_pos is None or len(self.real_robot_pos) == 0:
            self.get_logger().warn("Missing real robot or Waldo positions — cannot compute errors.")
            return

        # Ensure the number of robots matches the known list
        if len(self.real_robot_pos) != len(self.robots):
            self.get_logger().warn("Robot count mismatch between real positions and robot list.")
            return

        for i, robot in enumerate(self.robots):
            if robot not in self.waldo_pos:
                self.get_logger().warn(f"No estimated Waldo position received yet for {robot}.")
                continue

            # Real relative Waldo position from robot's perspective
            waldo_true_rel = self.real_waldo_pos - self.real_robot_pos[i]

            # Estimated Waldo position from the robot
            waldo_est_rel = self.waldo_pos[robot]

            # Compute Euclidean distance error
            error = np.linalg.norm(waldo_est_rel - waldo_true_rel)

            self.get_logger().info(
                f"[{robot}] Estimated relative Waldo position: {waldo_est_rel}, "
                f"True: {waldo_true_rel}, Error: {error:.3f}"
            )

def main():
    main_fn('ErrorCalc', ErrorCalc)

if __name__ == '__main__':
    main()
