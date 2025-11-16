from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

class GTFormatting(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        # Fills self.robots with the names of the robots
        self.get_list_of_robot_names()

        self.model_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_cb,
            10
        )

        self.robot_pos_pub = self.create_publisher(Float64MultiArray, "/robotPos", 100)
        self.waldo_pos_pub = self.create_publisher(Point, "/waldoPos", 100)
        self.robot_head_pub = self.create_publisher(Float64MultiArray, "/robotHeadings", 100)

    def quat_to_angle(self, quat):
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    def model_cb(self, msg: ModelStates):
        # self.get_logger().info(f"Models: {msg.name}")
        # self.get_logger().info(f"Robots: {self.robots}")

        # msg contains the information (name, pose, and twist) of objects in the gazebo simulation
        # we only care about the information on waldo and the robots
        # we want to ensure that information on all the robots and waldo is contained within the msg before proceeding

        remove_idxs = [idx for idx in range(len(msg.name)) if "waldo" not in msg.name[idx] and "robot" not in msg.name[idx]]
        msg.name = [x for i, x in enumerate(msg.name) if i not in remove_idxs]
        msg.pose = [x for i, x in enumerate(msg.pose) if i not in remove_idxs]
        msg.twist = [x for i, x in enumerate(msg.twist) if i not in remove_idxs]

        # self.get_logger().info(f"Models*: {msg.name}")

        # Ensure the msg contains information on all the robots and waldo
        if len(msg.name) == len(self.robots) + 1:
            robot_positions = [None for _ in range(len(self.robots))]
            robot_headings = [None for _ in range(len(self.robots))]

            # Find waldo in the msg
            waldo_idx = next((i for i, item in enumerate(msg.name) if "waldo" in item), -1)

            # TODO: should pass, remove
            assert waldo_idx >= 0

            # Store waldo position
            waldo_position = msg.pose[waldo_idx].position

            # Remove waldo data from message so all that's left is the robots
            msg.name.pop(waldo_idx)
            msg.pose.pop(waldo_idx)
            msg.twist.pop(waldo_idx)

            # self.get_logger().info(f"Models**: {msg.name}")

            # TODO: should pass, remove
            assert len(msg.name) == len(self.robots)

            for idx, name in enumerate(msg.name):
                pose = msg.pose[idx]

                # Get the index of the current robot being considered, this is to ensure the order of positions matches
                # the rest of the program
                try:
                    name_idx = self.robots.index(name)
                except ValueError:
                    self.get_logger().warn("Robot name '{}' not found in list of names.".format(name))
                    name_idx = -1
                    raise # TODO: remove this and handle the exception

                robot_positions[name_idx] = [pose.position.x, pose.position.y]
                robot_headings[name_idx] = self.quat_to_angle(pose.orientation)

            if not robot_positions:
                return

            # ---- robot positions ---
            arr = np.array(robot_positions)

            robot_pos_msg = Float64MultiArray()
            robot_pos_msg.layout.dim.append(MultiArrayDimension(
                label="robots",
                size=arr.shape[0],
                stride=arr.shape[0] * arr.shape[1]
            ))
            robot_pos_msg.layout.dim.append(MultiArrayDimension(
                label="xy",
                size=arr.shape[1],
                stride=arr.shape[1]
            ))

            robot_pos_msg.data = arr.flatten().tolist()
            robot_headings = np.array(robot_headings)
            heading_msg = Float64MultiArray()
            heading_msg.data = robot_headings.flatten().tolist()

            self.robot_pos_pub.publish(robot_pos_msg)
            self.robot_head_pub.publish(heading_msg)

            # --- waldo position ---
            if waldo_position is not None:
                waldo_msg = Point()
                waldo_msg.x = waldo_position.x
                waldo_msg.y = waldo_position.y
                waldo_msg.z = 0.0
                self.waldo_pos_pub.publish(waldo_msg)
                #print(f"Published : {waldo_msg}")
        else:
            pass

def main():
    main_fn('GTFormatting', GTFormatting)

if __name__ == '__main__':
     main()     
