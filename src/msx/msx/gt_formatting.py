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
        robot_positions = []
        waldo_position = None
        robot_headings = []

        for idx, name in enumerate(msg.name):
            pose = msg.pose[idx]
            if "robot" in name:
                robot_positions.append([pose.position.x, pose.position.y])
                robot_headings.append(self.quat_to_angle(pose.orientation))
            elif name == "waldo":
                waldo_position = pose.position

        if not robot_positions:
            return


        # ---- robot positions ---
        arr = np.array(robot_positions)

        msg_out = Float64MultiArray()
        msg_out.layout.dim.append(MultiArrayDimension(
            label="robots",
            size=arr.shape[0],
            stride=arr.shape[0] * arr.shape[1]
        ))
        msg_out.layout.dim.append(MultiArrayDimension(
            label="xy",
            size=arr.shape[1],
            stride=arr.shape[1]
        ))

        msg_out.data = arr.flatten().tolist()
        robot_headings = np.array(robot_headings)
        heading_msg = Float64MultiArray()
        heading_msg.data = robot_headings.flatten().tolist()

        self.robot_pos_pub.publish(msg_out)
        self.robot_head_pub.publish(heading_msg)

        # --- waldo position ---
        if waldo_position is not None:
            waldo_msg = Point() 
            waldo_msg.x = waldo_position.x
            waldo_msg.y = waldo_position.y
            waldo_msg.z = 0.0
            self.waldo_pos_pub.publish(waldo_msg)
            #print(f"Published : {waldo_msg}")

        # in other node:
        # data = np.array(msg.data
        #arr = data.reshape((-1, 2))   # Nx2-Matrix: N robots, 2 columns (x,y)
        #x_i, y_i = arr[i, 0], arr[i, 1]

def main():
    main_fn('GTFormatting', GTFormatting)

if __name__ == '__main__':
     main()     
