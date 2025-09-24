from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

class MSXMockCamNode(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.listener_callback,
            10
        )

        self.pub = self.create_publisher(Float64MultiArray, "/robot_pos/data", 100)

    def listener_callback(self, msg: ModelStates):
        robot_positions = []
        for idx, name in enumerate(msg.name):
            if "robot" in name:
                pose = msg.pose[idx]
                robot_positions.append([pose.position.x, pose.position.y])

        if not robot_positions:
            return

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

        self.pub.publish(msg_out)

        # in other node:
        # data = np.array(msg.data
        #arr = data.reshape((-1, 2))   # Nx2-Matrix: N robots, 2 columns (x,y)
        #x_i, y_i = arr[i, 0], arr[i, 1]

def main():
    main_fn('MSXMockCamNode', MSXMockCamNode)

if __name__ == '__main__':
     main()     