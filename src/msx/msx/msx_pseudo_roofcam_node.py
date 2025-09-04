from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import numpy as np

class MSXPseudoRoofcamNode(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.phi = np.array([])
        self.alpha = np.array([])
        self.t = np.zeros((1, 2))

        self.robot_publishers = {}
        self.pub_all = self.create_publisher(Float32MultiArray, "/roof_cam_view", 10)

        self.subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.listener_callback,
            10
        )


    def listener_callback(self, msg: ModelStates):
        for idx, name in enumerate(msg.name):
            if "robot" in name: 
                pose = msg.pose[idx]

                if name not in self.robot_publishers:
                    topic_name = f"/{name}"
                    self.robot_publishers[name] = self.create_publisher(Pose, topic_name + "/pose", 10)
                    self.get_logger().info(f"Created publisher for {name} as topic {topic_name}")

                # publish Pose
                self.robot_publishers[name].publish(pose)

        # kombiniere alle Arrays in einen Vektor
        combined = np.concatenate([
            self.phi.astype(np.float32).flatten(),
            self.alpha.astype(np.float32).flatten(),
            self.t.astype(np.float32).flatten()
        ])

        msg_all = Float32MultiArray()
        msg_all.data = combined.tolist()

        # optional: Dimensionen dokumentieren
        msg_all.layout.dim.append(MultiArrayDimension(label="phi",   size=len(self.phi),   stride=len(self.phi)))
        msg_all.layout.dim.append(MultiArrayDimension(label="alpha", size=len(self.alpha), stride=len(self.alpha)))
        msg_all.layout.dim.append(MultiArrayDimension(label="t",     size=self.t.size,    stride=self.t.size))

        self.pub_all.publish(msg_all)

def main():
    main_fn('MSXPseudoRoofcamNode', MSXPseudoRoofcamNode)

if __name__ == '__main__':
    main()
