from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class MSXPseudoRoofcamNode(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.robot_publishers = {}

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
                    topic_name = f"/{name}/pose"
                    self.robot_publishers[name] = self.create_publisher(Pose, topic_name, 10)
                    self.get_logger().info(f"Created publisher for {name} as topic {topic_name}")

                self.robot_publishers[name].publish(pose)

def main():
    main_fn('MSXPseudoRoofcamNode', MSXPseudoRoofcamNode)
    rclpy.init(args=args)
    node = MSXPseudoRoofcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
