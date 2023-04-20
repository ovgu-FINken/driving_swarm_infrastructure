from rclpy.node import Node
import rclpy
from termcolor import colored
import tf2_ros


class DrivingSwarmNode(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.get_logger().info("starting node "+colored(f"{name}", "green"))
        self.name = name


    def get_list_of_robot_names(self):
        self.declare_parameter('robot_names', ['invalid_name'])
        self.robots = self.get_parameter('robot_names').get_parameter_value().string_array_value
        if self.robots[0] == 'invalid_name':
            self.get_logger().warn(colored('no robot names specified!', 'red'))
            self.get_logger().warn('please specify a list of robot names via the parameter "robot_names"')
            
    def get_frames(self):
        self.get_own_frame()
        self.get_reference_frame()
    
    def get_own_frame(self):
        self.declare_parameter('own_frame', 'base_link')
        self.own_frame = self.get_parameter('own_frame').get_parameter_value().string_value
        if self.own_frame != 'base_link':
            self.get_logger().warn(f'own_frame is {self.own_frame}, this is not the default value "base_link"')
    
    def get_reference_frame(self):
        self.declare_parameter('reference_frame', 'map')
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        if self.reference_frame != 'map':
            self.get_logger().warn(f'reference_frame is {self.reference_frame}, this is not the default value "map"')
            
    def setup_tf(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
    def wait_for_tf(self):
        self.get_logger().info("waiting for tf " + colored(f"{self.name}", 'yellow'))
        f = self.tf_buffer.wait_for_transform_async(
            self.reference_frame, self.own_frame, rclpy.time.Time().to_msg()
        )
        rclpy.spin_until_future_complete(self, f)
        self.get_logger().info('tf available for'+colored(f"{self.name}", 'green'))


def main_fn(name, NodeClass):
    rclpy.init()
    node = NodeClass(name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting Down")
        node.destroy_node()
    rclpy.shutdown()
