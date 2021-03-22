from rclpy.node import Node
from std_msgs.msg import String
import rclpy
import functools
from termcolor import colored

class CommandNode(Node):
    def __init__(self):
        super().__init__("command_node")
        # fetch swarm size and robot names
        self.declare_parameter("robots")
        self.robots = self.get_parameter("robots").get_parameter_value().string_array_value
        
        self.cmd_pub = self.create_publisher(String, "/command", 1)

        self.robot_status = {robot: None for robot in self.robots}
        for robot in self.robots:
            self.create_subscription(String, f"/{robot}/status", functools.partial(self.robot_cb, robot), 1)
        self.set_status("ready")
        
    
    def robot_cb(self, msg, robot):
        self.get_logger().info(colored(f"{robot}: {msg.data}", "yellow"))
        self.robot_status[robot] = msg.data
        if self.status == "ready":
            if all([status == "ready" for status in self.robot_status.values]):
                self.set_status("go")
            
        
    def set_status(self, status):
        self.status = status
        self.cmd_pub.publish(String(data=status))
        self.get_logger().info(colored(status, "red"))
        

def main():
    rclpy.init()
    node = CommandNode()
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()