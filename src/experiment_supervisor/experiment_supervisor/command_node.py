from rclpy.node import Node
from std_msgs.msg import String
import rclpy
import sys
import functools
from termcolor import colored


class CommandNode(Node):
    def __init__(self):
        super().__init__("command_node")
        # fetch swarm size and robot names
        self.declare_parameter("robots")
        self.declare_parameter("run_timeout")
        self.robots = self.get_parameter("robots")\
            .get_parameter_value().string_array_value
        qos_profile = rclpy.qos.qos_profile_system_default
        qos_profile.reliability = rclpy.qos.QoSReliabilityPolicy\
            .RMW_QOS_POLICY_RELIABILITY_RELIABLE
        qos_profile.durability = rclpy.qos.QoSDurabilityPolicy\
            .RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        self.cmd_pub = self.create_publisher(String, "/command", qos_profile)

        self.robot_status = {robot: None for robot in self.robots}
        for robot in self.robots:
            self.create_subscription(
                String, f"/{robot}/status",
                functools.partial(self.robot_cb, robot),
                qos_profile
            )
            self.get_logger().info(f"subscribing /{robot}/status")
        self.run_timeout = self.get_parameter("run_timeout")\
            .get_parameter_value().double_value
        self.get_logger().info(f"run_timeout: {self.run_timeout}")
        self.set_status("ready")
    
    def robot_cb(self, robot, msg):
        self.get_logger().info(colored(f"{robot}: {msg.data}", "yellow"))
        self.robot_status[robot] = msg.data
        self.get_logger().info(f"{self.robot_status}")
        if self.status == "ready":
            if all(
                [status == "ready" for status in self.robot_status.values()]
            ):
                self.set_status("go")
                if self.run_timeout > 0:
                    self.create_timer(self.run_timeout, callback=self.exit)
            
    def set_status(self, status):
        self.status = status
        self.cmd_pub.publish(String(data=status))
        self.get_logger().info(colored(status, "red"))
    
    def exit(self):
        self.set_status("stop")
        self.get_logger().info("exiting")
        sys.exit(0)
        

def main():
    rclpy.init()
    node = CommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
