from rclpy.node import Node
from std_msgs.msg import String
import rclpy
import sys
import functools
from termcolor import colored
from driving_swarm_utils.node import DrivingSwarmNode


class CommandNode(DrivingSwarmNode):
    def __init__(self):
        super().__init__("command_node")
        self.logger_ = self.get_logger()
        # fetch swarm size and robot names
        self.declare_parameter("run_timeout", 0.0)
        self.declare_parameter("init_timeout", 0.0)
        self.get_list_of_robot_names()
        self.cmd_pub = self.create_publisher(String, "/command", 10)

        self.robot_status = {robot: None for robot in self.robots}
        for robot in self.robots:
            self.create_subscription(
                String, f"/{robot}/status",
                functools.partial(self.robot_cb, robot),
                10
            )
            self.logger_.info(f"subscribing /{robot}/status")
        self.run_timeout = self.get_parameter("run_timeout")\
            .get_parameter_value().double_value
        self.init_timeout = self.get_parameter("init_timeout")\
            .get_parameter_value().double_value
        self.set_status("ready")
        self.init_timer = None
        if self.init_timeout > 0:
            self.init_timer = self.create_timer(self.init_timeout, self.init_timeout_callback)
    
    def run_timeout_callback(self):
        self.logger_.info(f"run timer is due with status {self.status}")
        self.exit()
        
    def init_timeout_callback(self):
        self.logger_.info(f"init timer is due with status {self.status}")
        if self.status == "ready":
            self.exit()
        if self.init_timer is not None:
            self.init_timer.cancel()
            self.init_timer.destroy()
            self.init_timer = None
    
    def robot_cb(self, robot, msg):
        self.logger_.info(colored(f"{robot}: {msg.data}", "yellow"))
        self.robot_status[robot] = msg.data
        self.logger_.info(f"{self.robot_status}")
        if self.status == "ready":
            if all(
                [status == "ready" for status in self.robot_status.values()]
            ):
                self.set_status("go")
                if self.run_timeout > 0:
                    self.create_timer(self.run_timeout, callback=self.run_timeout_callback)
                if self.init_timer is not None:
                    self.init_timer.cancel()
                    self.init_timer.destroy()
                    self.init_timer = None
            
    def set_status(self, status):
        self.status = status
        self.cmd_pub.publish(String(data=status))
        self.logger_.info(colored(status, "green"))
    
    def exit(self):
        self.set_status("stop")
        self.logger_.info("exiting")
        sys.exit(0)
        

def main():
    rclpy.init()
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger_.info(f'got keyboard interrupt, shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
