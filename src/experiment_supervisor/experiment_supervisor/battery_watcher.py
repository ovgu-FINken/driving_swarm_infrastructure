
import rclpy
import functools
from termcolor import colored
from driving_swarm_utils.node import DrivingSwarmNode
from sensor_msgs.msg import BatteryState


class BatteryWatcher(DrivingSwarmNode):
    def __init__(self):
        super().__init__("batter_watcher")
        self.logger_ = self.get_logger()
        self.battry_percentages = {}

        self.get_list_of_robot_names()
        for robot in self.robots:
            self.create_subscription(
                BatteryState, f"/{robot}/battery_state",
                functools.partial(self.robot_cb, robot),
                10
            )
            self.logger_.info(f"subscribing /{robot}/battery_state")
        self.create_timer(10.0, self.timer_cb)

    def timer_cb(self):
        self.get_logger().info("battery status:")
        for robot in self.robots:
            if robot in self.battry_percentages:
                msg = f"\t{robot}: battery at {self.battry_percentages[robot]:.1f}%"
                if self.battry_percentages[robot] < 30:
                    self.get_logger().info(colored(msg, "red"))
                elif self.battry_percentages[robot] < 50:
                    self.get_logger().info(colored(msg, "yellow"))
                else:
                    self.get_logger().info(msg)
                
        
    def robot_cb(self, robot, msg):
        self.battry_percentages[robot] = msg.percentage

def main():
    rclpy.init()
    node = BatteryWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger_.info(f'got keyboard interrupt, shutting down')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
