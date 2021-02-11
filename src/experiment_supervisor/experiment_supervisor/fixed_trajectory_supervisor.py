import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

class FixedTrajectorySupervisor(Node):
    def __init__(self):
        super().__init__('fixed_trajector_supervisor')
        self.get_logger().info('Starting')
        self.declare_parameter("robots")
        robots = self.get_parameter("robots").get_parameter_value().string_array_value
        self.robots = { robot: {} for robot in robots}
        
        for robot in self.robots.keys():
            client = self.create_client(Empty, "nav/start")
            self.robots[robot]["client"] = client
            client.wait_for_service()
            self.get_logger().info(f"connected to {robot}")

        self.start_all()

    def start_all(self):
        for robot in self.robots.keys():
            request = Empty.Request()
            self.robots[robot]["client"].call_async(request)

def main():
    rclpy.init()
    node = FixedTrajectorySupervisor()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
