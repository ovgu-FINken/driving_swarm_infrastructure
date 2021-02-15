import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from trajectory_generator.vehicle_model_node import TrajectoryGenerator, Vehicle
from nav_msgs.msg import Path

class FixedTrajectorySupervisor(Node):
    def __init__(self):
        super().__init__('fixed_trajector_supervisor')
        self.get_logger().info('Starting')
        self.declare_parameter("robots")
        self.robots = self.get_parameter("robots").get_parameter_value().string_array_value
        self.declare_parameter('vehicle_model')
        self.declare_parameter('step_size')
        self.declare_parameter('turn_radius')
        self.reference_frame = 'world'
        self.vm = TrajectoryGenerator(
            model = Vehicle(self.get_parameter('vehicle_model').get_parameter_value().integer_value),
            step = self.get_parameter('step_size').get_parameter_value().double_value,
            r = self.get_parameter('turn_radius').get_parameter_value().double_value
        )
        
        for robot in self.robots.keys():
            client = self.create_client(Empty, "nav/start")
            self.robots[robot]["client"] = client
            self.robots[robot]["trajectory"] = self.vm.tuples_to_path(self.robots[robot]["waypoint"])
            client.wait_for_service()
            self.get_logger().info(f"connected to {robot}")
            path = Path()
            path.header.frame_id = self.reference_frame
            for pose in trajectory:
                pose3d = PoseStamped()
                pose3d.header.frame_id = self.reference_frame
                pose3d.header.stamp = self.get_clock().now().to_msg()
                pose3d.pose.position.x = pose.x
                pose3d.pose.position.y = pose.y
                pose3d.pose.position.z = 0.0
                pose3d.pose.orientation = yaw_to_orientation(pose.theta) 
                
                path.poses.append(pose3d)

        
        # create follow trajectory action goal
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=ti)
        #self.get_logger().info('sending path to action server')

        self.start_all()
    
    def send_paths(self, trajectory, ti=0):
        
        # convert trajectory to correct space
        path = Path()
        path.header.frame_id = self.reference_frame
        for pose in trajectory:
            pose3d = PoseStamped()
            pose3d.header.frame_id = self.reference_frame
            pose3d.header.stamp = self.get_clock().now().to_msg()
            pose3d.pose.position.x = pose.x
            pose3d.pose.position.y = pose.y
            pose3d.pose.position.z = 0.0
            pose3d.pose.orientation = yaw_to_orientation(pose.theta) 
            
            path.poses.append(pose3d)

        
        # create follow trajectory action goal
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=ti)
        #self.get_logger().info('sending path to action server')

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
