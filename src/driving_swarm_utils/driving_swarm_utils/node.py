from rclpy.node import Node
import rclpy
from termcolor import colored
import tf2_ros
import tf_transformations
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


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
        self.get_logger().info('tf available for '+colored(f"{self.name}", 'green'))
        
    def get_tf_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.own_frame,
                rclpy.time.Time().to_msg(),
            ).transform
            tt = trans.translation
            q = trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w
            pose = (tt.x, tt.y, tf_transformations.euler_from_quaternion(q)[2])

        except Exception as e:
            self.get_logger().warn(f"Exception in tf transformations\n{e}")
            return None
        return pose
    
    def tuple_to_pose_stamped_msg(self, x, y, yaw, frame=None):
        pose = PoseStamped()
        pose.header.stamp = rclpy.time.Time().to_msg()
        if frame is None:
            pose.header.frame_id = self.reference_frame
        else:
            pose.header.frame_id = frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose
    
    def pose_stamped_to_tuple(self, pose: PoseStamped):
        return pose.pose.position.x, pose.pose.position.y, tf_transformations.euler_from_quaternion((
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ))[2]
    
    def setup_command_interface(self, autorun=True):
        self.autorun = autorun
        qos_profile = rclpy.qos.qos_profile_system_default
        qos_profile.reliability = (
            rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )
        qos_profile.durability = (
            rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )
        self.status_pub = self.create_publisher(String, "status", qos_profile)
        self.create_subscription(
            String, "/command", self.command_cb, qos_profile
        )
        
    def set_state(self, state: str):
        self.get_logger().info("setting state to " + colored(str(state), "blue"))
        self.status_pub.publish(String(data=str(state)))
        
    def set_state_ready(self):
        self.set_state("ready")
        
    def command_cb(self, msg):
        if msg.data == "go":
            self.get_logger().info("going")
            self.started = True
            if self.autorun:
                self.set_state("running")
            
        elif msg.data == "stop":
            self.set_state("stopped")
            raise KeyboardInterrupt()


def main_fn(name, NodeClass):
    rclpy.init()
    node = NodeClass(name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting Down")
        node.destroy_node()
    rclpy.shutdown() 

