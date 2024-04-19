import rclpy
import numpy as np

from driving_swarm_utils.node import DrivingSwarmNode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from driving_swarm_utils.utils import detect_tb_from_ranges

class ReactiveController(DrivingSwarmNode):
    def __init__(self):
        super().__init__('reactive_controller')
        self.declare_parameter('synchronise', True)
        self.synchronise = self.get_parameter('synchronise').get_parameter_value().bool_value
        self.sign_pub = self.create_publisher(Int32, "nav/sign", 1)
        self.get_frames()
        self.setup_tf()
        self.setup_command_interface(autorun=True)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.left_distance = 0
        self.right_distance = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.wait_for_tf()
        self.set_state_ready()
        self.create_timer(0.1, self.timer_cb)
        self.sign = np.random.choice([-1, 1])
        
    def timer_cb(self):
        if self.synchronise and not self.started:
            return
        msg = Twist()
        if self.forward_distance > 0.3:
            msg.linear.x = 0.10
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = self.sign * 1.0
        self.sign_pub.publish(Int32(data=int(self.sign)))
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        self.forward_distance = min(r[:45] + r[-45:])
        tbs = detect_tb_from_ranges(r, 0.0, 0.0, 0.0, msg.angle_min, msg.angle_increment)
        # check whether another tb is in front, and whether to change the sign
        if tbs:
            for x,y,*_ in tbs:
                if x < 0:
                    continue
                # bot is on the side, not in front
                if np.abs(y) > x:
                    continue
                if x < 0.4 and np.random.rand() < self.p_switch:
                    self.sign *= -1
                self.forward_distance = np.linalg.norm(np.array([x,y])) - 0.15


def main(args=None):
    rclpy.init(args=args)

    node = ReactiveController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
