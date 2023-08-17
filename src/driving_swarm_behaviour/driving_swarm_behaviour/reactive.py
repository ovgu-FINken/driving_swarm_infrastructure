import rclpy
import numpy as np

from driving_swarm_utils.node import DrivingSwarmNode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveController(DrivingSwarmNode):

    def __init__(self):
        super().__init__('reactive_controller')
        self.declare_parameter('synchronise', True)
        self.synchronise = self.get_parameter('synchronise').get_parameter_value().bool_value
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
        self.sign = 1.0
        self.clear = False
        
    def timer_cb(self):
        if self.synchronise and not self.started:
            return
        msg = Twist()
        if self.forward_distance > 0.4:
            self.clear = True

        if self.left_distance < 0.12:
            msg.linear.x = 0.0
            msg.angular.z = -0.1
        elif self.right_distance < 0.12:
            msg.linear.x = 0.0
            msg.angular.z = 0.1
        else:
            if self.forward_distance > 0.3:
                msg.linear.x = 0.10
                msg.angular.z = 0.0
            else:
                if self.clear:
                    # with probability 0.1, reverse turning direction (only when the roboter was clearing the obstacles before)
                    if np.random.rand() < 0.1:
                        self.sign *= -1.0
                        self.clear = False
                msg.linear.x = 0.0
                msg.angular.z = self.sign * 1.0
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        self.forward_distance = min(r[:60] + r[-60:])
        self.left_distance = min(r[60:120])
        self.right_distance = min(r[-120:-60])


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
