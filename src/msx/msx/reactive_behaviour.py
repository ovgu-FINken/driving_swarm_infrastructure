import rclpy
import numpy as np
import time

from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

class ReactiveController(DrivingSwarmNode):
    def __init__(self, name):
        super().__init__(name)
        self.forward_distance = 0.0
        self.sign = 1.0
        self.clear = False
        
        self.sign_pub = self.create_publisher(Int32, "nav/sign", 1)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.setup_command_interface(autorun=True)
        
        # setup and wait for tf
        self.setup_tf()
        self.get_frames()
        self.wait_for_tf()
        
        # once tf is ready, the robot is ready to start
        self.set_state_ready()
        
        # time control variables
        self.active = True  # start by driving
        self.last_switch_time = time.time()
        self.period = 10.0  # 10 seconds active, 10 seconds inactive
        self.cur_speed = np.random.uniform(0.03, 0.1)

        # the timer_cb will publish the cmd_vel messages
        self.create_timer(0.1, self.timer_cb)
        
    def timer_cb(self):
        # publish cmd_vel messages based on free empty space in front of the robot

        # if the robot is not started, do nothing
        if not self.started:
            return

        current_time = time.time()
        elapsed = current_time - self.last_switch_time

        # change speed every 10 seconds
        if elapsed >= self.period:
            self.cur_speed = np.random.uniform(0.03, 0.1)
            self.cur_speed = 0.05
            self.last_switch_time = current_time
        
        msg = Twist()

        # TODO: Remove this line, which disables reactive behaviour
        # self.active = False

        # if the robot has empty space in front of it, move forward
        if self.active:
            # if self.forward_distance > 0.5:
            #     self.clear = True
            if self.forward_distance > 0.4:
                msg.linear.x = self.cur_speed
                msg.angular.z = 0.0
            else:
                # if the robot is too close to an obstacle, turn
                # if self.clear:
                #     # with probability 0.1, reverse turning direction (only when the roboter was clearing the obstacles before)
                #     if np.random.rand() < 0.1:
                #         self.sign *= -1.0
                #         self.clear = False
                msg.linear.x = 0.0
                msg.angular.z = self.sign * 0.5
            self.sign_pub.publish(Int32(data=int(self.sign)))
            self.publisher.publish(msg)

    def laser_cb(self, msg):
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        self.forward_distance = min(r[:45] + r[-45:])

def main():
    main_fn('reactive_controller', ReactiveController)

if __name__ == '__main__':
    main()
