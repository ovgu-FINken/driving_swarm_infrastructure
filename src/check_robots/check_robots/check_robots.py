import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist


class RobotCheck(Node):
    def __init__(self):
        super().__init__('self_check')
        self.create_subscription(TFMessage, 'tf', self.tf_cb, 100)
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_ok = None
        self.once = False
        self.t = 0

    def tf_cb(self, msg):
        for frame in msg.transforms:
            if frame.header.frame_id == 'odom' and not self.once:
                self.once = True
                self.tf_ok = False
                if abs(frame.transform.translation.x) < 1000 and abs(frame.transform.translation.y) < 1000:
                    self.tf_ok = True
                    self.create_timer(0.1, self.timer_cb)
                else:
                    self.get_logger().warn(f'Transform problem: {frame}')
    
    def timer_cb(self):
        self.once = True
        msg = Twist()

        if self.t < 10:
            msg.linear.x = 0.03
        elif self.t < 20:
            msg.linear.x = -0.03
        self.t += 1
        self.cmd_vel.publish(msg)
        

    def result(self):
        if self.tf_ok:
            self.get_logger().info('TF is okay')
        else:
            self.get_logger().warn('TF IS BROKEN')
        
def main():
    rclpy.init()
    node = RobotCheck()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
