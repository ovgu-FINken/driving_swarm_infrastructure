import rclpy
import tf2_ros
import tf2_py
import yaml
import numpy as np

from rclpy.node import Node
from driving_swarm_messages.msg import Range


class FakeRange(Node):

    def __init__(self):
        super().__init__('range_node')
        self.tfBuffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=3.0))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.reference_frame = 'world'
        self.own_frame = 'base_link'
        self.declare_parameter('anchor_list',
                                '''
- {x: 0.0, y: 0.0, z: 1.0}
- {x: 1.0, y: 0.0, z: 1.0}
- {x: 0.0, z: 1.0, y: 1.0}
- {x: 1.0, y: 1.0, z: 1.0}
- {x: 0.0, y: 0.0, z: 2.0}''')
        anchors = self.get_parameter('anchor_list').get_parameter_value().string_value
        self.anchors = yaml.safe_load(anchors)
        self.get_logger().info(f'achors: {self.anchors}')
        self.declare_parameter('rate', 0.2)
        rate = self.get_parameter('rate').get_parameter_value().double_value
        self.t = 0
        
        f = self.tfBuffer.wait_for_transform_async(
            self.own_frame, self.reference_frame, rclpy.time.Time().to_msg()
        )
        self.get_logger().info(f"waiting for transform map -> {self.reference_frame}")
        rclpy.spin_until_future_complete(self, f)

        self.range_pub = self.create_publisher(Range, 'range', 10)
        self.create_timer(rate, self.timer_cb)
        self.get_logger().info('ranging node started')
        
    def timer_cb(self):
        # get robot position
        try:
            trans = self.tfBuffer.lookup_transform(
                self.reference_frame,
                self.own_frame,
                rclpy.time.Time().to_msg(),
            )

        except Exception as e:
            self.get_logger().warn(f"Exception in tf transformations\n{e}")
            return
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        
        # compute range
        anchor = self.anchors[self.t % len(self.anchors)]
        ax, ay, az = anchor['x'], anchor['y'], anchor['z']
        dist = np.linalg.norm(np.array([x - ax, y - ay, z - az]))

        # publish range message
        self.get_logger().debug(f'robot at position ({x:.1f}, {y:.1f}, {z:.1f}). Dist = {dist}')
        msg = Range()
        msg.anchor.x, msg.anchor.y, msg.anchor.z = ax, ay, az
        msg.range = dist
        self.range_pub.publish(msg)
        self.t += 1
        


def main(args=None):
    rclpy.init(args=args)

    node = FakeRange()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

