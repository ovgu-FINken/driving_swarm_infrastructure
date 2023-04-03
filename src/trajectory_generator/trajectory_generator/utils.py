from geometry_msgs.msg import Quaternion
import tf_transformations

def yaw_from_orientation(orientation):
    """Create a yaw angle from a quaternion orientation"""
    q = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf_transformations.euler_from_quaternion(q)[2]


def yaw_to_orientation(yaw):
    """Create a quaternion orientation from a yaw angle"""
    q = tf_transformations.quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])