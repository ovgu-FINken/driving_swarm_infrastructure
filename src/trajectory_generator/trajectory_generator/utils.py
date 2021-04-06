from geometry_msgs.msg import Quaternion
import PyKDL


def yaw_from_orientation(orientation):
    rot = PyKDL.Rotation.Quaternion(
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    return rot.GetRPY()[2]


def yaw_to_orientation(yaw):
    q = PyKDL.Rotation.RPY(0, 0, yaw).GetQuaternion()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
