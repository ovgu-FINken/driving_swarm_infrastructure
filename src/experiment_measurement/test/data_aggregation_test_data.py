import numpy as np
import pandas as pd

import geometry_msgs.msg

data = pd.DataFrame(
    np.array([
        [
            0,
            "/robot1/cmd_vel",
            geometry_msgs.msg.Twist,
            geometry_msgs.msg.Twist(
                linear =geometry_msgs.msg.Vector3(x=0.3, y=0.0, z=0.0),
                angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)
            )
        ],
        [
            1,
            "/robot1/amcl_pose",
            geometry_msgs.msg.PoseWithCovarianceStamped,
            geometry_msgs.msg.PoseWithCovarianceStamped(
                pose=geometry_msgs.msg.PoseWithCovariance(
                    pose=geometry_msgs.msg.Pose(
                        position=geometry_msgs.msg.Point(
                            x=0.0,
                            y=0.0,
                            z=0.0
                        )
                    )
                )
            )
        ],
        [
            7,
            "/robot1/cmd_vel",
            geometry_msgs.msg.Twist,
            geometry_msgs.msg.Twist(
                linear =geometry_msgs.msg.Vector3(x=0.3, y=0.0, z=0.0),
                angular=geometry_msgs.msg.Vector3(x=-1.0, y=0.0, z=1.0)
            )
        ],
        [
            12,
            "/robot1/amcl_pose",
            geometry_msgs.msg.PoseWithCovarianceStamped,
            geometry_msgs.msg.PoseWithCovarianceStamped(
                pose=geometry_msgs.msg.PoseWithCovariance(
                    pose=geometry_msgs.msg.Pose(
                        position=geometry_msgs.msg.Point(
                            x=1.2,
                            y=1.0,
                            z=0.0
                        )
                    )
                )
            )
        ],
        [
            17,
            "/robot1/amcl_pose",
            geometry_msgs.msg.PoseWithCovarianceStamped,
            geometry_msgs.msg.PoseWithCovarianceStamped(
                pose=geometry_msgs.msg.PoseWithCovariance(
                    pose=geometry_msgs.msg.Pose(
                        position=geometry_msgs.msg.Point(
                            x=1.8,
                            y=1.4,
                            z=0.0
                        )
                    )
                )
            )
        ],
        [
            21,
            "/robot1/cmd_vel",
            geometry_msgs.msg.Twist,
            geometry_msgs.msg.Twist(
                linear =geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0),
                angular=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=-1.0)
            )
        ],
        [
            22,
            "/robot1/cmd_vel",
            geometry_msgs.msg.Twist,
            geometry_msgs.msg.Twist(
                linear =geometry_msgs.msg.Vector3(x=0.6, y=0.0, z=0.0),
                angular=geometry_msgs.msg.Vector3(x=0.6, y=0.0, z=-2.0)
            )
        ],
        [
            24,
            "/robot1/amcl_pose",
            geometry_msgs.msg.PoseWithCovarianceStamped,
            geometry_msgs.msg.PoseWithCovarianceStamped(
                pose=geometry_msgs.msg.PoseWithCovariance(
                    pose=geometry_msgs.msg.Pose(
                        position=geometry_msgs.msg.Point(
                            x=1.0,
                            y=1.6,
                            z=0.0
                        )
                    )
                )
            )
        ],
    ]),
    columns=['timestamp', 'name', 'type', 'data']
)
