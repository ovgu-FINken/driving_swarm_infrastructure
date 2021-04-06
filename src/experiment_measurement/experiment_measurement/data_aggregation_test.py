import unittest

import numpy as np
import math
import pandas as pd
from pandas.testing import assert_series_equal

import geometry_msgs.msg

from experiment_measurement.data_aggregation import aggregate_tables
from experiment_measurement import data_aggregation_basti


class Test(unittest.TestCase):
    def setUp(self):
        self.step_size = 10
        self.input_df = pd.DataFrame(
            np.array(
                [
                    [
                        0,
                        "/robot1/cmd_vel",
                        geometry_msgs.msg.Twist,
                        geometry_msgs.msg.Twist(
                            linear=geometry_msgs.msg.Vector3(
                                x=0.3, y=0.0, z=0.0
                            ),
                            angular=geometry_msgs.msg.Vector3(
                                x=0.0, y=0.0, z=0.0
                            ),
                        ),
                    ],
                    [
                        1,
                        "/robot1/amcl_pose",
                        geometry_msgs.msg.PoseWithCovarianceStamped,
                        geometry_msgs.msg.PoseWithCovarianceStamped(
                            pose=geometry_msgs.msg.PoseWithCovariance(
                                pose=geometry_msgs.msg.Pose(
                                    position=geometry_msgs.msg.Point(
                                        x=0.0, y=0.0, z=0.0
                                    )
                                )
                            )
                        ),
                    ],
                    [
                        7,
                        "/robot1/cmd_vel",
                        geometry_msgs.msg.Twist,
                        geometry_msgs.msg.Twist(
                            linear=geometry_msgs.msg.Vector3(
                                x=0.3, y=0.0, z=0.0
                            ),
                            angular=geometry_msgs.msg.Vector3(
                                x=-1.0, y=0.0, z=1.0
                            ),
                        ),
                    ],
                    [
                        12,
                        "/robot1/amcl_pose",
                        geometry_msgs.msg.PoseWithCovarianceStamped,
                        geometry_msgs.msg.PoseWithCovarianceStamped(
                            pose=geometry_msgs.msg.PoseWithCovariance(
                                pose=geometry_msgs.msg.Pose(
                                    position=geometry_msgs.msg.Point(
                                        x=1.2, y=1.0, z=0.0
                                    )
                                )
                            )
                        ),
                    ],
                    [
                        17,
                        "/robot1/amcl_pose",
                        geometry_msgs.msg.PoseWithCovarianceStamped,
                        geometry_msgs.msg.PoseWithCovarianceStamped(
                            pose=geometry_msgs.msg.PoseWithCovariance(
                                pose=geometry_msgs.msg.Pose(
                                    position=geometry_msgs.msg.Point(
                                        x=1.8, y=1.4, z=0.0
                                    )
                                )
                            )
                        ),
                    ],
                    [
                        21,
                        "/robot1/cmd_vel",
                        geometry_msgs.msg.Twist,
                        geometry_msgs.msg.Twist(
                            linear=geometry_msgs.msg.Vector3(
                                x=0.0, y=0.0, z=0.0
                            ),
                            angular=geometry_msgs.msg.Vector3(
                                x=1.0, y=0.0, z=-1.0
                            ),
                        ),
                    ],
                    [
                        22,
                        "/robot1/cmd_vel",
                        geometry_msgs.msg.Twist,
                        geometry_msgs.msg.Twist(
                            linear=geometry_msgs.msg.Vector3(
                                x=0.6, y=0.0, z=0.0
                            ),
                            angular=geometry_msgs.msg.Vector3(
                                x=0.6, y=0.0, z=-2.0
                            ),
                        ),
                    ],
                    [
                        24,
                        "/robot1/amcl_pose",
                        geometry_msgs.msg.PoseWithCovarianceStamped,
                        geometry_msgs.msg.PoseWithCovarianceStamped(
                            pose=geometry_msgs.msg.PoseWithCovariance(
                                pose=geometry_msgs.msg.Pose(
                                    position=geometry_msgs.msg.Point(
                                        x=1.0, y=1.6, z=0.0
                                    )
                                )
                            )
                        ),
                    ],
                ]
            ),
            columns=["timestamp", "name", "type", "data"],
        )
        self.res = aggregate_tables(
            self.input_df,
            data_aggregation_basti.table_column_config,
            self.step_size,
        )

    def test_timestamp(self):
        assert_series_equal(
            self.res["robot1"]["timestamp"],
            pd.Series([10, 20, 30], name="timestamp"),
        )

    def test_trans_vel(self):
        assert_series_equal(
            self.res["robot1"]["trans_vel"],
            pd.Series([0.3, None, 0.6], name="trans_vel"),
        )

    def test_rot_vel(self):
        assert_series_equal(
            self.res["robot1"]["rot_vel"],
            pd.Series([1.0, None, -2.0], name="rot_vel"),
        )

    def test_pose_x(self):
        assert_series_equal(
            self.res["robot1"]["pose_x"],
            pd.Series([0.0, 1.8, 1.0], name="pose_x"),
        )

    def test_pose_y(self):
        assert_series_equal(
            self.res["robot1"]["pose_y"],
            pd.Series([0.0, 1.4, 1.6], name="pose_y"),
        )

    def test_traveled_distance(self):
        assert_series_equal(
            self.res["robot1"]["traveled_distance"],
            pd.Series(
                [
                    0.0,
                    math.sqrt(1.2 ** 2 + 1.0 ** 2)
                    + math.sqrt(0.6 ** 2 + 0.4 ** 2),
                    math.sqrt(1.2 ** 2 + 1.0 ** 2)
                    + math.sqrt(0.6 ** 2 + 0.4 ** 2)
                    + math.sqrt(0.8 ** 2 + 0.2 ** 2),
                ],
                name="traveled_distance",
            ),
        )


if __name__ == "__main__":
    unittest.main()
