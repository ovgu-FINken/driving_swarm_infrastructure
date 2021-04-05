import unittest

import numpy as np
import math
import pandas as pd
from pandas.testing import assert_series_equal

import geometry_msgs.msg

import data_aggregation_test_data
from experiment_measurement.data_aggregation import aggregate_tables
from experiment_measurement.config import basti


class Test(unittest.TestCase):
    def setUp(self):
        self.step_size = 10
        self.input_df = data_aggregation_test_data.data
        self.res = aggregate_tables(
            self.input_df,
            basti.table_column_config,
            self.step_size
        )

    def test_timestamp(self):
        assert_series_equal(
            self.res['robot1']['timestamp'],
            pd.Series([10, 20, 30], name='timestamp')
        )


    def test_trans_vel(self):
        assert_series_equal(
            self.res['robot1']['trans_vel'],
            pd.Series([0.3, None, 0.6], name='trans_vel')
        )


    def test_rot_vel(self):
        assert_series_equal(
            self.res['robot1']['rot_vel'],
            pd.Series([1.0, None, -2.0], name='rot_vel')
        )


    def test_pose_x(self):
        assert_series_equal(
            self.res['robot1']['amcl_pose_x'],
            pd.Series([0.0, 1.8, 1.0], name='amcl_pose_x')
        )


    def test_pose_y(self):
        assert_series_equal(
            self.res['robot1']['amcl_pose_y'],
            pd.Series([0.0, 1.4, 1.6], name='amcl_pose_y')
        )


    def test_traveled_distance(self):
        assert_series_equal(
            self.res['robot1']['amcl_traveled_distance'],
            pd.Series([
                0.0,
                math.sqrt(1.2**2 + 1.0**2) + math.sqrt(0.6**2 + 0.4**2),
                math.sqrt(1.2**2 + 1.0**2) + math.sqrt(0.6**2 + 0.4**2)
                + math.sqrt(0.8**2 + 0.2**2),
            ], name='amcl_traveled_distance')
        )


        
if __name__ == '__main__':
    unittest.main()

