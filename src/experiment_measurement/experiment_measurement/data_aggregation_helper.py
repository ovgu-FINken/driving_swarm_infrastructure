"""Aggregate the data to one table."""
import copy
import math

import numpy as np

from transformations import euler_from_quaternion
# from att_rep_controller.utils import __cart2pol


class TableConfig:

    def __init__(self, robot_name, df, t, t_prev):
        self.robot_name = robot_name
        self.df = df
        self.t = t
        self.t_prev = t_prev

    def __repr__(self):
        return 'TableConfig({}\n {}\n {}\n {})'.format(
            self.robot_name, self.df, self.t, self.t_prev
        )

class TableColumn:

    def __init__(self, topic_name, column_name, fn):
        self.topic_name = topic_name
        self.column_name = column_name
        self.fn = fn  # lamda function

    def __call__(self, table_config):
        return self.fn(table_config)


def get_latest_in_interval(conf):
    """Return the datapoint which is right before timestamp or at timestamp."""
    res = conf.df.loc[
        conf.df.timestamp.le(conf.t)
        & conf.df.timestamp.gt(conf.t_prev)
    ]
    return res.iloc[-1]


def filter_tf_child_frame_id(conf):
    """Filter the tf data for a specific topic (robot)."""
    conf_copy = copy.copy(conf)
    mask = conf.df['data'].map(lambda x: x.transforms[0].child_frame_id == conf.robot_name)

    conf_copy.df = conf.df[
        mask
    ]
    return conf_copy


def quaternion_to_euler(data):
    """Calculate the euler representation of a quaternion."""
    quaternion = [
        data.w,
        data.x,
        data.y,
        data.z,
    ]
    return euler_from_quaternion(quaternion)


def amcl_calc_x_and_y(conf):
    """Extract x and y position of amcl data."""
    conf_copy = copy.copy(conf)
    conf_copy.df = conf.df.copy()
    conf_copy.df['x'] = conf_copy.df['data'].map(lambda x: x.pose.pose.position.x)
    conf_copy.df['y'] = conf_copy.df['data'].map(lambda x: x.pose.pose.position.y)
    return conf_copy


def tf_calc_x_and_y(conf):
    """Extract x and y position of tf data."""
    conf_copy = copy.copy(conf)
    conf_copy.df = conf.df.copy()
    conf_copy.df['x'] = conf_copy.df['data'].map(lambda x: x.transforms[0].transform.translation.x)
    conf_copy.df['y'] = conf_copy.df['data'].map(lambda x: x.transforms[0].transform.translation.y)
    return conf_copy


def calculate_travelled_distance(conf):
    """
    Calculate the travelled distance of the robot basef on the position.

    Interpolate the points with linear funcitons.
    """
    df = conf.df[conf.df['timestamp'] <= conf.t]

    df = df.filter(['x', 'y'])
    df_shifted = df.shift(1)
    df = df.subtract(df_shifted)
    df = df.applymap(lambda x: x ** 2)
    return (df['x'] + df['y']).map(lambda x: math.sqrt(x)).sum()


def calculate_reached_goals(conf):
    """Calculate the goals reached at the time."""
    df = conf.df[conf.df['timestamp'] <= conf.t]
    df = df['data']
    df_shifted = df.shift(1)
    return (df != df_shifted).sum()


def filter_force_id(conf, ident):
    """Filter the forces (rviz arrow marker) by the id to get the specified force."""
    conf_copy = copy.copy(conf)
    mask = conf.df['data'].map(lambda x: x.id == ident)

    conf_copy.df = conf.df[
        mask
    ]
    # if conf.robot_name == "robot4" or conf.robot_name == "/robot4":
    #     print(conf_copy)
    return conf_copy


def get_vector_length(point):
    """
    Calculate the length of the arrow/vector, negate if it is a repulsion, i.e. the vector shows
    in backward direction (ego-perpective).
    """
    r = np.sqrt(point.x ** 2 + point.y ** 2)
    if point.x < 0:
        return -1 * r
    return r
