"""Aggregate the data to one table."""
import math

import pandas as pd

from transformations import euler_from_quaternion

from rosbag2df import read_rosbag_all_in_one


class TableConfig:
    def __init__(self, df, t, t_prev):
        self.df = df
        self.t = t
        self.t_prev = t_prev


class TableColumn:
    def __init__(self, topic_name, column_name, fn):
        self.topic_name = topic_name
        self.column_name = column_name
        self.fn = fn

    def __call__(self, table_config):
        return self.fn(table_config)


def get_latest_in_interval(conf):
    """Return the datapoint which is right before timestamp or at timestamp."""
    return conf.df[
        (conf.df['timestamp'] <= conf.t)
        & (conf.df['timestamp'] > conf.t_prev)
    ].iloc[-1]


def quaternion_to_euler(data):
    """Calculate the euler representation of a quaternion."""
    quaternion = [
        data.pose.pose.orientation.w,
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
    ]
    return euler_from_quaternion(quaternion)


def calculate_travelled_distance(conf):
    """Calculate the travelled distance of the robot basef on the position.

    Interpolate the points with linear funcitons.
    """
    df = conf.df[conf.df['timestamp'] <= conf.t].copy()
    df['x'] = df['data'].map(lambda x: x.pose.pose.position.x)
    df['y'] = df['data'].map(lambda x: x.pose.pose.position.y)

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
