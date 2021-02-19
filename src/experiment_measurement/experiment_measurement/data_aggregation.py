"""Aggregate the data to one table."""
import math

import pandas as pd

from rosbag2df import read_rosbag_all_in_one

from transformations import euler_from_quaternion


def get_latest_in_t(df, t):
    """Return the datapoint which is right before timestamp or at timestamp."""
    return df[df['timestamp'] <= t].iloc[-1]


def quaternion_to_euler(data):
    """Calculate the euler representation of a quaternion."""
    quaternion = [
        data.pose.pose.orientation.w,
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z
    ]
    return euler_from_quaternion(quaternion)


def calculate_travelled_distance(data, t):
    """Calculate the travelled distance of the robot basef on the position.

    Interpolate the points with linear funcitons.
    """
    data = data[data['timestamp'] <= t].copy()
    data['x'] = data['data'].map(lambda x: x.pose.pose.position.x)
    data['y'] = data['data'].map(lambda x: x.pose.pose.position.y)
    print(data)
    data = data.filter(['x', 'y'])
    data_shifted = data.shift(1)
    data = data.subtract(data_shifted)
    data = data.applymap(lambda x: x**2)
    return (data['x'] + data['y']).map(lambda x: math.sqrt(x)).sum()


def calculate_reached_goals(data, t):
    """Calculate the goals reached at the time."""
    data = data[data['timestamp'] <= t]
    data = data['data']
    data_shifted = data.shift(1)
    return (data != data_shifted).sum()


"""
df_aggregated_topics ::= table_column, [ df_aggregated_topics ]
table_column         ::= ( topic, table_column_name, func )

topic                ::= string
table_column_name    ::= string
func                 ::= message_dataframe, time -> Object

message_dataframe    ::= pd.Series      # All messages to one topic and robot
time                 ::= number
"""
df_aggregated_topics = [
    (
        'cmd_vel',
        'trans_vel',
        lambda msg_df, t: get_latest_in_t(msg_df, t)['data'].linear.x
    ),
    (
        'cmd_vel',
        'rot_vel',
        lambda msg_df, t: get_latest_in_t(msg_df, t)['data'].angular.z
    ),
    (
        'amcl_pose',
        'pose_x',
        lambda msg_df, t:
            get_latest_in_t(msg_df, t)['data'].pose.pose.position.x
    ),
    (
        'amcl_pose',
        'pose_y',
        lambda msg_df, t:
            get_latest_in_t(msg_df, t)['data'].pose.pose.position.y
    ),
    (
        'amcl_pose',
        'pose_theta',
        lambda msg_df, t:
            quaternion_to_euler(get_latest_in_t(msg_df, t)['data'])[2]
    ),
    (
        'amcl_pose',
        'traveled_distance',
        lambda msg_df, t: calculate_travelled_distance(msg_df, t)
    ),
    (
        'scan',
        'min_obstacle_dist',
        lambda msg_df, t: min(get_latest_in_t(msg_df, t)['data'].ranges)
    ),
    (
        'goal',
        'reached_goals',
        lambda msg_df, t: calculate_reached_goals(msg_df, t)
    ),
]


def aggregate_tables(df, step_size):
    """Create a table for each robot with corresponding values."""
    res = {}
    xmin = df['timestamp'][0] + step_size
    xmax = df['timestamp'].iloc[-1]

    robots = {itm[0] for itm in df['name'].str
              .findall(r'\/([^\/]*)\/') if len(itm) > 0}
    for robot in robots:
        robot_df = df[df['name'].str.match(r'\/{}\/.*'.format(robot))]
        robot_ret_df = pd.DataFrame(
            range(xmin, xmax, step_size),
            columns=['timestamp']
        )

        for topic in df_aggregated_topics:
            tmp_col = []
            robot_df_topic = robot_df[robot_df['name'].str
                                      .match(r'\/.*\/{}'.format(topic[0]))]

            for t in range(xmin, xmax, step_size):
                try:
                    topic_data = topic[2](robot_df_topic, t)
                except Exception as e:
                    print(e)
                    topic_data = None
                tmp_col.append(topic_data)
            robot_ret_df[topic[1]] = tmp_col
        res[robot] = robot_ret_df
    return res


def main():
    """Is the main entry point when called from command."""
    db_path = '/home/turtle-student/Downloads/\
Telegram Desktop/rosbag2_2021_02_09-15_11_58_0.db3'
    data_dict = read_rosbag_all_in_one(db_path)
    print(aggregate_tables(data_dict['rosbag'], 10**9)['robot1'])


if __name__ == '__main__':
    main()
