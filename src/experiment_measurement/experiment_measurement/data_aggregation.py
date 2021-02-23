"""Aggregate the data to one table."""
import math

import pandas as pd

from transformations import euler_from_quaternion

from rosbag2df import read_rosbag_all_in_one
import data_aggregation_basti
import data_aggregation_helper

def aggregate_tables(df, table_column_config, step_size):
    """Create a table for each robot with corresponding values."""
    res = {}
    xmin = df['timestamp'][0] + step_size
    xmax = df['timestamp'].iloc[-1] + step_size

    robots = {
        itm[0]
        for itm in df['name'].str.findall(r'\/([^\/]*)\/') if len(itm) > 0
    }
    for robot in robots:
        robot_df = df[df['name'].str.match(r'\/{}\/.*'.format(robot))]
        robot_ret_df = pd.DataFrame(
            range(xmin, xmax, step_size), columns=['timestamp']
        )

        for topic in table_column_config:
            tmp_col = []
            
            robot_df_topic = robot_df[
                robot_df['name'].str.match(r'\/.*\/{}'.format(topic.topic_name))
            ]

            for t in range(xmin, xmax, step_size):
                conf = data_aggregation_helper.TableConfig(robot_df_topic, t, t - step_size)
                try:
                    topic_data = topic(conf)
                except Exception as e:
                    # print(e)
                    topic_data = None
                tmp_col.append(topic_data)
            robot_ret_df[topic.column_name] = tmp_col
        res[robot] = robot_ret_df
    return res


def main():
    """Is the main entry point when called from command."""
    db_path = '/home/turtle-student/Downloads/\
Telegram Desktop/rosbag2_2021_02_09-15_11_58_0.db3'
    data_dict = read_rosbag_all_in_one(db_path)
    print(aggregate_tables(
        data_dict['rosbag'],
        data_aggregation_basti.table_column_config,
        10**9
    )['robot1'])


if __name__ == '__main__':
    main()
