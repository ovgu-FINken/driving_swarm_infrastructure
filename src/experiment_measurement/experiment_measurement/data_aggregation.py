"""Aggregate the data to one table."""
import argparse
import os

import math

import pandas as pd

from transformations import euler_from_quaternion

# from rosbag2df import read_rosbag_all_in_one
# import data_aggregation_helper

from experiment_measurement.rosbag2df import read_rosbag_all_in_one
from experiment_measurement import data_aggregation_helper


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

            if topic.topic_name.startswith('/'):
                robot_df_topic = robot_df[
                    df['name'].str.match(topic.topic_name)
                ]
            else:
                robot_df_topic = robot_df[
                    robot_df['name'].str.match(r'\/.*\/{}'.format(topic.topic_name))
                ]

            for t in range(xmin, xmax, step_size):
                conf = data_aggregation_helper.TableConfig(
                    robot,
                    robot_df_topic,
                    t,
                    t - step_size
                )
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
    """This is the main entry point when called from command."""
    my_parser = argparse.ArgumentParser(
        description = 'Aggregates the turtlebot data to one table'
    )

    my_parser.add_argument('input_config',
                           type=str,
                           help='configuration to use, e.g. data_aggregation_basti')
    my_parser.add_argument('input_file',
                           type=str,
                           help='path to the db3 database file')
    my_parser.add_argument('--of',
                           metavar='output_folder',
                           type=str,
                           help='path to the directory to store the exported csv files')
    my_parser.add_argument('--force',
                           action='store_true',
                           help='don\'t ask for approval if a file would be overwritten')
    my_parser.add_argument('--step_size',
                           type=int,
                           help='step size in nanoseconds, default in 1 second',
                           default=10**9)


    args = my_parser.parse_args()

    input_config_path    = args.input_config
    input_file_path      = args.input_file
    output_folder_path   = args.of
    force                = args.force
    try:
        step_size        = int(args.step_size)
    except ValueError:
        raise ValueError("Step Size could not be converted to int!")


    assert os.path.isfile(input_file_path),\
        'The input file path specified does not exist'

    if output_folder_path and not os.path.exists(output_folder_path):
        os.makedirs(output_folder_path)

    try:
        data_aggregation = __import__(input_config_path)
    except ModuleNotFoundError:
        raise ModuleNotFoundError("The configuraiton {} does not exist. Please specify a valid configuration file relative to the data_aggregation.py script without the .py extension.".format(input_config_path))


    data_dict = read_rosbag_all_in_one(input_file_path)
    data = aggregate_tables(
        data_dict['rosbag'],
        data_aggregation.table_column_config,
        step_size
    )
    for robot, r_data in data.items():
        if not output_folder_path:
            print(robot + ":")
            print(r_data)
        else:
            csv_output_path = os.path.join(output_folder_path, robot + ".csv")
            if os.path.exists(csv_output_path) and not force:
                raise Exception("File {} exists!".format(csv_output_path))
            r_data.to_csv(csv_output_path)

if __name__ == '__main__':
    main()
