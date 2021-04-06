"""Aggregate the data to one table."""
import argparse
import datetime
import os
import time
import traceback

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
    xmin = df["timestamp"][0] + step_size
    xmax = df["timestamp"].iloc[-1] + step_size

    robots = {
        itm[0]
        for itm in df["name"].str.findall(r"\/(robot[^\/]*)\/")
        if len(itm) > 0
    }
    for robot in robots:
        robot_df = df[df["name"].str.match(r"\/{}\/.*".format(robot))]
        robot_ret_df = pd.DataFrame(
            range(xmin, xmax, step_size), columns=["timestamp"]
        )

        for topic in table_column_config:
            tmp_col = []

            if topic.topic_name.startswith("/"):
                robot_df_topic = df[df["name"].str.match(topic.topic_name)]
            else:
                robot_df_topic = robot_df[
                    robot_df["name"].str.match(
                        r"\/.*\/{}".format(topic.topic_name)
                    )
                ]

            for t in range(xmin, xmax, step_size):
                conf = data_aggregation_helper.TableConfig(
                    robot, robot_df_topic, t, t - step_size
                )
                try:
                    topic_data = topic(conf)
                except IndexError:
                    topic_data = None
                tmp_col.append(topic_data)
            robot_ret_df[topic.column_name] = tmp_col
        res[robot] = robot_ret_df
    return res


def main():
    """This is the main entry point when called from command."""
    my_parser = argparse.ArgumentParser(
        description="Aggregates the turtlebot data to one table"
    )

    my_parser.add_argument(
        "db_file", type=str, help="path to the db3 database file"
    )
    my_parser.add_argument(
        "config_file",
        type=str,
        help="configuration to use inside the config folder without 'py' ending, e.g. basti",
    )
    my_parser.add_argument(
        "--out",
        metavar="output_folder",
        type=str,
        help="path to the directory to store the exported csv files",
    )
    my_parser.add_argument(
        "--step_size",
        type=int,
        help="step size in nanoseconds, default: 1 second",
        default=10 ** 9,
    )

    args = my_parser.parse_args()

    input_db_path = args.db_file
    config = args.config_file
    output_folder_path = args.out
    try:
        step_size = int(args.step_size)
    except ValueError:
        raise ValueError("Step Size could not be converted to int!")

    assert os.path.isfile(
        input_db_path
    ), "The input db file path specified does not exist"

    if output_folder_path and not os.path.exists(output_folder_path):
        os.makedirs(output_folder_path)

    try:
        data_aggregation = __import__("config." + config, fromlist=[None])
    except ModuleNotFoundError:
        raise ModuleNotFoundError(
            "The configuration {} does not exist. Please specify a valid configuration file inside the config folder without the .py extension.".format(
                config
            )
        )

    data_dict = read_rosbag_all_in_one(input_db_path)
    data = aggregate_tables(
        data_dict["rosbag"], data_aggregation.table_column_config, step_size
    )
    export_path_prefix = datetime.datetime.fromtimestamp(
        int(time.time())
    ).isoformat()

    for robot, r_data in data.items():
        if not output_folder_path:
            print(robot + ":")
            print(r_data)
        else:
            csv_output_path = os.path.join(
                output_folder_path,
                export_path_prefix.replace(":", "-") + "_" + robot + ".csv",
            )
            if os.path.exists(csv_output_path):
                raise Exception("File {} exists!".format(csv_output_path))
            r_data.to_csv(csv_output_path)

    if output_folder_path:
        print(
            "Wrote {} files!".format(len(data))
        )  # TODO: add export_path_prefix to printout info


if __name__ == "__main__":
    main()
