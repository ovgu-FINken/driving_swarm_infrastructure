"""Aggregate the data to one table."""
import argparse
import datetime
from multiprocessing.pool import ThreadPool
import os
import time

import numpy as np
import pandas as pd

# from rosbag2df import read_rosbag_all_in_one
# import data_aggregation_helper
# from builtin_interfaces.msg import Time
from rclpy.time import Time

from experiment_measurement.rosbag2df import read_rosbag_all_in_one
from experiment_measurement import data_aggregation_helper


def aggregate_tables(df, table_column_config, step_size):
    """Create a table for each robot with corresponding values for each timestep defined by user."""
    res = {}
    xmin = df['timestamp'][0] + step_size
    xmax = df['timestamp'].iloc[-1] + step_size
    # TODO: We currently find the namespace with this regex. This is not very robust.
    # Instead we can simply read the robot_names_file and use the names from there.
    robots = {
        itm[0][0] for itm in df['name'].str.findall(r'\/((turtle|ro)bot[^\/]*)\/') if len(itm) > 0
    }
    print(f"found topics for namespaces: {robots}")
    for robot in robots:
        robot_df = df[df['name'].str.match(r'\/{}\/.*'.format(robot))]
        robot_ret_df = pd.DataFrame(
            range(xmin, xmax, step_size), columns=['timestamp']
        )
        for topic in table_column_config:
            tmp_col = []
            if topic.topic_name.startswith('/'):
                robot_df_topic = df[
                    df['name'].str.match(topic.topic_name)
                ]
                #print(robot_df_topic)
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
                except IndexError:
                    topic_data = None
                except (AttributeError, KeyError):  # No data
                    topic_data = None
                tmp_col.append(topic_data)
            robot_ret_df[topic.column_name] = tmp_col
        res[robot] = robot_ret_df
    return res


def aggregate_tables_by_msg_header(df, table_column_config, msg_topic):
    """Create a table for each robot with corresponding values for each timestamp stored in the messages' header (sent to msg_topic)."""
    res = {}

    robots = {
        itm[0][1] for itm in df['name'].str.findall(r'\/((turtle|ro)bot[^\/]*)\/') if len(itm) > 0
    }

    for robot in robots:
        robot_df = df[df['name'].str.match(r'\/{}\/.*'.format(robot))]
        
        t_min = df['timestamp'][0]
        timestamps_abs = [t_min]
        if not msg_topic.startswith('/'):
            msg_topic = '/' + robot + '/' + msg_topic 
        msgs = robot_df.loc[robot_df['name'] == msg_topic, 'data']
        for msg in msgs:
            timestamps_abs.append(
                Time.from_msg(msg.header.stamp).nanoseconds
            )
        timestamps_rel = np.array(timestamps_abs)
        timestamps_rel -= t_min
        robot_ret_df = pd.DataFrame(
            timestamps_rel[1:], columns=['timestamp']
        )

        for topic in table_column_config:
            tmp_col = []

            if topic.topic_name.startswith('/'):
                robot_df_topic = df[
                    df['name'].str.match(topic.topic_name)
                ]
            else:
                robot_df_topic = robot_df[
                    robot_df['name'].str.match(r'\/.*\/{}'.format(topic.topic_name))
                ]

            for i, t in enumerate(timestamps_abs[1:], start=1):
                conf = data_aggregation_helper.TableConfig(
                    robot,
                    robot_df_topic,
                    t,
                    timestamps_abs[i-1],
                )
                try:
                    topic_data = topic(conf)
                except IndexError:
                    topic_data = None
                except (AttributeError, KeyError):  # No data
                    topic_data = None
                tmp_col.append(topic_data)
            robot_ret_df[topic.column_name] = tmp_col
        res[robot] = robot_ret_df
    return res


def aggregate_tables_by_msg_timestamps(df, table_column_config, msg_topic):
    """Create a table for each robot with corresponding values for each timestamp the message was sent to msg_topic."""
    res = {}

    robots = {
        itm[0][1] for itm in df['name'].str.findall(r'\/((turtle|ro)bot[^\/]*)\/') if len(itm) > 0
    }
    for robot in robots:
        robot_df = df[df['name'].str.match(r'\/{}\/.*'.format(robot))]
        t_min = df['timestamp'][0]
        timestamps_abs = [t_min]
        t_min = df['timestamp'][0]
        if not msg_topic.startswith('/'):
            msg_topic = '/' + robot + '/' + msg_topic 
        msg_timestamps = robot_df.loc[robot_df['name'] == msg_topic, 'timestamp']
        timestamps_abs = [t_min] + [ts for ts in msg_timestamps]
        timestamps_rel = np.array(timestamps_abs)
        timestamps_rel -= t_min
        robot_ret_df = pd.DataFrame(
            timestamps_rel[1:], columns=['timestamp']
        )

        for topic in table_column_config:
            tmp_col = []

            if topic.topic_name.startswith('/'):
                robot_df_topic = df[
                    df['name'].str.match(topic.topic_name)
                ]
            else:
                robot_df_topic = robot_df[
                    robot_df['name'].str.match(r'\/.*\/{}'.format(topic.topic_name))
                ]

            for i, t in enumerate(timestamps_abs[1:], start=1):
                conf = data_aggregation_helper.TableConfig(
                    robot,
                    robot_df_topic,
                    t,
                    timestamps_abs[i-1],
                )
                try:
                    topic_data = topic(conf)
                except IndexError:
                    topic_data = None
                except (AttributeError, KeyError):  # No data
                    topic_data = None
                tmp_col.append(topic_data)
            robot_ret_df[topic.column_name] = tmp_col
        res[robot] = robot_ret_df
    return res

def db3_to_df(db3_file, table_column_config, step_size):
    data = read_rosbag_all_in_one(db3_file)

    # create pandas dataframe from raw data
    tables = aggregate_tables(data['rosbag'], table_column_config, step_size)
    
    # combine data for multiple robots into one dataframe
    for robot in tables.keys():
        tables[robot]['robot'] = robot
        
    
    df = pd.concat(tables.values()).reset_index(drop=True)
    df['db3'] = db3_file
    return df


def aggregate_many(db3_files, table_column_config, step_size=None):
    if step_size is None:
        step_size = 10**9

    with ThreadPool() as pool:
        results = pool.starmap(
            db3_to_df, [(f, table_column_config, step_size) for f in db3_files]
        )
    return pd.concat(results).reset_index(drop=True)

def main():
    """Program starts here."""
    my_parser = argparse.ArgumentParser(
        description='Aggregates the turtlebot data to one table'
    )

    my_parser.add_argument('db_file',
                           type=str,
                           help='path to the db3 database file')
    my_parser.add_argument('config_file',
                           type=str,
                           help='configuration to use inside the config folder without' +
                           " 'py' ending, e.g. basti")
    my_parser.add_argument('--out',
                           metavar='output_folder',
                           type=str,
                           help='path to the directory to store the exported csv files')
    my_parser.add_argument('--step_size',
                           type=int,
                           help='step size in nanoseconds, default: 1 second',
                           default=10**9)

    args = my_parser.parse_args()

    input_db_path = args.db_file
    config = args.config_file
    output_folder_path = args.out
    try:
        step_size = int(args.step_size)
    except ValueError:
        raise ValueError('Step Size could not be converted to int!')

    assert os.path.isfile(input_db_path),\
        'The input db file path specified does not exist'

    if output_folder_path and not os.path.exists(output_folder_path):
        os.makedirs(output_folder_path)

    try:
        data_aggregation = __import__('config.' + config, fromlist=[None])
    except ModuleNotFoundError:
        raise ModuleNotFoundError(('The configuration {} does not exist. Please specify ' +
                                   'a valid configuration file inside the config folder ' +
                                   'without the .py extension.').format(config))

    data_dict = read_rosbag_all_in_one(input_db_path)
    data = aggregate_tables(
        data_dict['rosbag'],
        data_aggregation.table_column_config,
        step_size
    )
    export_path_prefix = datetime.datetime.fromtimestamp(int(time.time())).isoformat()

    for robot, r_data in data.items():
        if not output_folder_path:
            print(robot + ':')
            print(r_data)
        else:
            csv_output_path = os.path.join(
                output_folder_path, export_path_prefix.replace(':', '-') + '_' + str(robot) + '.csv'
            )
            if os.path.exists(csv_output_path):
                raise Exception('File {} exists!'.format(csv_output_path))
            r_data.to_csv(csv_output_path)

    if output_folder_path:
        print('Wrote {} files!'.format(len(data)))  # TODO: add export_path_prefix to printout info


if __name__ == '__main__':
    main()
