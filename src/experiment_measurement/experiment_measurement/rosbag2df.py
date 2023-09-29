import rclpy
from rclpy.node import Node, ParameterNotDeclaredException
import rosidl_runtime_py

import sqlite3
import pandas as pd
from datetime import datetime
import os
import copy

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def read_rosbag_per_topic(db_file_path):
    """
    @brief: read the data from a rosbag file, i.e. a sqlite3 database file created by rosbag2 while recording
    @args: db_file_path: absolute path to the sqlite3 database file to load
    @returns: a dictionary of dataframes, each df contains all messages of one topic; the topic name is the key
    """

    # get the data from the database
    cnx = sqlite3.connect(db_file_path)
    df_topics = pd.read_sql_query("SELECT * FROM topics", cnx)
    dict_df = {}

    # for each topic in the db-file create a csv-file named after the topic plus the timestamp
    for topic_id, topic_name, topic_type in zip(df_topics['id'], df_topics['name'], df_topics['type']):
        df_msgs = pd.read_sql_query(
            f"SELECT * FROM messages WHERE topic_id={topic_id}", cnx)
        msg_type = get_message(topic_type)
        msgs = []
        if not df_msgs.empty:
            # deserialize and rename columns
            for msg_b in df_msgs['data']:
                msgs.append(deserialize_message(msg_b, msg_type))
            df_msgs = df_msgs.replace(topic_id, topic_name)
            df_msgs = df_msgs.rename(columns={"topic_id": "topic_name"})
            df_msgs = df_msgs.rename(columns={"data": "data_bytes"})
            df_msgs['data'] = pd.Series(msgs)
            del df_msgs['data_bytes']
            dict_df[f'{topic_name}'] = df_msgs

    return dict_df

def read_rosbag_all_in_one(db_file_path, topics=None):
    """
    @brief: read the data from a rosbag file, i.e. a sqlite3 database file created by rosbag2 while recording
    @args: db_file_path: absolute path to the sqlite3 database file to load
    @returns: a dataframe containing all messages with topic name and serialized data
    """

    # get the data from the database
    cnx = sqlite3.connect(db_file_path)
    if topics is None:
        topics = ["%goal_completed", "/tf", "/clock", "/command", "%status", "%cmd_vel", "%plan"]
    topics_str = [f" topics.name LIKE '{t}' " for t in topics]
    topics_str = "OR".join(topics_str)
    df = pd.read_sql_query(f"SELECT messages.*, topics.name, topics.type FROM messages JOIN topics ON messages.topic_id==topics.id WHERE {topics_str}", cnx)
    dict_df = {}

    msgs = []
    for _, row in df.iterrows():
        msg_type = get_message(row['type'])
        # deserialize and rename columns
        msgs.append(deserialize_message(row['data'], msg_type))
    df = df.rename(columns={"data": "data_bytes"})
    df['data'] = pd.Series(msgs)
    del df['data_bytes']
    dict_df['rosbag'] = df

    return dict_df


class Rosbag2Df(Node):
    def __init__(self):
        super().__init__('rosbag_2_df')

        # parameters
        self.declare_parameter('db_file_path')
        self.declare_parameter('csv_file_dir', value="")

        csv_file_dir = self.get_parameter('csv_file_dir').value
        try:
            db_file_path = self.get_parameter('db_file_path').value
        except ParameterNotDeclaredException:
            print("Please, set a value for db_file_path parameter.")
            raise KeyboardInterrupt

        ##### prepare file and path names #####
        # get the current timestamp in readable format for better identification of the data
        ts = self.get_clock().now().nanoseconds / 1e9
        ts = datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')

        # adapt path expressions
        if csv_file_dir.strip():
            if not csv_file_dir[-1] == "/":
                csv_file_dir += "/"
            if "~" in csv_file_dir:
                csv_file_dir = csv_file_dir.replace("~", "")
            elif "~/" in csv_file_dir:
                csv_file_dir = csv_file_dir.replace("~/", "")

        # create folder for better overview of stored data
        file_dir = f'{csv_file_dir}{ts}_exp_data'
        os.makedirs(file_dir)

        # read the data from the rosbag file
        dict_df = read_rosbag_per_topic(db_file_path)
        dict_rosbag = read_rosbag_all_in_one(db_file_path)

        # save as csv files
        for topic_name in dict_df:
            file_name = f'{ts}_{topic_name.replace("/", "_")}.csv'
            path = file_dir + '/' + file_name
            dict_df[topic_name].to_csv(path, sep=",")
        file_name = f'{ts}_all_topics.csv'
        path = file_dir + '/' + file_name
        dict_rosbag['rosbag'].to_csv(path, sep=",")

        self.get_logger().info(f'All csv files stored in {file_dir}.')


def main():
    rclpy.init()
    node = Rosbag2Df()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'got keyboard interrupt, shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
