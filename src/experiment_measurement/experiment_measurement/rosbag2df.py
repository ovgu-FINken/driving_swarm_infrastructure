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

        # get the data from the database
        cnx = sqlite3.connect(db_file_path)
        df_topics = pd.read_sql_query("SELECT * FROM topics", cnx)

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

        self.get_logger().info(str(csv_file_dir))
        # create folder for better overview of stored data
        file_dir = f'{csv_file_dir}{ts}_exp_data'
        os.makedirs(file_dir)

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
                # df_msgs.pop['data']
                df_msgs['data'] = pd.Series(msgs)

                # save as csv file
                file_name= f'{ts}_{topic_name.replace("/", "_")}.csv'
                path = file_dir + '/' + file_name
                df_msgs.to_csv(path, sep="|")

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
