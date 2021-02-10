import rclpy
from rclpy.node import Node

import sqlite3
import pandas as pd


class Rosbag2Df(Node):
    def __init__(self):
        super().__init__('rosbag_2_df')

    path = r'/home/traichel/Downloads/Telegram Desktop/'
    db = path + 'rosbag2_2021_02_09-15_11_58_0.db3'

    cnx = sqlite3.connect(db)
    df_topics = pd.read_sql_query("SELECT * FROM topics", cnx)
    # df_msgs = pd.read_sql_query("SELECT * FROM messages", cnx)

    df_dict = {}

    for topic_id, topic_name in zip(df_topics['id'], df_topics['name']):
        df_msgs = pd.read_sql_query(f"SELECT * FROM messages WHERE topic_id={topic_id}", cnx)
        df_msgs.replace(topic_id, topic_name)
        df_dict[f'{topic_name}'] = df_msgs
    
def main():
    rclpy.init()
    node = Rosbag2Df()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'got keyboard interrupt, shutting down')
        node.destroy_node()


if __name__ == '__main__':
    main()
