import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import pandas as pd
from experiment_measurement.rosbag2df import read_rosbag_all_in_one

"""
    # this is how an aggregated dataframe should loook like *per robot*
    # these should be stored and returned as a dictionary
    df_aggregated_robot1 = pd.DataFrame({
        'timestamp': [],
        'trans_vel': [],
        'rot_vel': [], 
        'pos_x': [],
        'pos_y': [],
        'pos_theta': [],
        'travelled_dist': [],
        'min_obstacle_dist': [],
        'reached_goals': []
    })
"""

db_path = "/home/traichel/Downloads/rosbag2_2021_02_09-15_11_58_0.db3"
dict_all_in_one = read_rosbag_all_in_one(db_path)

#TODO