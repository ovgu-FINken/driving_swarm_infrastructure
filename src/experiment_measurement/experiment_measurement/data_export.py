
import sqlite3
import pandas as pd
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import logging
from transformations import euler_from_quaternion
from scipy import interpolate

def read_rosbag_all_in_one(db_file_path, topics=None):
    """
    @brief: read the data from a rosbag file, i.e. a sqlite3 database file created by rosbag2 while recording
    @args: db_file_path: absolute path to the sqlite3 database file to load
    @returns: a dataframe containing all messages with topic name and serialized data
    """

    # get the data from the database
    cnx = sqlite3.connect(db_file_path)
    if topics is None:
        topics = ["%goal_completed", "/tf", "/clock", "/command", "%status", "%cmd_vel", "%current_node"]
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

    return df

def clock_to_s(timestamp, name, type, data):
    return {'timestamp': timestamp, 'sim_time': data.clock.sec + data.clock.nanosec * 1e-9}

def tf_to_xyt(timestamp, name, type, data):
    x = data.transforms[0].transform.translation.x
    y = data.transforms[0].transform.translation.y
    q = data.transforms[0].transform.rotation
    theta = euler_from_quaternion([q.w, q.x, q.y, q.z])[2]
    
    return {'timestamp': timestamp,
            'x': x,
            'y': y,
            'theta': theta,
            'robot': data.transforms[0].child_frame_id}

def get_name_from_topic(topic):
    return topic.split('/')[1]

def get_topic_variable(topic):
    return topic.split('/')[-1]

def get_atomic_value(timestamp, name, type, data):
    if name.startswith('/robot') or name.startswith('/turtlebot'):
        return {'timestamp': timestamp, 'robot': get_name_from_topic(name), get_topic_variable(name): data.data}
    return {'timestamp': timestamp, get_topic_variable(name): data.data}

def twist_to_vw(timestamp, name, type, data):
    topic = get_topic_variable(name)
    return {'timestamp': timestamp, 'robot': get_name_from_topic(name), f'{topic}_trans': data.linear.x, f'{topic}_rot': data.angular.z}

class DataConverter:
    # this class is used to convert the data from the raw msg data to a numeric column(s)

    def __init__(self):
        self._types_func = {}
        self.register_type('rosgraph_msgs/msg/Clock', clock_to_s)
        self.register_type('tf2_msgs/msg/TFMessage', tf_to_xyt)
        self.register_type('geometry_msgs/msg/Twist', twist_to_vw)
        self.register_type('std_msgs/msg/Int32', get_atomic_value)
        self.register_type('std_msgs/msg/Float32', get_atomic_value)
        self.register_type('std_msgs/msg/String', get_atomic_value)

    def register_type(self, type_name, func):
        # register a function for a type
        # the function is called with the data, name and type of a msg
        # the function should return a dict, containing column names and numeric values
        self._types_func[type_name] = func
        
    def _convert_row(self, row):
        # convert a row of a dataframe
        # the row contains the data, name and type of a msg
        # the function should return a dict, containing column names and numeric values
        if row['type'] in self._types_func:
            return self._types_func[row['type']](row['timestamp'], row['name'], row['type'], row['data'])
         
    def convert_df(self, df):
        
        # convert the data in the dataframe by using the registered functions
        for type_name in df.type.unique():
            if type_name not in self._types_func:
                topics = df[df.type == type_name].name.unique()
                logging.warn('type not registered: ' + type_name)
                logging.warn('topics: ' + str(topics))
        out = [self._convert_row(row) for _, row in df.iterrows() if row['type'] in self._types_func]
        out = pd.DataFrame.from_records(out)
        out["timestamp"] = pd.to_timedelta(out["timestamp"], unit='ns')
        if "sim_time" not in out.columns:
            out["t"] = out["timestamp"]
        else:
            out["t"] = pd.to_timedelta(out["sim_time"], unit='s')
            out.set_index("timestamp", inplace=True)
            out["t"].interpolate(inplace=True, method='linear', limit_direction='forward', axis=0)
            out.reset_index(inplace=True)
        if len(out.loc[out.command.eq('go')]):
            out.t = out.t - out.loc[out.command.eq('go')].t.min()    
            out.timestamp = out.timestamp - out.loc[out.command.eq('go')].timestamp.min()
        out.drop(index=out.loc[out.t.lt("0")].index, inplace=True)
        return out.groupby(["robot", pd.Grouper(key='t', freq='1s')], dropna=False).first().reset_index()
        
if __name__ == "__main__":
    data = read_rosbag_all_in_one("/home/semai/data/all_experiments/all_experiments/ccr.1.sim/rosbag_1 robots_2023-09-12_20:13:12_0.db3")
    converter = DataConverter()
    data = converter.convert_df(data)
