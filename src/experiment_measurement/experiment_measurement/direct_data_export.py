import rclpy
import pandas as pd
from termcolor import colored
from driving_swarm_utils.node import DrivingSwarmNode
from functools import partial
from geometry_msgs.msg import Twist
import yaml

def get_topic_type_from_str(topic_type):
    if topic_type == "Int32":
        from std_msgs.msg import Int32
        return Int32
    if topic_type == "String":
        from std_msgs.msg import String
        return String
    else:
        raise ValueError(f"unknown topic type {topic_type}")

class DirectDataExport(DrivingSwarmNode):
    def __init__(self):
        super().__init__("direct_data_export")
        self.get_list_of_robot_names()
        self.get_logger().info(f"robots: {self.robots}")
        self.data = []
        self.t = 0

        self.column_generators = {
            "t":self.column_t, 
            "robot": self.column_name,
        }

        self.declare_parameter("data_export_config_file", "data_export_config.yaml")
        self.data_export_config_file = self.get_parameter("data_export_config_file").get_parameter_value().string_value
        # read data export config
        self.get_logger().info(f"reading data export config from {self.data_export_config_file}")
        with open(self.data_export_config_file, "r") as f:
            self.data_export_config = yaml.safe_load(f)
        self.get_logger().info(f"data export config: {self.data_export_config}")


        self.declare_parameter("data_file", "data.csv.gz")
        self.data_file = self.get_parameter("data_file").get_parameter_value().string_value
        self.get_logger().info("data file: " + colored(self.data_file, "blue"))

        
        # create subscription for cmd_vel
        if self.data_export_config["cmd_vel"]:
            self.column_generators |= {
                "cmd_vel_x": self.column_vx,
                "cmd_vel_rot": self.column_vrot
            }
            self._cmd_vels = {}
            for robot in self.robots:
                self.get_logger().info(f"subscribing to cmd_vel {robot}")
                self.create_subscription(Twist, f"/{robot}/cmd_vel", partial(self.cmd_vel_cb, robot=robot), 10)

        # setup global topic subscribers
        if self.data_export_config["global_topics"]:
            self.global_topic_msgs = {}
            for topic_name, topic_type in self.data_export_config["global_topics"].items():
                self.get_logger().info(f"subscribing to {topic_name}")
                self.create_subscription(get_topic_type_from_str(topic_type), topic_name, partial(self.global_topic_cb, topic_name=topic_name), 10)
            self.column_generators[topic_name] = partial(self.global_topic_column, topic_name=topic_name)
            

        # setup local topic subscribers
        if self.data_export_config["robot_topics"]:
            self.robot_topic_msgs = {}
            for topic_name, topic_type in self.data_export_config["robot_topics"].items():
                self.robot_topic_msgs[topic_name] = {}
                self.get_logger().info(f"subscribing to {topic_name}")
                for robot in self.robots:
                    self.create_subscription(get_topic_type_from_str(topic_type), f"/{robot}/{topic_name}", partial(self.robot_topic_cb, robot=robot, topic_name=topic_name), 10)
                self.column_generators[topic_name] = partial(self.robot_topic_column, topic_name=topic_name)

        if self.data_export_config["tf"]:
            self.setup_tf()
            self.own_frame = self.robots[0]
            self.reference_frame = "world"
            #self.wait_for_tf()

            self.column_generators |= {
                "x": self.column_x,
                "y": self.column_y,
                "theta": self.column_theta,
            }
        
        self.create_timer(1.0, self.timer_cb)

    def robot_topic_column(self, robot, topic_name):
        if topic_name not in self.robot_topic_msgs:
            self.robot_topic_msgs[topic_name] = {}
        if robot not in self.robot_topic_msgs[topic_name]:
            self.robot_topic_msgs[topic_name][robot] = None
        return self.robot_topic_msgs[topic_name][robot]

    def robot_topic_cb(self, msg, robot=None, topic_name=None):
        self.robot_topic_msgs[topic_name][robot] = msg.data

    def global_topic_column(self, robot, topic_name):
        if topic_name not in self.global_topic_msgs:
            self.global_topic_msgs[topic_name] = None
        return self.global_topic_msgs[topic_name]

    def global_topic_cb(self, msg, topic_name=None):
        self.global_topic_msgs[topic_name] = msg.data

    def timer_cb(self):
        if self.data_export_config["tf"]:
            self.get_robot_poses()
        self.append_time_step()
        self.t+=1

    def append_time_step(self):
        # for each each column: go through each robot and get the data
        # self.column_generator contains a dict with column names as keys and functions as values
        # the functions return the column data for a single robot
        data_dict = {column_name: [column_data(robot) for robot in self.robots] for column_name, column_data in self.column_generators.items()}
        #self.get_logger().info("\n" + str(data_dict))
        data = pd.DataFrame(data_dict)
        #self.get_logger().info("\n" + str(data))
        self.data.append(data)
        
    def column_t(self, robot):
        return self.t

    def column_name(self, robot):
        return robot
    
    def column_x(self, robot):
        if robot not in self.robot_poses:
            return None
        if not self.robot_poses[robot]:
            return None
        return self.robot_poses[robot][0]
    
    def column_y(self, robot):
        if robot not in self.robot_poses:
            return None
        if not self.robot_poses[robot]:
            return None
        return self.robot_poses[robot][1]
    
    def column_theta(self, robot):
        if robot not in self.robot_poses:
            return None
        if not self.robot_poses[robot]:
            return None
        return self.robot_poses[robot][2]
    
    def column_vx(self, robot):
        if robot not in self._cmd_vels:
            return None
        return self._cmd_vels[robot].linear.x

    def column_vrot(self, robot):
        if robot not in self._cmd_vels:
            return None
        return self._cmd_vels[robot].angular.z

    def save_data(self):
        df = pd.concat(self.data, ignore_index=True)
        df.to_csv(self.data_file, index=False)
        self.get_logger().info(f"saving data to {self.data_file}")
        self.get_logger().info(f"the data:\n{df.head(20)}")
        
    def cmd_vel_cb(self, msg, robot):
        self._cmd_vels[robot] = msg

    def get_robot_poses(self):
        self.robot_poses = {}
        self.robot_poses = {robot : self.get_tf_pose(frame=robot) for robot in self.robots}

    def get_robot_pose(self, robot):
        self.robot_poses[robot] = self.lookup_tf_pose("world", robot)


def main():
    rclpy.init()
    node = DirectDataExport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'got keyboard interrupt, shutting down')
    except Exception as e:
        node.get_logger().error(e)
    finally:
        node.save_data()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    

if __name__ == "__main__":
    main()
    
