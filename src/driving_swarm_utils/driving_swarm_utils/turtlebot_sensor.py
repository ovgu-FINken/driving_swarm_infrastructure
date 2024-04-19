#!/usr/bin/python3

from driving_swarm_utils.node import main_fn, DrivingSwarmNode
from driving_swarm_utils.utils import detect_tb_from_ranges
from sensor_msgs.msg import LaserScan


class TurtlebotSensor(DrivingSwarmNode):
    def __init__(self, name: str='turtlebot_sensor') -> None:
        super().__init__(name)
        self.create_subscription(LaserScan, 'scan', self.laser_cb, 100)
    
    def laser_cb(self, msg):
        tb = detect_tb_from_ranges(msg.ranges)
    


def main():
    main_fn('turtlebot_sensor', TurtlebotSensor)


if __name__ == '__main__':
    main()