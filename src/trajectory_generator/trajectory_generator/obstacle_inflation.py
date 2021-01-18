#!/usr/bin/env python

import numpy as np
import PyKDL
from skimage import io, segmentation
from scipy.ndimage import gaussian_filter

import rclpy
import tf2_ros
import tf2_geometry_msgs
import tf2_py
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan

def yaw_from_orientation(orientation):
    rot = PyKDL.Rotation.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
    return rot.GetRPY()[2]

class InflationNode(Node):
    def __init__(self):
        super().__init__('inflation_node')
        self.get_logger().info('Starting')
        self.robot_frames = ['robot1', 'robot2']
        self.get_logger().info(self.get_namespace())
        self.robot_frames.remove(self.get_namespace().split('/')[1])
        self.own_frame = 'base_link'
        self.reference_frame = 'map'
        
        self.safety_dist = 0.20

        self.tfBuffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=3.0))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        self.raw_map = None
        self.create_subscription(OccupancyGrid, 'map', self.map_cb, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'nav/obstacles', 10)
        self.scan = None
        self.create_subscription(LaserScan, 'scan', self.scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.5, self.timer_cb)
        self.map_client = self.create_client(GetMap, 'map_server/map')
        self.map_client.wait_for_service()
        future = self.map_client.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        self.update_map(future.result().map)
        self.get_logger().info('init done')
        
    def scan_cb(self, msg):
        self.scan = msg
        
    def scan_image(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.raw_map.header.frame_id, self.scan.header.frame_id, rclpy.time.Time().to_msg())
        except Exception as e:
            self.get_logger().warn(f'{e}')
            return np.zeros_like(self.map_image)
        
        scan_img = np.zeros_like(self.map_image)
        
        own_theta = yaw_from_orientation(trans.transform.rotation)
        r_max = 2.0

        for i, r in enumerate(self.scan.ranges):
            if r > r_max:
                continue
            x = trans.transform.translation.y + r * np.sin(own_theta + i / 180.0 * np.pi)
            y = trans.transform.translation.x + r * np.cos(own_theta + i / 180.0 * np.pi)
            x, y = self.metric_to_px_coorditaes(x, y)
            if x < 0 or x >= self.raw_map.info.width:
                continue
            if y < 0 or y >= self.raw_map.info.height:
                continue
            scan_img[x, y] = 100
        return scan_img
        
    def convert_map_data_to_image(self, map_data):
        image = np.array(map_data.data, dtype=int)
        image = np.reshape(image, (map_data.info.width, map_data.info.height))
        image[image < 50] = 0
        image[image >= 50] = 100
        return image
    
    def metric_to_px_coorditaes(self, x, y):
        x = (x - self.raw_map.info.origin.position.x ) / self.raw_map.info.resolution
        y = (y - self.raw_map.info.origin.position.y ) / self.raw_map.info.resolution
        return int(x), int(y)

    def transform_to_px_coordinates(self, t):
        y = (t.transform.translation.x - self.raw_map.info.origin.position.x) / self.raw_map.info.resolution
        x = (t.transform.translation.y - self.raw_map.info.origin.position.y) / self.raw_map.info.resolution
        return int(x), int(y)

    def combine_sensor_sources(self):
        other = np.zeros_like(self.map_image)
        for _, robot_frame in enumerate(self.robot_frames):
            try:
                trans = self.tfBuffer.lookup_transform(self.raw_map.header.frame_id, robot_frame, rclpy.time.Time().to_msg())
                x, y = self.transform_to_px_coordinates(trans)
                #self.get_logger().info(f'{robot_frame}: {x}, {y}')
                other[x][y] = 100
            except Exception as e:
                self.get_logger().warn(f'{e}')
        other = segmentation.expand_labels(other, distance=2.0 * self.safety_dist/self.raw_map.info.resolution)
        image = segmentation.expand_labels(self.map_image, distance=self.safety_dist/self.raw_map.info.resolution)
        scan_image = segmentation.expand_labels(self.scan_image(), distance=self.safety_dist/self.raw_map.info.resolution)
        image = gaussian_filter(image + other + scan_image, 5)
        image[image > 100] = 100
        image[image <= 0] = 0
        return image
    
    def convert_image_to_map_data(self, image):
        map_data = self.raw_map
        map_data.data = [int(x) for x in image.flat]
        return map_data
    
    def update_map(self, map_msg):
        self.raw_map = map_msg
        #extract map as image
        self.map_image = self.convert_map_data_to_image(map_msg)
    
    def map_cb(self, msg):
       self.get_logger().info('received a map') 
       self.update_map(msg)
       
    def timer_cb(self):
        if self.raw_map is not None:
            self.map_publisher.publish(self.convert_image_to_map_data(self.combine_sensor_sources()))
        else:
            self.get_logger().info('no map received yet')

   
def main():
    rclpy.init()
    node = InflationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting Down')
        node.destroy_node()

if __name__ == "__main__":
    main()
