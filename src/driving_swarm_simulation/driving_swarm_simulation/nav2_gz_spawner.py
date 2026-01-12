"""Script used to spawn a robot in a generic position."""
import argparse
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from lifecycle_msgs.srv import GetState
import time
from scipy.spatial.transform import Rotation
from driving_swarm_utils.node import DrivingSwarmNode


class Spawner(DrivingSwarmNode):
    def __init__(self, args):
        super().__init__(f'spawner_{args.robot_name}')
        self.args = args
        topic = f'{args.robot_namespace}/initialpose'
        self.pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.spawn_robot(args)

    def spawn_robot(self, args):
        # Get input arguments from user
        # Start self
        self.get_logger().info('Creating Service client to connect to `/world/default/create`')
        self.get_logger().info(f'args={args}')
        
        # Gazebo verwendet jetzt /world/<world_name>/create statt /spawn_entity
        client = self.create_client(SpawnEntity, '/world/default/create')
        
        self.get_logger().info('Connecting to `/world/default/create` service...')
        if not client.service_is_ready():
            client.wait_for_service()
            self.get_logger().info('...connected!')

        self.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.robot_name, args.robot_namespace, args.x, args.y, args.z))

        # Get path to the robot's sdf file
        if args.turtlebot_type is not None:
            sdf_file_path = os.path.join(
                get_package_share_directory('turtlebot3_gazebo'), 'models',
                'turtlebot3_{}'.format(args.turtlebot_type), 'model.sdf')
        else:
            sdf_file_path = args.sdf

        self.get_logger().info(f"sdf: {sdf_file_path}")

        # Parse and modify SDF for Gazebo (nicht Gazebo Classic)
        tree = ET.parse(sdf_file_path)
        root = tree.getroot()
        
        # Ändere den Modellnamen im SDF
        model = root.find('model')
        if model is not None:
            model.set('name', args.robot_name)
        
        # Für Gazebo: Aktualisiere Plugin-Namen und Remappings
        for plugin in root.iter('plugin'):

            plugin_name = plugin.get('filename', '')
            
            if 'diff_drive' in plugin_name.lower() or 'libgazebo_ros_diff_drive' in plugin_name:
                
                if 'libgazebo_ros_diff_drive' in plugin_name:
                    plugin.set('filename', 'gz-sim-diff-drive-system')
                    plugin.set('name', 'gz::sim::systems::DiffDrive')
                
                
                ros_params = plugin.find('ros')
                if ros_params is None:
                    ros_params = ET.SubElement(plugin, 'ros')
                
                
                namespace_elem = ros_params.find('namespace')
                if namespace_elem is None:
                    namespace_elem = ET.SubElement(ros_params, 'namespace')
                namespace_elem.text = args.robot_namespace
                
                
                ros_tf_remap = ET.SubElement(ros_params, 'remapping')
                ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'

        
        rot = Rotation.from_euler('xyz', [0.0, 0.0, args.yaw])
        quat = rot.as_quat()  # Returns [x, y, z, w]
        
        
        pose = Pose()
        pose.position.x = args.x
        pose.position.y = args.y
        pose.position.z = args.z
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        
        self.initial_pose = pose
        
        # Erstelle EntityFactory Message
        entity_factory = EntityFactory()
        entity_factory.sdf = ET.tostring(root, encoding='unicode')
        entity_factory.name = args.robot_name
        entity_factory.allow_renaming = False
        entity_factory.pose = pose
        
        # Set data for request
        # ros_gz_interfaces/srv/SpawnEntity hat folgende Struktur:
        # ros_gz_interfaces/EntityFactory entity_factory
        # ---
        # bool success
        request = SpawnEntity.Request()
        request.entity_factory = entity_factory
        
        self.get_logger().info('Sending service request to `/world/default/create`')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            print('response: %r' % future.result())
            if future.result().success:
                self.get_logger().info(f'Successfully spawned entity')
            else:
                self.get_logger().error(f'Failed to spawn entity')
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())
        
        self.done = True

    def wait_for_localization(self):
        request = GetState.Request()
        topic = f'/{self.args.robot_name}/amcl/get_state'
        client = self.create_client(GetState, topic)
        
        if not client.service_is_ready():
            self.get_logger().info(f'waiting for service {topic}')
            client.wait_for_service()
            self.get_logger().info(f'connected to state service')
        
        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                print('response: %r' % future.result())
                self.get_logger().info(f'{future.result()}')
                if future.result().current_state.id == 3:
                    break
            else:
                raise RuntimeError(
                    'exception while calling service: %r' % future.exception())
            time.sleep(1.0)
        
        self.send_initial_pose()

    def send_initial_pose(self):
        time.sleep(5.0)
        # Send initial pose
        self.get_logger().info('Sending initial pose')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position = self.initial_pose.position
        pose.pose.pose.orientation = self.initial_pose.orientation
        
        self.pub.publish(pose)
        self.get_logger().info('Done! Shutting down self.')
        
        if self.done:
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo with navigation2')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0.0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0.0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0.0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('-yaw', type=float, default=0.0,
                        help='rotation of the initial position in [rad]')
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-t', '--turtlebot_type', type=str,
                       choices=['waffle', 'burger'])
    group.add_argument('-s', '--sdf', type=str,
                       help="the path to the robot's model file (sdf)")
    
    args, _ = parser.parse_known_args()
    
    rclpy.init()
    node = Spawner(args)
    node.wait_for_localization()
    node.destroy_node()


if __name__ == '__main__':
    main()