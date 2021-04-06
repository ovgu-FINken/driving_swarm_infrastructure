# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Script used to spawn a robot in a generic position."""

import argparse
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
import PyKDL


class Spawner(Node):
    def __init__(self, args):
        super().__init__(f"spawner_{args.robot_name}")
        self.args = args
        topic = f"{args.robot_namespace}/initialpose"
        self.pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.spawn_robot(args)

    def spawn_robot(self, args):
        # Get input arguments from user
        # Start self
        self.get_logger().info(
            "Creating Service client to connect to `/spawn_entity`"
        )
        self.get_logger().info(f"args={args}")
        client = self.create_client(SpawnEntity, "/spawn_entity")

        self.get_logger().info("Connecting to `/spawn_entity` service...")
        if not client.service_is_ready():
            client.wait_for_service()
            self.get_logger().info("...connected!")

        self.get_logger().info(
            "spawning `{}` on namespace `{}` at {}, {}, {}".format(
                args.robot_name, args.robot_namespace, args.x, args.y, args.z
            )
        )

        # Get path to the robot's sdf file
        if args.turtlebot_type is not None:
            sdf_file_path = os.path.join(
                get_package_share_directory("turtlebot3_gazebo"),
                "models",
                "turtlebot3_{}".format(args.turtlebot_type),
                "model.sdf",
            )
        else:
            sdf_file_path = args.sdf
        self.get_logger().info(f"sdf: {sdf_file_path}")

        # We need to ***remap*** the transform (/tf) topic so each robot has its own.
        # We do this by adding `ROS argument entries` to the sdf file for
        # each plugin broadcasting a transform. These argument entries provide the
        # remapping rule, i.e. /tf -> /<robot_id>/tf
        tree = ET.parse(sdf_file_path)
        root = tree.getroot()
        for plugin in root.iter("plugin"):
            # TODO(orduno) Handle case if an sdf file from non-turtlebot is provided
            if "turtlebot3_diff_drive" in plugin.attrib.values():
                # The only plugin we care for now is 'diff_drive' which is
                # broadcasting a transform between`odom` and `base_footprint`
                break

        ros_params = plugin.find("ros")
        ros_tf_remap = ET.SubElement(ros_params, "remapping")
        ros_tf_remap.text = "/tf:=/" + args.robot_namespace + "/tf"

        # Set data for request
        request = SpawnEntity.Request()
        request.name = args.robot_name
        request.xml = ET.tostring(root, encoding="unicode")
        request.robot_namespace = args.robot_namespace
        request.initial_pose.position.x = args.x
        request.initial_pose.position.y = args.y
        request.initial_pose.position.z = args.z
        rot = PyKDL.Rotation.RPY(0.0, 0.0, args.yaw).GetQuaternion()
        request.initial_pose.orientation.x = rot[0]
        request.initial_pose.orientation.y = rot[1]
        request.initial_pose.orientation.z = rot[2]
        request.initial_pose.orientation.w = rot[3]

        self.initial_pose = request.initial_pose

        self.get_logger().info("Sending service request to `/spawn_entity`")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print("response: %r" % future.result())
        else:
            raise RuntimeError(
                "exception while calling service: %r" % future.exception()
            )
        self.done = True

    def wait_for_localization(self):
        request = GetState.Request()
        topic = f"/{self.args.robot_name}/amcl/get_state"
        client = self.create_client(GetState, topic)
        if not client.service_is_ready():
            self.get_logger().info(f"waiting for service {topic}")
            client.wait_for_service()
            self.get_logger().info("connected to state service")

        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                print("response: %r" % future.result())
                self.get_logger().info(f"{future.result()}")
                if future.result().current_state.id == 3:
                    break
            else:
                raise RuntimeError(
                    "exception while calling service: %r" % future.exception()
                )
        self.send_initial_pose()

    def send_initial_pose(self):

        # Send initial pose
        # geometry_msgs/msg/PoseWithCovarianceStamped
        self.get_logger().info("Sending initial pose")
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        # pose.header.time = self.get_clock().now().stamp()
        pose.pose.pose = self.initial_pose
        self.pub.publish(pose)

        self.get_logger().info("Done! Shutting down self.")
        if self.done:
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Spawn Robot into Gazebo with navigation2"
    )
    parser.add_argument(
        "-n",
        "--robot_name",
        type=str,
        default="robot",
        help="Name of the robot to spawn",
    )
    parser.add_argument(
        "-ns",
        "--robot_namespace",
        type=str,
        default="robot",
        help="ROS namespace to apply to the tf and plugins",
    )
    parser.add_argument(
        "-x",
        type=float,
        default=0.0,
        help="the x component of the initial position [meters]",
    )
    parser.add_argument(
        "-y",
        type=float,
        default=0.0,
        help="the y component of the initial position [meters]",
    )
    parser.add_argument(
        "-z",
        type=float,
        default=0.0,
        help="the z component of the initial position [meters]",
    )

    parser.add_argument(
        "-yaw",
        type=float,
        default=0.0,
        help="rotation of the initial position in [rad]",
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "-t", "--turtlebot_type", type=str, choices=["waffle", "burger"]
    )
    group.add_argument(
        "-s",
        "--sdf",
        type=str,
        help="the path to the robot's model file (sdf)",
    )

    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Spawner(args)
    node.wait_for_localization()
    node.destroy_node()


if __name__ == "__main__":
    main()
