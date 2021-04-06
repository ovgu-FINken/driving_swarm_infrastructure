#!/usr/bin/env python

import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
import numpy as np
from scipy import optimize
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from driving_swarm_messages.srv import VehicleModel, UpdateTrajectory
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, OccupancyGrid
from trajectory_generator.vehicle_model_node import (
    TrajectoryGenerator,
    Vehicle,
)
from time import time
from trajectory_generator.utils import *


class PSOPlanner(Node):
    def __init__(self):
        super().__init__("pso_planner")
        self.get_logger().info("Starting")
        self.own_frame = "base_link"
        self.reference_frame = "map"
        self.client_futures = []
        self.current_trajectory = None
        self.population = None

        self.w_time = 0.05
        self.w_length = 0.05
        self.w_danger = 0.5

        self.declare_parameter("vehicle_model")
        self.declare_parameter("step_size")
        self.declare_parameter("turn_radius")
        self.vm = TrajectoryGenerator(
            model=Vehicle(
                self.get_parameter("vehicle_model")
                .get_parameter_value()
                .integer_value
            ),
            step=self.get_parameter("step_size")
            .get_parameter_value()
            .double_value,
            r=self.get_parameter("turn_radius")
            .get_parameter_value()
            .double_value,
        )

        self.tfBuffer = tf2_ros.Buffer(
            cache_time=rclpy.time.Duration(seconds=10)
        )
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.costmap_initalised = False
        self.create_subscription(
            OccupancyGrid, "nav/obstacles", self.obstacle_cb, 9
        )

        self.follow_action_client = self.create_client(
            UpdateTrajectory, "nav/follow_trajectory"
        )
        self.follow_action_client.wait_for_service()
        self.get_logger().info("connected to trajectory follower service")
        self.goal = None
        f = self.tfBuffer.wait_for_transform_async(
            self.own_frame, self.reference_frame, rclpy.time.Time().to_msg()
        )
        self.get_logger().info("waiting for transform map -> baselink")
        rclpy.spin_until_future_complete(self, f)
        while not self.costmap_initalised:
            rclpy.spin_once(self)

        self.create_subscription(PoseStamped, "nav/goal", self.goal_cb, 9)
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info("setup done")

    def goal_cb(self, msg):
        if self.goal is None or self.goal.pose != msg.pose:
            self.get_logger().info(f"got goal")  #: {msg}')
        else:
            self.get_logger().debug("got old goal")
            return
        self.goal = msg
        self.current_trajectory = None
        self.population = None  # will be reninitialized

    def obstacle_cb(self, msg):
        self.costmap_initalised = True
        self.vm.set_costmap(msg)

    def cancel_all_current_actions(self):
        self.get_logger().info("cancel current actions")
        self.vm_client_futures = []
        request = UpdateTrajectory.Request()
        self.client_futures = [self.follow_action_client.call_async(request)]

    def timer_cb(self):
        for future in self.client_futures:
            if future.done():
                self.current_trajectory = future.result().trajectory
                self.client_futures.remove(future)
        if self.goal is not None:
            self.compute_path()

    def weighted_sum(self, traj):
        objectives = self.vm.compute_trajectory_objectives(traj)
        return (
            self.w_time * objectives["time"]
            + self.w_length * objectives["length"]
            + self.w_danger * objectives["danger"]
        )

    def evaluate_solution(self, wps):
        trajectory = self.vm.tuples_to_path(
            [self.start_wp] + wps + [self.goal_wp]
        )
        return self.weighted_sum(trajectory)

    def fitness(self, x):
        x = x[~np.isnan(x)]
        wps = [(x, y, t, 1.0) for x, y, t in zip(x[0::3], x[1::3], x[2::3])]
        return self.evaluate_solution(wps)

    def pso(
        self,
        population=None,
        pop_size=20,
        c_social=1.4,
        c_cognitive=1.4,
        w=0.5,
        granularity_0=1,
    ):
        # Multi-Granularity PSO
        if population is None:
            population = np.random.rand(pop_size, granularity_0 * 3)
            for i, w in enumerate(np.linspace(0.1, 0.9, num=5)):
                population[i, 0] = self.start_wp[0] * w + self.goal_wp[0] * (
                    w - 1.0
                )
                population[i, 1] = self.start_wp[1] * w + self.goal_wp[1] * (
                    w - 1.0
                )
                population[i, 2] = self.start_wp[2] * w + self.goal_wp[2] * (
                    w - 1.0
                )

        v_t = np.zeros_like(population)
        pbest = population.copy()
        pbest_fitness = []
        for p in population:
            pbest_fitness.append(self.fitness(p))
        gbest_fitness = np.min(pbest_fitness)
        gbest = pbest[pbest_fitness.index(gbest_fitness)]

        end_time = time() + 0.1
        n_gen = 0
        while time() < end_time:
            n_gen += 1
            v_cognitive = (
                c_cognitive * np.random.rand(*population.shape) * pbest
                - population
            )
            v_cognitive = (
                c_cognitive * np.random.rand(*population.shape) * pbest
                - population
            )
            v_social = (
                c_social * np.random.rand(*population.shape) * gbest
                - population
            )
            population_ = (
                w * v_t
                + v_cognitive
                + v_social
                + population
                + 0.001 * (np.random.rand(*population.shape) - 0.5)
            )
            np.clip(population_, -5.0, 5.0, population_)
            v_t = population_ - population
            population = population_

            for i, p in enumerate(population):
                f = self.fitness(p)
                if f < pbest_fitness[i]:
                    pbest_fitness[i] = f
                    pbest[i] = p
            gbest_fitness = np.min(pbest_fitness)
            gbest = pbest[pbest_fitness.index(gbest_fitness)]
        x = gbest[~np.isnan(gbest)]
        wps = [(x, y, t, 1.0) for x, y, t in zip(x[0::3], x[1::3], x[2::3])]
        self.get_logger().info(
            f"gen: {n_gen}"
        )  # < rclpy.time.Duration(seconds=0.5):
        return wps, population

    def compute_path(self):
        waypoint_tuples, ti = self.get_waypoints()

        self.start_wp = waypoint_tuples[0]
        if self.start_wp is None:
            return

        current = None
        if self.current_trajectory is not None and ti != 0:
            current = [
                (
                    p.pose.position.x,
                    p.pose.position.y,
                    yaw_from_orientation(p.pose.orientation),
                )
                for p in self.current_trajectory.poses[ti:]
            ]
        trivial = self.vm.tuples_to_path(waypoint_tuples)
        self.goal_wp = waypoint_tuples[1]

        opt, self.population = self.pso(population=self.population)
        # optimize.minimize(self.evaluate_solution, [.0, .0, .0, 1.0],method='Nelder-Mead')
        best = self.vm.tuples_to_path([self.start_wp] + opt + [self.goal_wp])
        # check if better than trivial solution
        if self.weighted_sum(trivial) < self.weighted_sum(best):
            best = trivial

        if current is not None and self.weighted_sum(
            current
        ) < self.weighted_sum(best):
            self.get_logger().info("using trivial solution")
            if ti > 0:
                return  # we currently use the current trajectory anyways

        self.get_logger().info(
            f"fitness: {self.vm.compute_trajectory_objectives(best)}"
        )
        # self.vm.plot_trajectory_in_costmap(best)
        self.send_path(
            [Pose2D(x=x, y=y, theta=theta) for x, y, theta in best], ti=ti
        )

    def send_path(self, trajectory, ti=0):
        # convert trajectory to correct space
        path = Path()
        path.header.frame_id = self.reference_frame
        for pose in trajectory:
            pose3d = PoseStamped()
            pose3d.header.frame_id = self.reference_frame
            pose3d.header.stamp = self.get_clock().now().to_msg()
            pose3d.pose.position.x = pose.x
            pose3d.pose.position.y = pose.y
            pose3d.pose.position.z = 0.0
            pose3d.pose.orientation = yaw_to_orientation(pose.theta)

            path.poses.append(pose3d)

        # create follow trajectory action goal
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=ti)
        self.client_futures.append(
            self.follow_action_client.call_async(request)
        )

    def get_waypoints(self):
        start = None
        ti = 0
        if self.current_trajectory is not None:
            ti = self.get_trajectory_index(self.current_trajectory, offset=1.0)

        if ti is None or ti == 0:
            ti = 0
            try:
                trans = self.tfBuffer.lookup_transform(
                    self.reference_frame,
                    self.own_frame,
                    rclpy.time.Time().to_msg(),
                )
                frame = tf2_kdl.transform_to_kdl(trans)
                start = (frame.p.x(), frame.p.y(), frame.M.GetRPY()[2])

            except Exception as e:
                self.get_logger().info(f"Exception in tf transformations\n{e}")
        else:
            start = (
                self.current_trajectory.poses[ti].pose.position.x,
                self.current_trajectory.poses[ti].pose.position.y,
                yaw_from_orientation(
                    self.current_trajectory.poses[ti].pose.orientation
                ),
            )

        goal = (
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            yaw_from_orientation(self.goal.pose.orientation),
        )

        return [start, goal], ti

    def convert_map_data_to_image(self, map_data):
        image = np.array(map_data.data, dtype=int)
        image = np.reshape(image, (map_data.info.width, map_data.info.height))
        image[image < 50] = 0
        image[image >= 50] = 100
        return image

    def get_trajectory_index(self, trajectory, offset=0.0):
        NANO = 0.001 ** 3
        current_stamp = self.get_clock().now() + rclpy.time.Duration(
            seconds=offset
        )
        trajectory_start = rclpy.time.Time.from_msg(
            self.current_trajectory.header.stamp
        )
        rate = 1.0
        t = ((current_stamp - trajectory_start).nanoseconds * NANO) * rate
        if t < 0.0:
            self.get_logger().warn("trajectory start is in the future")
            t = 0.0

        last_index = int(np.floor(t))
        if last_index + 2 >= len(self.current_trajectory.poses):
            return None
        return last_index


def main():
    rclpy.init()
    node = PSOPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting Down")
        node.destroy_node()


if __name__ == "__main__":
    main()
