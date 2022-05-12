import rclpy

from driving_swarm_nav_graph.nav_graph import NavGraphNode

import numpy as np
import traceback
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32
from std_srvs.srv import Empty
from functools import partial
from itertools import groupby
import yaml

import polygonal_roadmaps as poro

class NavGraphGlobalPlanner(NavGraphNode):
    def __init__(self):
        super().__init__('global_planner')
        self.get_logger().info("Starting")
        self.own_frame = "base_link"
        self.reference_frame = "map"
        self.goal = None
        self.started = False
        self.current_trajectory = None

        self.declare_parameter('robot_names')
        self.robots = self.get_parameter('robot_names').get_parameter_value().string_array_value
        qos_profile = rclpy.qos.qos_profile_system_default
        qos_profile.reliability = (
            rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )
        qos_profile.durability = (
            rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )
        self.declare_parameter('planner_config')
        self.planner_config = self.get_parameter('planner_config').get_parameter_value().string_value
        self.get_logger().info(f'Planner config: {self.planner_config}')
            

        self.plans = None
        self.node_occupancies = {}
        self.robot_publishers = {}
        self.get_logger().info(str(self.robots))
        self.plan_publisher = self.create_publisher(String, f'/plan', qos_profile)
        for robot in self.robots:
            self.create_subscription(Int32, f"/{robot}/nav/cell", partial(self.cell_cb,robot=robot), 10)
            self.node_occupancies[robot] = None
            self.robot_publishers[robot] = self.create_publisher(String, f'/{robot}/nav/plan', qos_profile)
        self.create_timer(1.0, self.timer_cb)
    
    def timer_cb(self):
        # if no current plan: make plan
        self.get_logger().info(f'agents @: {self.node_occupancies}')
        if None in self.node_occupancies.values():
            return
        if self.plans is None:
            self.get_logger().info('Start planning')
            self.plans = self.make_plan()
            self.node_constraints = self.make_node_constraints(self.plans)
            self.get_logger().info('Plan creation done')
            self.get_logger().info(f'{self.plans}')
        # execute plans
        self.execute_plans(self.plans)

    def cell_cb(self, msg, robot=None):
        if robot in self.node_occupancies and self.plans is not None:
            old = self.node_occupancies[robot]
            new = msg.data
            if new != old:
                # when node occupancies change, we can remove old nodes from plan
                if new in self.plans[robot]:
                    i = self.plans[robot].index(new)
                    self.plans[robot] = self.plans[robot][i:]
                # remove constraint for node no longer occupied
                if old in self.node_constraints and \
                        self.node_constraints[old] and \
                        self.node_constraints[old][0] == robot:
                    del self.node_constraints[old][0]
        self.node_occupancies[robot] = msg.data
        
    def make_plan(self) -> dict:
        if None in self.node_occupancies.values():
            return
        # create start and goal assignments
        
        self.env.state = self.env.start = list(self.node_occupancies.values())
        self.env.goal = list(reversed(self.env.state))
        self.planner = poro.utils.create_planner_from_config_file(self.planner_config, self.env)
        self.plan_executor = poro.polygonal_roadmap.Executor(self.env, self.planner)

        #self.get_logger().info(f'{list(sg)}')
        # make plan
        self.plan_executor.run()
        plans = self.plan_executor.get_history_as_solution()
        self.get_logger().info(f'{plans}')
        plans = {k: v for k, v in zip(self.node_occupancies.keys(), plans)}
        return plans

    def make_node_constraints(self, plans) -> dict:
        # get preconditions from plan
        node_visits = {}
        # node visits contains the order of robots visiting each node in the graph
        for robot, plan in plans.items():
            for t, n in enumerate(plan):
                if n in node_visits.keys():
                    node_visits[n].append((t, robot))
                else:
                    node_visits[n] = [(t, robot)]
        for k in node_visits.keys():
            node_visits[k] = [robot for _, robot in sorted(node_visits[k], key=lambda x: x[0])]
            node_visits[k] = [x[0] for x in groupby(node_visits[k])]
        return node_visits

    def execute_plans(self, plans: dict):

        # send message to all participating robots
        for robot, plan in plans.items():
            nv = self.node_constraints
            robot_plan = []
            for node in plan:
                # do not insert one node twice
                #if robot_plan and node == robot_plan[-1]:
                #    continue
                if nv[node] and nv[node][0] != robot:
                    break
                else:
                    robot_plan.append(node)
                    #del nv[node][0]
            # remove duplicates [1,1,1,2,1,3,3,1] -> [1,2,1,3,1]
            # https://stackoverflow.com/questions/5738901/removing-elements-that-have-consecutive-duplicates
            robot_plan = [x[0] for x in groupby(robot_plan)]
            #self.get_logger().info(f'sending plan to {robot}: {robot_plan}')
            self.send_plan_to_robot(robot_plan, robot)
        str_plan = String()
        str_plan.data = yaml.dump({'plan': plans, 'constraints': nv})
        self.plan_publisher.publish(str_plan)
        self.get_logger().info(f'plans: {plans}')
        self.get_logger().info(f'constraints: {nv}')
        
    def send_plan_to_robot(self, plan, robot):
        if plan:
            msg = String(data=yaml.dump(plan))
            self.robot_publishers[robot].publish(msg)
    

def main():
    rclpy.init()
    node = NavGraphGlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()