from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
from std_msgs.msg import Int32, Int32MultiArray
from driving_swarm_messages.msg import BeliefState as BeliefStateMsg
from std_msgs.msg import ColorRGBA, Int32, Int32MultiArray, String
import networkx as nx
import numpy as np
import functools
from termcolor import colored
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


class CCRGlobalPlanner(DrivingSwarmNode):
    """This node will execute the local planner for the CCR. It will use the map or a given graph file to generate a roadmap and convert local coordinates to graph nodes.
    The local planner will publish the next waypoint, current node and the graph to the global planner, which in turn is able to generate a discrete plan by uing CCR.
    This node (the local planner) is then able to compute waypoints using a vehicle model in the feasible region of the workspace and generate waypoints for execution.    
    """

    def __init__(self, name):
        super().__init__(name)

        # robot names needed to subscribe to other plans
        self.declare_parameter('robot_names', ['invalid_name'])
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value

        self.declare_parameter('graph_file', 'graph.yaml')
        self.declare_parameter('x_min', -2.0)
        self.declare_parameter('x_max', 3.0)
        self.declare_parameter('y_min', -2.0)
        self.declare_parameter('y_max', 1.0)
        map_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.get_logger().info(f'loading graph from {map_file}')
        
        wx = (self.get_parameter('x_min').get_parameter_value().double_value, self.get_parameter('x_max').get_parameter_value().double_value)
        wy = (self.get_parameter('y_min').get_parameter_value().double_value, self.get_parameter('y_max').get_parameter_value().double_value)
        self.declare_parameter('grid_type', 'square')
        self.declare_parameter('grid_size', .5)
        self.declare_parameter('inflation_size', 0.2)
        self.declare_parameter('horizon', 5)
        self.declare_parameter('inertia', 0.1)
        self.declare_parameter('wait_cost', 1.01)
        self._published_plan = []
        self.state = None
        self.plan = []
        self.goal = None
        self.update_path = False
        self.poly_pub2 = self.create_publisher(MarkerArray, '/cells', 10)
        points = None 
        if map_file.endswith(".yaml"):
            grid_size = self.get_parameter('grid_size').get_parameter_value().double_value
            tiling = self.get_parameter('grid_type').get_parameter_value().string_value
            if tiling == 'hex':
                points = geometry.hexagon_tiling(grid_size, working_area_x=wx, working_area_y=wy)
            elif tiling == 'square':
                points = geometry.square_tiling(grid_size, working_area_x=wx, working_area_y=wy)
            elif tiling == 'random':
                points = geometry.random_tiling(50, working_area_x=wx, working_area_y=wy)
            else:
                self.get_logger().warn('no tiling specified, using hex')
                points = geometry.hexagon_tiling(grid_size, working_area_x=wx, working_area_y=wy)
        assert points is not None
        self.env = environment.RoadmapEnvironment(map_file,
                                                        None,
                                                        None,
                                                        generator_points=points,
                                                        wx=wx,
                                                        wy=wy,
                                                        offset=self.get_parameter('inflation_size').get_parameter_value().double_value)
        self.planning_problem_parameters = environment.PlanningProblemParameters(
            pad_path=False,
            conflict_horizon=self.get_parameter('horizon').get_parameter_value().integer_value,
            wait_action_cost=self.get_parameter('wait_cost').get_parameter_value().double_value,
        )
        self.g = self.env.get_graph().to_directed()
        planning.compute_normalized_weight(self.g, self.planning_problem_parameters.weight_name)
        self.g.add_edges_from([(n, n) for n in self.g.nodes()], weight=self.env.planning_problem_parameters.weight_name)
        self.declare_parameter('priorities', "same")
        priorities_method = self.get_parameter('priorities').get_parameter_value().string_value
        if priorities_method == "same":
            self.priorities = {i: 0 for i, _ in enumerate(self.robot_names)}
        elif priorities_method == "index":
            self.priorities = {i: i for i, _ in enumerate(self.robot_names)}
        else:
            self.priorities = {i: i for i, _ in enumerate(self.robot_names)}
            
        self.ccr_agent = planning.PriorityAgent(
            self.g,
            self.state,
            self.goal,
            self.planning_problem_parameters,
            index=self.robot_names.index(self.robot_name),
            limit=self.env.planning_horizon,
            inertia=self.get_parameter('inertia').get_parameter_value().double_value,
        )
        self.create_subscription(Int32, "nav/goal_node", self.goal_cb, 10)
        self.create_subscription(Int32, "nav/current_node", self.state_cb, 10)
        self.plan_pub = self.create_publisher(Int32MultiArray, "nav/plan", 10)

        for robot in [n for n in self.robot_names if n != self.robot_name]:
            self.create_subscription(
                Int32MultiArray, f"/{robot}/nav/plan",
                functools.partial(self.robot_cb, robot),
                10
            )
            self.get_logger().info(f"subscribing /{robot}/nav/plan")
            
        
        self.create_timer(2.0, self.timer_cb)
        self.create_timer(0.3, self.fast_timer_cb)
        
    def timer_cb(self):
        if self.state is None:
            return
        if self.goal is None:
            return
        
       # publish things 
        self.publish_plan(change_only=False)
        
    def fast_timer_cb(self):
        self.publish_plan(change_only=True)

    def goal_cb(self, msg):
        if msg.data == self.goal:
            return
        self.goal = msg.data
        self.get_logger().info(f"received goal node {self.goal}")
        self.ccr_agent.update_goal(self.goal)
        self.update_plan()

    def state_cb(self, msg):
        if msg.data == self.state:
            return
        old_state = self.state
        self.state = msg.data
        # self.get_logger().info(f"received state {self.state}")
        self.ccr_agent.update_state(self.state)
        self.update_plan()
        
    def update_plan(self):
        #self.get_logger().info(f"updating plan: {self.state} -> {self.goal}")
        if self.state is None or self.goal is None:
            return
        self.ccr_agent.make_plan_consistent()

        if not self.plan or self.plan != self.ccr_agent.get_plan():
            plan = self.ccr_agent.get_plan()
            if plan != self.plan:
                self.plan = plan
                #self.publish_plan(change_only=True)
                #self.get_logger().info(f"new plan: {self.plan}")

    def publish_plan(self, change_only=True):
        # feasibility check
        if self.plan:
            for n1, n2 in zip(self.plan[:-1], self.plan[1:]):
                if not (n1, n2) in self.g.edges():
                    self.get_logger().warn(f"plan is not feasible")
                    self.get_logger().warn(f"edge {n1} -> {n2} is not in the graph")
                    self.get_logger().warn(f"path: self.plan")
        msg = Int32MultiArray()
        msg.data = self.plan
        if change_only and self._published_plan == self.plan:
            return
        self.plan_pub.publish(msg)
        self._published_plan = self.plan

    def robot_cb(self, robot, msg):
        # check if plan has changed
        plan = list(msg.data)
        other_index = self.robot_names.index(robot)
        if other_index in self.ccr_agent.other_paths:
            if plan == self.ccr_agent.other_paths[other_index]:
                return
        #self.get_logger().info(f"received plan from {robot}: {plan}")
        self.ccr_agent.update_other_paths({self.robot_names.index(robot): list(msg.data)})
        self.update_plan()


def main():
    main_fn("ccr_global_planner_baseline", CCRGlobalPlanner)

if __name__ == '__main__':
    main()