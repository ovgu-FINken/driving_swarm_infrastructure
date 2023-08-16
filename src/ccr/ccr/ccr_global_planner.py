from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
from std_msgs.msg import Int32, Int32MultiArray
from driving_swarm_messages.msg import BeliefState
import networkx as nx
import numpy as np
import functools
from termcolor import colored

class CCRGlobalPlanner(DrivingSwarmNode):
    """This node will execute the local planner for the CCR. It will use the map or a given graph file to generate a roadmap and convert local coordinates to graph nodes.
    The local planner will publish the next waypoint, current node and the graph to the global planner, which in turn is able to generate a discrete plan by uing CCR.
    This node (the local planner) is then able to compute waypoints using a vehicle model in the feasible region of the workspace and generate waypoints for execution.    
    """

    def __init__(self, name):
        super().__init__(name)

        # robot names needed to subscribe to other plans
        self.declare_parameter('robot_names', ['invalid_name'])
        self.other_robots = self.get_parameter('robot_names').get_parameter_value().string_array_value
        self.other_robots.remove(self.robot_name)
        self.other_robots_plans = {robot: [] for robot in self.other_robots}

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
        self.state = None
        self.goal = None
        self.update_path = False
         
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
                                                        offset=0.18)
        self.planning_problem_parameters = environment.PlanningProblemParameters()
        self.cache = planning.SpaceTimeAStarCache(self.env.g)
        self.g = self.env.get_graph().to_directed()
        planning.compute_normalized_weight(self.g, self.planning_problem_parameters.weight_name)
        self.weight = "weight"
        astar_kwargs = {
            'limit': self.env.planning_horizon,
            'wait_action_cost': self.env.planning_problem_parameters.wait_action_cost,
        }
        self.cache = planning.SpaceTimeAStarCache(self.g, kwargs=astar_kwargs)
        self.ccr_agent = planning.CCRAgent(self.g, self.state, self.goal, self.planning_problem_parameters, index=self.robot_name, sta_star_cache=self.cache)
        self.create_subscription(Int32, "nav/goal_node", self.goal_cb,1)
        self.create_subscription(Int32, "nav/current_node", self.state_cb, 1)
        self.plan_pub = self.create_publisher(Int32MultiArray, "nav/plan", 1)

        # trigger CDM for specific node
        self.cdm_pub = self.create_publisher(Int32, "/nav/cdm", 1)
        ## -- once a message is received here, the robot will publish its opinion on the given node
        self.create_subscription(Int32, "/nav/cdm", self.cdm_cb, 1)
        self.opinion_pub = self.create_publisher(BeliefState, "/nav/opinion", 1)
        self.create_subscription(BeliefState, "/nav/opinion", self.opinion_cb, 1)
        
        
        for robot in self.other_robots:
            self.create_subscription(
                Int32MultiArray, f"/{robot}/nav/plan",
                functools.partial(self.robot_cb, robot),
                10
            )
            self.get_logger().info(f"subscribing /{robot}/nav/plan")
            
        for robot in self.other_robots:
            self.create_subscription(
                Int32MultiArray, f"/{robot}/nav/cdm",
                functools.partial(self.robot_cb, robot),
                10
            )
            self.get_logger().info(f"subscribing /{robot}/nav/plan")
        
        self.create_timer(1.0, self.timer_cb)
        
    def timer_cb(self):
        if self.state is None:
            return
        if self.goal is None:
            return
        self.publish_plan()
        
    def goal_cb(self, msg):
        if msg.data == self.goal:
            return
        self.goal = msg.data
        self.get_logger().info(f"received goal {self.goal}")
        self.ccr_agent.update_goal(self.goal)
        self.update_plan()

    def state_cb(self, msg):
        if msg.data == self.state:
            return
        self.state = msg.data
        self.get_logger().info(f"received state {self.state}")
        self.ccr_agent.update_state(self.state)
        self.update_plan()
        
    def trigger_cdm(self):
        options = self.ccr_agent.get_cdm_node()
        if len(options) == 0:
            self.get_logger().warn("no CDM options for {self.robot_name}")
            return
        node = np.random.choice(list(options))
        self.cdm_pub.publish(Int32(data=node))
        self.get_logger().info(colored("triggering CDM", "yellow") + f" at node {node}")

    def update_plan(self):
        self.get_logger().info(f"updating plan: {self.state} -> {self.goal}")
        if self.state is None or self.goal is None:
            return
        self.ccr_agent.make_plan_consistent()
            
        # when it is not possible to make plan consistent, trigger CDM
        if self.ccr_agent.get_conflicts():
            self.get_logger().info(f"plan is consistent and not conflict free, triggering CDM")
            self.trigger_cdm()
            
        self.plan = self.ccr_agent.get_plan()
        self.publish_plan()
        self.get_logger().info(f"plan: {self.plan}")
    
    def publish_plan(self):
        msg = Int32MultiArray()
        msg.data = self.plan
        self.plan_pub.publish(msg)

    def robot_cb(self, robot, msg):
        if self.other_robots_plans[robot] == list(msg.data):
            return
        self.other_robots_plans[robot] = list(msg.data)
        self.ccr_agent.update_other_paths({robot: self.other_robots_plans[robot]})
        self.update_plan()
    
    def cdm_cb(self, msg):
        opinion = self.ccr_agent.get_cdm_opinion(msg.data)
        self.opinion_pub.publish(Int32(data=opinion))
        
    def opinion_cb(self, msg):
        bs = BeliefState(state=msg.state, priorities=msg.priorities)
        if bs.state in self.ccr_agent.belief:
            bs += self.ccr_agent.belief[bs.state]
        self.ccr_agent.set_belief(bs)
        self.update_plan()
            

def main():
    main_fn("ccr_global_planner", CCRGlobalPlanner)

if __name__ == '__main__':
    main()
