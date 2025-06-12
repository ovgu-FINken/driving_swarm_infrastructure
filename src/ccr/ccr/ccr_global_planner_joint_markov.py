import yaml
from ccr.utils import random_decision, greedy_decision, probabilities_from_values
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
from std_msgs.msg import Int32, Int32MultiArray
from std_msgs.msg import ColorRGBA, Int32, Int32MultiArray, String, Float32MultiArray
import networkx as nx
import numpy as np
import numpy.typing as npt
import functools
from termcolor import colored
from visualization_msgs.msg import MarkerArray, Marker
from itertools import product
from geometry_msgs.msg import Point, Pose
from rclpy.time import Duration


def get_state_value(node: int, t: int, values: npt.NDArray[np.float64]) -> float:
    """Get the state value for a given node and time step.
    :param node: node index
    :type node: int
    :param t: time step
    :type t: int
    :return: state value
    :rtype: float
    """
    horizon = values.shape[1]
    if t < 0:
        return 0.0
    if t >= horizon:
        return values[node, -1]
    return values[node, t]


class CCRGlobalPlannerMarkov(DrivingSwarmNode):
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
        self.declare_parameter('wait_cost', 1.01)
        self._published_plan = []
        self.state = None
        self.plan = []
        self.goal = None
        self.update_path = False
        self.cell_pub = self.create_publisher(MarkerArray, '/cells', 10)
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
        self.declare_parameter('planner_params_file', "planner_params.yaml")
        planner_params_file = self.get_parameter('planner_params_file').get_parameter_value().string_value
        with open(planner_params_file, 'r') as stream:
            self.params = yaml.safe_load(stream)

        self.get_logger().info(f"planner params: {self.params}")
        self.nodelist = tuple(i for i, _ in enumerate(self.g.nodes()))
        self.other_states = {}
        self.other_values = {}
        self.other_goals = {}
        self.distances = {}
        self.plan_pub = self.create_publisher(Int32MultiArray, "nav/plan", 10)
        self.node_distances = np.array([float('inf')] * len(self.nodelist))
        self.state_values = np.zeros( (len(self.nodelist), self.params["horizon"]) )
        self.value_pub = self.create_publisher(Float32MultiArray, "nav/state_values", 10)

        for robot in self.robot_names:
            self.create_subscription(
                Int32, f"/{robot}/nav/goal_node",
                functools.partial(self.goal_cb, robot),
                10
            )
            self.create_subscription(
                Int32, f"/{robot}/nav/current_node",
                functools.partial(self.state_cb, robot),
                10
            )
            if robot == self.robot_name:
                continue
            self.create_subscription(
                Float32MultiArray, f"/{robot}/nav/state_values",
                functools.partial(self.value_cb, robot),
                10
            )

        self.create_timer(1.0, self.timer_cb)
        self.create_timer(0.1, self.fast_timer_cb)
        
    def timer_cb(self):
        """Publish visualization markers.
        """
        if self.state is None:
            return
        if self.goal is None:
            return
        self.publish_visualization_markers(ns=f"{self.robot_name}_o")

    def fast_timer_cb(self):
        if self.state is None:
            return
        if self.goal is None:
            return
        self.get_logger().info(f"making update:")
        self.update_state_values()
        self.get_logger().info(f"update done")
        self.plan = self.update_plan()
        self.publish_plan(change_only=True)
        #self.update_occupancy()
        self.get_logger().info(f"publishing state values for {self.robot_name}")
        self.value_pub.publish(Float32MultiArray(data=self.state_values.flatten().tolist()))                            

    def value_cb(self, robot, msg):
        if robot == self.robot_name:
            return 
        arr = np.array(msg.data).reshape(self.state_values.shape)
        self.other_values[robot] = arr
        self.get_logger().info(f"received state values from {robot}")


    def goal_cb(self, robot, msg):
        if robot != self.robot_name:
            self.other_goals[robot] = msg.data
            return
        if msg.data == self.goal:
            return
        self.goal = msg.data
        self.get_logger().info(f"received new goal node {self.goal}")
        self.state_values = self.default_state_values()
        
    def dist(self, start, goal):
        """Compute the distance between two nodes in the graph.
        :param start: start node
        :type start: int
        :param goal: goal node
        :type goal: int
        :return: distance between start and goal node
        :rtype: float
        """
        if (start, goal) not in self.distances:
            try:
                dist = nx.shortest_path_length(self.g, start, goal, weight=self.env.planning_problem_parameters.weight_name)
            except nx.NetworkXNoPath:
                dist = float('inf')
            self.distances[start, goal] = dist
        return self.distances[start, goal]
        

    def transition_reward(self, u: int, i: int, goal: int|None) -> float:
        if goal is None:
            return 0.0
        if not (u, i) in self.g.edges():
            return 0.0
        if i == self.goal:
            return 2.0
        return self.dist(u, goal) - self.dist(i, goal)

    def transition_fitness(self, u: int, i: int, t: int, values: npt.NDArray[np.float64]) -> float:
        """ reward for the transition from from_node at time t to to_node at time t+1 """
        gain = values[i, t+1] - values[u, t]
        return (gain + self.params["offset"]) ** self.params["tau"]
        
    def simluate_pair(self, other: str) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """simulate the a pair of two policies, one for this robot and one for the other robot.
        The policies are given by the state values of the robot and the other robot.
        Return value will be the probailities of the robots to occupy state s_i_j at time t, the discounted reward of all visited states for self and the discounted reward of the other robot.

        :param other: name of other robot
        :type other: str
        :return: probabilities of the robots to occupy state s_i_j at time t, the discounted reward of self and the discounted reward of the other robot
        :rtype: list[npt.NDArray[np.float64]], dict, dict
        """
        s = np.zeros((len(self.nodelist), len(self.nodelist), self.params["horizon"]))
        s[self.state, self.other_states[other], 0] = 1.0
        transition_probabilities_self = {}
        transition_probabilities_other = {}
        rlnl = range(len(self.nodelist))
        for t in range(self.params["horizon"] - 1):
            transition_probabilities_self[t] = np.zeros((len(self.nodelist), len(self.nodelist)))
            transition_probabilities_other[t] = np.zeros((len(self.nodelist), len(self.nodelist)))
            # transition (u,v)(t) -> (i,j)(t+1)
            transition_fitness_self = np.zeros((len(self.nodelist), len(self.nodelist)))
            transition_fitness_other = np.zeros((len(self.nodelist), len(self.nodelist)))

            for u, v, i, j in product(rlnl, rlnl, rlnl, rlnl):
                # if no edge -> no transition
                if (u, i) not in self.g.edges():
                    continue
                if (v, j) not in self.g.edges():
                    continue
                # if collision is possible, skip this transition
                # node conflict:
                if i == j:
                    continue
                # edge conflict:
                if v == i:
                    continue
                if u == j:
                    continue
                # compute transition fitness (decision function)
                transition_fitness_self[u, i] = self.transition_fitness(u, i, t, self.state_values)
                transition_fitness_other[v, j] = self.transition_fitness(v, j, t, self.other_values[other])
            
            # compute the probabilities for the transitions
            # we assume fitness proportional selection (roulette wheel selection)
            for u in rlnl:
                p_sum = np.sum(transition_fitness_self[u, :])
                if p_sum > 0:
                    transition_probabilities_self[t][u, :] = transition_fitness_self[u, :] / p_sum
                p_sum = np.sum(transition_fitness_other[u, :])
                if p_sum > 0:
                    transition_probabilities_other[t][u, :] = transition_fitness_other[u, :] / p_sum

            # compute the next state
            for u, v, i, j in product(rlnl, rlnl, rlnl, rlnl):
                s[i, j, t+1] += s[u, v, t] * transition_probabilities_self[t][u, i] * transition_probabilities_other[t][v, j]
            
        # compute the rewards for the visited states
        rewards_self = np.zeros((len(self.nodelist), len(self.nodelist), self.params["horizon"]))
        rewards_other = np.zeros((len(self.nodelist), len(self.nodelist), self.params["horizon"]))
        rewards_self[:, :, -1] = self.state_values[:, -1]
        rewards_other[:, :, -1] = self.other_values[other][:, -1]
        
        
        # reward at the last time step is based on the state-distance to the goal
        for t in range(self.params["horizon"] - 2, -1, -1):
            for u, v, i, j in product(rlnl, rlnl, rlnl, rlnl):
                p = transition_probabilities_self[t][u, i] * transition_probabilities_other[t][v, j]
                # compute the reward for the transition
                r_t = self.transition_reward(u, i, self.goal)
                r_next_state = rewards_self[i, j, t + 1]
                rewards_self[u, v, t] = p * (r_t + self.params['gamma'] * r_next_state)
                
                r_t = self.transition_reward(v, j, self.other_goals[other]) if other in self.other_goals else 0.0
                r_next_state = rewards_other[i, j, t + 1]
                rewards_other[u, v, t] = p * (r_t + self.params['gamma'] * r_next_state)

        return (s, rewards_self, rewards_other)
        
        
    def default_state_values(self):
        values = np.zeros((len(self.nodelist), self.params["horizon"]))
        for i in range(len(self.nodelist)):
            values[i,:] = 100 - self.dist(self.nodelist[i], self.goal)
        return values
            
    def update_state_values(self):
        rewards_self = np.zeros((len(self.nodelist), self.params["horizon"]))
        rewards_other = np.zeros((len(self.nodelist), self.params["horizon"]))
        self.get_logger().info(f"computing state values for {self.other_values.keys()}")
        if not len(self.other_values):
            self.get_logger().info("no other values")
            return
        for robot in self.other_values.keys():
            if robot == self.robot_name:
                continue
            #print(colored(f"simulating pair with {robot}", "blue"))
            s, r_s, r_o = self.simluate_pair(robot)
            rewards_self += np.sum(r_s, axis=1)
            rewards_other += np.sum(r_o, axis=1)
        
        # parameters:
        alpha = self.params["alpha"]
        beta = self.params["beta"]

        # update the state values for this robot
        self.get_logger().info(f"updating state values for {self.robot_name}, rewards_self: {rewards_self.shape}, rewards_other: {rewards_other.shape}")
        for t in range(self.params["horizon"]):
            for i in range(len(self.nodelist)-1):
                self.state_values[i, t] = (1 - alpha) * self.state_values[i, t] + alpha * (beta * rewards_self[i, t] + (1 - beta) * rewards_other[i, t])

    def update_plan(self) -> list[int]|None:
        if self.state is None:
            return
        if self.goal is None:
            return
        if self.state == self.goal:
            return
        self.get_logger().info(f"updating plan for {self.robot_name} from {self.state} to {self.goal}")
        s = self.state
        plan = [s]
        for t in range(self.params["horizon"]-1):
            neighbors = list(self.g.neighbors(s))
            fitness = {}
            for n in neighbors:
                if t < 2 and n in self.other_states.values():
                    continue
                if n == self.goal:
                    plan.append(n)
                    return plan
                fitness[n] = self.transition_fitness(s, n, t, self.state_values)
            if len(fitness) == 0:
                self.get_logger().warn(f"no neighbors found for {s} at time {t}")
                return plan
            s = max(fitness, key=fitness.get)
            plan.append(s)
        return plan

    def plan_is_feasible(self) -> bool:
        if not self.plan:
            return False
        for n1, n2 in zip(self.plan[:-1], self.plan[1:]):
            if not (n1, n2) in self.g.edges():
                self.get_logger().warn(f"plan is not feasible")
                self.get_logger().warn(f"edge {n1} -> {n2} is not in the graph")
                self.get_logger().warn(f"path: {self.plan}")
                return False

        # dont go where another robot is at the moment.
        if len(self.plan) > 1:
            if self.plan[1] in self.other_states.values():
                self.plan = [self.state]
        return True

    def publish_plan(self, change_only=True):
        if not self.plan_is_feasible():
            msg = Int32MultiArray()
            msg.data = [self.state]
            self.plan_pub.publish(msg)
            self._published_plan = [self.state]
            return 

        if change_only and self._published_plan[:4] == self.plan[:4]: # type: ignore
            return

        msg = Int32MultiArray()
        msg.data = self.plan
        self.plan_pub.publish(msg)
        self._published_plan = self.plan
        
    def state_cb(self, robot, msg):
        #self.get_logger().info(f"state_cb for robot:{robot} with state:{msg.data}")
        if robot == self.robot_name:
            if msg.data == self.state:
                return
            self.get_logger().info(f"state changed: {self.state} -> {msg.data}, goal: {self.goal}")
            self.state = msg.data
            return
        if robot in self.other_states:
            if self.other_states[robot] == msg.data:
                return
            #self.get_logger().info(f"other state changed: {robot} {self.other_states[robot]} -> {msg.data}")
        self.other_states[robot] = msg.data

    def publish_visualization_markers(self, ns="state_values", id=1):
        node_msg = MarkerArray()
        for node, val in zip(self.nodelist, self.state_values[:,1:]):
            # scale = 0.2 * sum(val)
            # if scale <= 0:
            #     continue # dont add markers for empty cells
            # marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.SPHERE)
            point = self.env.g.nodes()[node]['geometry'].center
            # marker.header.frame_id = 'map'
            # marker.scale.x = scale
            # marker.scale.y = scale
            # marker.scale.z = scale
            # marker.pose = Pose(position=Point(x=point.x, y=point.y, z=0.0))
            # marker.color = self.get_robot_color()
            # node_msg.markers.append(marker)
            # id += 1 
            marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.TEXT_VIEW_FACING)
            marker.header.frame_id = 'map'
            marker.pose.position = Point(x=point.x + self.robot_names.index(self.robot_name)*0.1+0.1, y=point.y, z=0.0) # type: ignore
            #marker.text = str(f"{val[0]*100:.0f},{val[1]*100:.0f},{val[2]*100:.0f},{val[3]*100:.0f}")
            marker.text = str(f"{val[0]:.2f},{val[1]:.2f},{val[2]:.2f},{val[3]:.2f}")
            marker.scale.z = 0.1
            marker.color = self.get_robot_color()
            #marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=.9)
            node_msg.markers.append(marker) #type: ignore
            id += 1
        self.cell_pub.publish(node_msg)
        
    def get_robot_color(self):
        if len(self.robot_names) == 1:
            return ColorRGBA(r=0.0, g=1.0, b=0.0, a=.5)
        robot_index:int = self.robot_names.index(self.robot_name) # type: ignore
        r = robot_index / (len(self.robot_names) - 1)
        b = 1.0 - r
        return ColorRGBA(r=r, g=0.0, b=b, a=.5)
    
def main():
    main_fn("ccr_global_planner_markov", CCRGlobalPlannerMarkov)

if __name__ == '__main__':
    main()
