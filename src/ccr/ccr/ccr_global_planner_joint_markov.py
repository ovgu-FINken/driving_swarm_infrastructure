import yaml
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
from geometry_msgs.msg import Point, Pose
from rclpy.time import Duration
import numba as nb

#@nb.jit 
def calculate_reward_matrix(goal, distances, adjacency):
    r = np.zeros_like(distances)
    for u in range(distances.shape[0]):
        for i in range(distances.shape[1]):
            r[u,i] = distances[u, goal] - distances[i, goal]

    np.fill_diagonal(r, -1)
    r[:,goal] = 10
    return adjacency
    return r * adjacency

#@nb.jit 
def calcuate_transition_fitness(rewards, values, adjacency, offset=5, tau=1):
    f = rewards.copy() + offset
    for u in range(f.shape[0]):
        for i in range(f.shape[1]):
            f[u,i] += values[i] - values[u]
    #return adjacency
    f = (f * adjacency) ** tau
    f = f / np.sum(f)
    return adjacency
    return np.clip(f, 0.0, 10)


#@nb.jit
def simulate_pair_jit(s0, N: int, T: int, goal_self, goal_other, adjacency, distances, values_self, values_other, gamma:float=1.0, offset=5, tau=1):
    # calculate reward matrix
    log = ""
    
    r_self = calculate_reward_matrix(goal_self, distances, adjacency) 
    r_other = calculate_reward_matrix(goal_other, adjacency, distances)
    s = np.zeros((T, N, N))
    s[0] = s0
    pT_self = np.zeros((T, N, N))
    pT_other = np.zeros((T, N, N))
    normalization = np.ones((T, N, N))
            
    for t in range(T - 1):
        # tranisition t -> t+1
        # state s[t+1] will be updated by using state s[t]
        
        # calculate transition probabilities for each agent
        pT_self[t] = calcuate_transition_fitness(r_self, values_self[t], adjacency, offset=offset, tau=tau)
        pT_other[t] = calcuate_transition_fitness(r_other, values_other[t], adjacency, offset=offset, tau=tau)        
        
        # transition u,v -> i,j
        for u in range(N):
            for v in range(N):
                splus = np.zeros_like(s[t])
                psum = 0
                for i in range(N):
                    if adjacency[u, i] == 0.0:
                        continue
                    for j in range(N):
                        if adjacency[v, j] == 0.0:
                            continue
                        # same state is not allowed for two agents
                        if i == j:
                            continue
                        # swapping states is not allowed for two agents
                        if u == j:
                            continue
                        if v == i:
                            continue


                        p = pT_self[t, u, i] * pT_self[t, v, j]
                        psum += p
                        splus[i,j] += p * s[t, u,v] 
                # all states should sum to one.
                normalization[t+1, u, v] = 1.0 / psum if psum > 0 else 0.0
                splus *= normalization[t+1, u, v]
                s[t+1] += splus
        log += " " + str(np.sum(s[t+1]))

    # compute the rewards for the visited states
    DR_self = np.zeros((T, N, N))
    DR_other = np.zeros((T, N, N))
    
    # rewards after horizon: values of the last state of the episode
    # DR_self[:, :, -1] = values_self[-1, :]
    # DR_other[:, :, -1] = values_other[-1, :]
    
    for t in range(T - 2, -1, -1):
        for u in range(N):
            for v in range(N):
                for i in range(N):
                    if adjacency[u, i] == 0.0:
                        continue
                    for j in range(N):
                        if adjacency[v, j] == 0.0:
                            continue
                        if i == j:
                            continue
                        if u == j:
                            continue
                        if v == i:
                            continue
                        p = pT_self[t, u, i] * pT_other[t, v, j] * normalization[t+1, u, v]
                        DR_self[t, u, v] += p * (1 + gamma * DR_self[t+1, i, j])
                        DR_other[t, u, v] += p * (r_other[v, j] + gamma * DR_other[t+1, i, j])

    return (s, DR_self, DR_other, log)

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
        largest_cc = max(nx.connected_components(self.env.g), key=len)
        # Create subgraph and make a copy (optional, depending on use case)
        self.env.g = self.env.g.subgraph(largest_cc).copy()
        self.g = self.env.get_graph().to_directed()
        planning.compute_normalized_weight(self.g, self.planning_problem_parameters.weight_name)
        self.g.add_edges_from([(n, n) for n in self.g.nodes()], weight=self.env.planning_problem_parameters.weight_name)
        self.declare_parameter('planner_params_file', "planner_params.yaml")
        planner_params_file = self.get_parameter('planner_params_file').get_parameter_value().string_value
        with open(planner_params_file, 'r') as stream:
            self.params = yaml.safe_load(stream)

        self.get_logger().info(colored(f"planner params: {self.params}", "yellow"))
        self.nodelist = tuple(n for _, n in enumerate(self.g.nodes()))
        self.N = len(self.nodelist)
        self.T = self.params["horizon"]
        self.adjacency = np.zeros((self.N, self.N))
        for i in range(self.N):
            for j in range(self.N):
                if self.connected(i,j):
                    self.adjacency[i,j] = 1
        self.distances = np.zeros((self.N, self.N))
        for i in range(self.N):
            for j in range(self.N):
                try:
                    self.distances[i,j] = nx.shortest_path_length(self.g, self.nodelist[i], self.nodelist[j], weight=self.env.planning_problem_parameters.weight_name)
                except nx.NetworkXNoPath:
                    self.distances[i,j] = 1e20
        self.other_states = {}
        self.other_values = {}
        self.other_goals = {}
        self.other_plans = {}
        self.s = None
        self.discounted_reward_self = None
        self.plan_pub = self.create_publisher(Int32MultiArray, "nav/plan", 10)
        self.state_values = np.zeros((self.T, self.N))
        
        self.get_logger().info(f"executing simulate pair for precompile")
        simulate_pair_jit(np.zeros_like(self.adjacency), self.N, self.T, 0, 0, self.adjacency, self.distances, self.state_values, self.state_values)
        self.get_logger().info(f"executing simulate pair for precompile done")
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
            self.create_subscription(
                Int32MultiArray, f"/{robot}/nav/plan",
                functools.partial(self.plan_cb, robot),
                10
            )

        self.get_logger().info(colored("init done", "green"))
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
        #self.get_logger().info(f"making update:")
        self.update_state_values()
        self.plan = self.update_plan()
        self.publish_plan(change_only=True)
        #self.get_logger().info(f"update done")
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
            self.other_goals[robot] = self.nodelist.index(msg.data)
            return
        if self.goal is not None and msg.data == self.nodelist[self.goal]:
            return
        # reverse lookup for goal
        # msg.data is in node indicies
        # self.goal should be index in nodelist
        self.goal = self.nodelist.index(msg.data)
        self.transition_rewards = calculate_reward_matrix(self.goal, self.distances, self.adjacency)
        self.get_logger().info(f"received new goal node {msg.data}(-> {self.goal})")
        assert msg.data in self.g.nodes(), "goal must be a valid node within the graph"
        self.state_values = self.default_state_values()
        
    def plan_cb(self, robot, msg):
        if robot == self.robot_name:
            return
        if len(msg.data) == 0:
            return
        self.other_plans[robot] = [self.nodelist.index(n) for n in msg.data]
    
    def connected(self, u: int, v: int) -> bool:
        """Check if two nodes are connected in the graph.
        :param u: first node
        :type u: int
        :param v: second node
        :type v: int
        :return: True if the nodes are connected, False otherwise
        :rtype: bool
        """
        if u == v:
            return True
        return self.g.has_edge(self.nodelist[u], self.nodelist[v])

    def simulate_pair(self, other: str) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """simulate the a pair of two policies, one for this robot and one for the other robot.
        The policies are given by the state values of the robot and the other robot.
        Return value will be the probailities of the robots to occupy state s_i_j at time t, the discounted reward of all visited states for self and the discounted reward of the other robot.

        :param other: name of other robot
        :type other: str
        :return: probabilities of the robots to occupy state s_i_j at time t, the discounted reward of self and the discounted reward of the other robot
        :rtype: list[npt.NDArray[np.float64]], dict, dict
        """
        N:int = len(self.nodelist)
        T:int = self.params["horizon"]
        s0 = np.zeros((N, N))
        s0[self.state, self.other_states[other]] = 1.0
            
        s, discounted_reward_self, discounted_reward_other, log = simulate_pair_jit(s0, N, T, self.goal, self.other_goals[other], self.adjacency, self.distances, self.state_values, self.other_values[other], gamma=self.params["gamma"])
        if np.isnan(discounted_reward_self).any():
            self.get_logger().warn("nan in discounted rewards")
            self.get_logger().warn(f"s0: {s0}, s: {s}")
        self.get_logger().info(f"simulate_pair_jit:\n{log}")
            
        return s, discounted_reward_self, discounted_reward_other

    def default_state_values(self):
        values = np.ones((self.T, self.N))
        #for i in range(len(self.nodelist)):
        #    values[i,:] = 100 - self.dist(self.nodelist[i], self.goal)
        return values
            
    def update_state_values(self):
        if not len(self.other_values):
            self.get_logger().info("no other values")
            return
        rewards_self = np.zeros((self.T, self.N))
        rewards_other = np.zeros((self.T, self.N))
        states = {}
        # self.get_logger().info(f"computing state values for {self.other_values.keys()}")
        for robot in self.other_values.keys():
            if robot not in self.other_states:
                continue
            if robot == self.robot_name:
                continue
            #print(colored(f"simulating pair with {robot}", "blue"))
            s, r_s, r_o = self.simulate_pair(robot)
            self.get_logger().info(f"next states: {np.sum(s[1], axis=1)})")
            # rs is a matrix with values [t, u, v]
            # we do not care about the v-state, so we sum over axis 2
            # now we have a matrix [t, u, <summed v>]
            rewards_self += r_s.sum(axis=2)
            rewards_other += r_o.sum(axis=2)
            states[robot] = s
        
        # parameters:
        alpha = self.params["alpha"]
        beta = 1 # self.params["beta"]

        # update the state values for this robot
        # self.get_logger().info(f"updating state values for {self.robot_name}, rewards_self: {rewards_self.shape}, rewards_other: {rewards_other.shape}")
        for t in range(self.T):
            for i in range(self.N):
                self.state_values[t, i] = (1 - alpha) * self.state_values[t, i] + alpha * (beta * rewards_self[t, i] + (1 - beta) * rewards_other[t, i])
        self.state_values = rewards_self
        # self.state_values = np.clip(self.state_values, 0.0, 1000.0) # all states are valuable

    def get_neigbours_by_index(self, u:int):
        n = self.nodelist[u]
        neighbours = self.g.neighbors(n)
        return [self.nodelist.index(i) for i in neighbours]

    def update_plan(self) -> list[int]|None:
        if self.state is None:
            return
        if self.goal is None:
            return
        if self.state == self.goal:
            return
        #self.get_logger().info(f"updating plan for {self.robot_name} from {self.state} to {self.goal}")
        s = self.state
        plan = [s]
        blocked_states = set(self.other_states.values())
        for robot, plan_other in self.other_plans.items():
            if robot == self.robot_name:
                continue
            if len(plan_other) < 2:
                continue
            # dont use the first two states of the other robot, because they are not yet in the future
            blocked_states |= set(plan_other[:2])

        for t in range(self.params["horizon"]-1):
            fitness = calcuate_transition_fitness(self.transition_rewards, self.state_values[t], self.adjacency, offset=self.params['offset'], tau=self.params['tau'])
            if t<2:
                for b in blocked_states:
                    fitness[b] = 0.0

            if len(fitness) == 0:
                self.get_logger().warn(f"no neighbors found for {s} at time {t}")
                return plan
            s = np.argmax(fitness)
            plan.append(s)
        return plan

    def plan_is_feasible(self) -> bool:
        if self.plan:
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
            msg.data = [ self.nodelist[self.state] ]
            self.plan_pub.publish(msg)
            self._published_plan = [self.state]
            return 

        if change_only and self._published_plan[:4] == self.plan[:4]: # type: ignore
            return

        self._published_plan = self.plan
        msg = Int32MultiArray()
        msg.data = [self.nodelist[n] for n in self.plan]
        self.plan_pub.publish(msg)
        
    def state_cb(self, robot, msg):
        # make sure to translate node label to index in nodelist
        if robot == self.robot_name:
            if self.nodelist.index(msg.data) == self.state:
                return
            self.get_logger().info(f"state changed: {self.state} -> {msg.data}, goal: {self.goal}")
            self.state = self.nodelist.index(msg.data)
            # we enter a new state, i.e., t=t+1
            #self.state_values[:,0:-1] = self.state_values[:,1:] # shift values to the left
            #self.state_values[:,-1] = self.default_state_values()[:,-1]
            #self.state_values = 0.5* self.state_values + 0.5* self.default_state_values()
            return

        if robot in self.other_states:
            if self.other_states[robot] == self.nodelist.index(msg.data):
                return
            #self.get_logger().info(f"other state changed: {robot} {self.other_states[robot]} -> {msg.data}")
        self.other_states[robot] = self.nodelist.index(msg.data)

    def publish_visualization_markers(self, ns="state_values", id=1):
        node_msg = MarkerArray()
        vis_values = self.state_values# [:,1:] # skip the first column, which is the current state value
        #if self.discounted_reward_self is None:
        #    return
        #vis_values = np.sum(self.discounted_reward_self, axis=1)
        #if self.s is None:
        #    return
        #vis_values = np.sum(self.s, axis=1) # sum over all possible states of the other robot, to get single robot occupancy probabilities

        for node, val in zip(self.nodelist, vis_values.transpose()):
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
            for val_i in val[:4]:
                marker.text += str(f"{val_i:.2f},")
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
