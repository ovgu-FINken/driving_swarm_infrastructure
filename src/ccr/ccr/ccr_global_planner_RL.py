import yaml
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
from std_msgs.msg import Int32, Int32MultiArray
from std_msgs.msg import ColorRGBA, Int32, Int32MultiArray, String, Float32MultiArray
import networkx as nx
import numpy as np
import functools
from termcolor import colored
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose
from rclpy.time import Duration


class CCRGlobalPlannerRL(DrivingSwarmNode):
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
            self.planner_params = yaml.safe_load(stream)

        self.get_logger().info(f"planner params: {self.planner_params}")
        self.nodelist = tuple( i for i,_ in enumerate(self.g.nodes()) )
        self.create_subscription(Int32, "nav/goal_node", self.goal_cb, 10)
        self.create_subscription(Int32, "nav/current_node", self.state_cb, 10)
        self.plan_pub = self.create_publisher(Int32MultiArray, "nav/plan", 10)
        self.occupancy_pub = self.create_publisher(Float32MultiArray, "nav/occupancy", 10)
        #self.v_pub = self.create_publisher(Float32MultiArray, "nav/v", 10)
        self.occupancy = {}
        self.v = {n: float('inf') for n in self.nodelist}
        self.other_states = {}

        for robot in [n for n in self.robot_names if n != self.robot_name]:
            self.create_subscription(
                Int32MultiArray, f"/{robot}/nav/plan",
                functools.partial(self.robot_cb, robot),
                10
            )
            self.create_subscription(
                Float32MultiArray, f"/{robot}/nav/occupancy",
                functools.partial(self.occupancy_cb, robot),
                10
            )
            self.create_subscription(
                Float32MultiArray, f"/{robot}/nav/v",
                functools.partial(self.v_cb, robot),
                10
            )
            
            self.get_logger().info(f"subscribing /{robot}/nav/plan")
            
        
        self.create_timer(1.0, self.timer_cb)
        self.create_timer(0.1, self.fast_timer_cb)
        
    def timer_cb(self):
        """Publish visualization markers.
        """
        if self.state is None:
            return
        if self.goal is None:
            return
        pass
        
    def fast_timer_cb(self):
        if self.state is None:
            return
        if self.goal is None:
            return
        self.update_plan()
        self.publish_plan(change_only=True)

    def goal_cb(self, msg):
        if msg.data == self.goal:
            return
        self.goal = msg.data
        self.get_logger().info(f"received new goal node {self.goal}")
        self.update_state_values()
    
    def update_state_values(self):
        # for each state in the graph find the shortest distance to the goal
        # the distance is the expected cost
        self.get_logger().info(f"updating state values")
        for node in self.nodelist:
            try:
                self.v[node] = nx.shortest_path_length(self.g, node, self.goal, weight=self.env.planning_problem_parameters.weight_name)
            except nx.NetworkXNoPath:
                self.v[node] = float('inf')
        self.get_logger().info(f"state values done for {len(self.nodelist)} nodes")
            

    def state_cb(self, msg):
        if msg.data == self.state:
            return
        self.get_logger().info(f"state changed: {self.state} -> {msg.data}")
        self.state = msg.data
        pass
        
    def update_plan(self) -> list[int]:
        if self.state is None:
            return
        if self.goal is None:
            return
        if self.state == self.goal:
            return
        neighbours = list(self.g.neighbors(self.state))
        best = min(neighbours, key=lambda x: self.v[x])
        self.plan = [self.state, best]
        return self.plan 

    def plan_is_feasible(self) -> bool:
        if not self.plan:
            return False
        for n1, n2 in zip(self.plan[:-1], self.plan[1:]):
            if not (n1, n2) in self.g.edges():
                self.get_logger().warn(f"plan is not feasible")
                self.get_logger().warn(f"edge {n1} -> {n2} is not in the graph")
                self.get_logger().warn(f"path: self.plan")
                return False
        
        
        return True

    def publish_plan(self, change_only=True):

        if change_only and self._published_plan == self.plan:
            return
        
        if not self.plan_is_feasible():
            return

        # dont go where another robot is at the moment.
        if len(self.plan) > 1:
            if self.plan[1] in self.other_states.values():
                self.plan = [self.state]

        msg = Int32MultiArray()
        msg.data = self.plan
        self.plan_pub.publish(msg)
        self._published_plan = self.plan

    def robot_cb(self, robot, msg):
        if robot == self.robot_name:
            return 
        plan = list(msg.data)
        self.other_states[robot] = plan[0]
        
    def occupancy_cb(self, robot, msg):
        if robot == self.robot_name:
            return 
        pass

    def publish_state_values(self, ns="v", id=1):
        values = {}
        for u in self.env.g.nodes():
            values[u] = np.sum(self.ccr_agent.get_q_matrix()[u,:])
                
        node_msg = MarkerArray()
        for key, val in values.items():
            point = self.env.g.nodes()[key]['geometry'].center
            marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.SPHERE)
            marker.header.frame_id = 'map'
            scale = (val + 0.0001) / (max(values.values())+0.0001)
            scale = 0.1 * scale # np.log(scale+1.0)
            if scale < 0:
                scale = 0.0
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.pose = Pose(position=Point(x=point.x, y=point.y, z=0.0))
            marker.color = self.get_robot_color()
            node_msg.markers.append(marker)
            id += 1 
        self.cell_pub.publish(node_msg)

    def get_robot_color(self):
        if len(self.robot_names) == 1:
            return ColorRGBA(r=0.0, g=1.0, b=0.0, a=.2)
        robot_index = self.robot_names.index(self.robot_name)
        r = robot_index / (len(self.robot_names) - 1)
        b = 1.0 - r
        return ColorRGBA(r=r, g=0.0, b=b, a=.2)

def main():
    main_fn("ccr_global_planner_RL", CCRGlobalPlannerRL)

if __name__ == '__main__':
    main()