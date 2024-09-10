import yaml
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
from std_msgs.msg import Int32, Int32MultiArray
from driving_swarm_messages.msg import BeliefState as BeliefStateMsg
from std_msgs.msg import ColorRGBA, Int32, Int32MultiArray, String, Float32MultiArray
import networkx as nx
import numpy as np
import functools
from termcolor import colored
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose
from rclpy.time import Duration


class CCRGlobalPlannerAco(DrivingSwarmNode):
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
        nodelist = [i for i,_ in enumerate(self.g.nodes())]
        self.ccr_agent = planning.LearningAgent(
            self.g,
            self.state,
            self.goal,
            nodelist,
            #self.planning_problem_parameters,
            #limit=self.env.planning_horizon,
            **self.planner_params,
        )
        self.create_subscription(Int32, "nav/goal_node", self.goal_cb, 10)
        self.create_subscription(Int32, "nav/current_node", self.state_cb, 10)
        self.plan_pub = self.create_publisher(Int32MultiArray, "nav/plan", 10)
        self.occupancy_pub = self.create_publisher(Float32MultiArray, "nav/occupancy", 10)
        self.q_pub = self.create_publisher(Float32MultiArray, "nav/q", 10)
        self.occupancy = {}
        self.q = {}

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
                Float32MultiArray, f"/{robot}/nav/q",
                functools.partial(self.q_cb, robot),
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
        self.publish_q_values(ns=f"{self.robot_name}_q")
        self.publish_p_values(ns=f"{self.robot_name}_p")
        self.publish_episodes(ns=f"{self.robot_name}_ep")
        
    def fast_timer_cb(self):
        if self.state is None:
            return
        if self.goal is None:
            return
        self.iterate()
        self.communicate()
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
        self.get_logger().info(f"state changed: {self.state} -> {msg.data}")
        self.state = msg.data
        self.ccr_agent.update_state(self.state)
        self.update_plan()
        
    def update_plan(self):
        #self.get_logger().info(f"updating plan: {self.state} -> {self.goal}")
        if self.state is None or self.goal is None:
            return
        self.ccr_agent.iteration()

        plan = self.ccr_agent.get_plan()
        if not self.plan or self.plan != plan:
            if plan != self.plan:
                self.plan = plan
                self.publish_plan(change_only=True)

    def publish_plan(self, change_only=True):

        if change_only and self._published_plan == self.plan:
            return
        # feasibility check

        if self.plan:
            for n1, n2 in zip(self.plan[:-1], self.plan[1:]):
                if not (n1, n2) in self.g.edges():
                    self.get_logger().warn(f"plan is not feasible")
                    self.get_logger().warn(f"edge {n1} -> {n2} is not in the graph")
                    self.get_logger().warn(f"path: self.plan")
        msg = Int32MultiArray()
        msg.data = self.plan
        self.plan_pub.publish(msg)
        self._published_plan = self.plan

    def robot_cb(self, robot, msg):
        # check if plan has changed
        plan = list(msg.data)
        # nothing to see here
        # TODO: check if other robots are in the next position, cancel own plans
        
    def occupancy_cb(self, robot, msg):
        if robot == self.robot_name:
            return 
        occupancy = self.ccr_agent.get_occupancy()
        self.occupancy[robot] = np.array(list(msg.data)).reshape(occupancy.shape)
        #self.get_logger().info(f"received occupancy from {robot}: {self.occupancy[robot]}")
        #self.get_logger().info(f"occupancy: {self.occupancy}")
        
        self.ccr_agent.set_occupancy(sum(self.occupancy.values()))

    def q_cb(self, robot, msg):
        if robot == self.robot_name:
            return
        self.q[robot] = np.array(list(msg.data)).reshape(self.ccr_agent.get_q_matrix().shape)

        q_sum = sum(self.q.values())
        self.ccr_agent.set_other_q(q_sum / sum(q_sum.flatten()))
    
    def communicate(self):
        if self.goal is None:
            return
        if self.state is None:
            return
        data = self.ccr_agent.get_occupancy()
        # data is an np.array with shape (nodes, time_steps)
        # self.get_logger().info(f"occupancy: {data.shape}")
        data = Float32MultiArray(data=data.flatten().tolist())
        self.occupancy_pub.publish(data)
        
        # same for get_q_matrix:
        data = self.ccr_agent.get_q_matrix()
        data = Float32MultiArray(data=data.flatten().tolist())
        self.q_pub.publish(data)
        
    def iterate(self):
        if self.goal is None:
            return 
        if self.state is None:
            return
        episodes = self.ccr_agent.iteration()
        #self.get_logger().info(f"episodes: {episodes}")
        self.plan = self.ccr_agent.get_plan()
        #self.get_logger().info(f"q: {self.ccr_agent.get_q_matrix()}")

    def publish_q_values(self, ns="q", id=1):
        values = {}
        for u in self.env.g.nodes():
            values[u] = np.sum(self.ccr_agent.get_q_matrix()[u,:])
                
        node_msg = MarkerArray()
        for key, val in values.items():
            point = self.env.g.nodes()[key]['geometry'].center
            marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.SPHERE)
            marker.header.frame_id = 'map'
            marker.scale.x = 0.1 * val / max(values.values())
            marker.scale.y = 0.1 * val / max(values.values())
            marker.scale.z = 0.1 * val / max(values.values())
            marker.pose = Pose(position=Point(x=point.x, y=point.y, z=0.0))
            marker.color = self.get_robot_color()
            node_msg.markers.append(marker)
            id += 1 
        self.cell_pub.publish(node_msg)

    def publish_p_values(self, ns="p", id=1):
        values = {}
        for u, v in self.ccr_agent.q_function(self.state, 0).items():
            values[(self.state, u)] = v
        edge_msg = MarkerArray()
        for key, val in values.items():
            start_point = self.env.g.nodes()[key[0]]['geometry'].center
            end_point = self.env.g.nodes()[key[1]]['geometry'].center
            marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.ARROW)
            marker.header.frame_id = 'map'
            marker.scale.x = 0.05 * val / max(values.values())
            marker.scale.y = 0.07 * val / max(values.values())
            marker.points = [Point(x=(start_point.x), y=(start_point.y), z=0.0),
                 Point(x=end_point.x, y=end_point.y, z=0.0)]
            marker.lifetime = Duration(seconds=2.0).to_msg()
            marker.color = ColorRGBA(r=0.1, g=.9, b=0.1, a=1.0)
            edge_msg.markers.append(marker)
            id += 1 
        self.cell_pub.publish(edge_msg)

        return edge_msg
    
    def publish_episodes(self, ns="episodes", id=1):
        msg = MarkerArray()
        for i, episode in enumerate(self.ccr_agent.latest_episodes):
            marker = Marker(action=Marker.ADD, ns=ns, id=i+id, type=Marker.LINE_STRIP)
            marker.header.frame_id = 'map'
            marker.scale.x = 0.01
            marker.points = [Point(
                x=self.env.g.nodes()[n]['geometry'].center.x,
                y=self.env.g.nodes()[n]['geometry'].center.y, z=0.1) for n in episode]
            marker.colors = [self.get_robot_color() for _ in episode]
            msg.markers.append(marker)
            id += 1
        self.cell_pub.publish(msg)
        return msg

    def get_robot_color(self):
        if len(self.robot_names) == 1:
            return ColorRGBA(r=0.0, g=1.0, b=0.0, a=.2)
        robot_index = self.robot_names.index(self.robot_name)
        r = robot_index / (len(self.robot_names) - 1)
        b = 1.0 - r
        return ColorRGBA(r=r, g=0.0, b=b, a=.2)

def main():
    main_fn("ccr_global_planner_aco", CCRGlobalPlannerAco)

if __name__ == '__main__':
    main()