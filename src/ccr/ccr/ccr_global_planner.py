from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment
from std_msgs.msg import Int32, Int32MultiArray
import networkx as nx

class CCRGlobalPlanner(DrivingSwarmNode):
    """This node will execute the local planner for the CCR. It will use the map or a given graph file to generate a roadmap and convert local coordinates to graph nodes.
    The local planner will publish the next waypoint, current node and the graph to the global planner, which in turn is able to generate a discrete plan by uing CCR.
    This node (the local planner) is then able to compute waypoints using a vehicle model in the feasible region of the workspace and generate waypoints for execution.    
    """

    def __init__(self, name):
        super().__init__(name)

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
        self.create_subscription(Int32, "nav/goal_node", self.goal_cb,1)
        self.create_subscription(Int32, "nav/current_node", self.state_cb, 1)
        self.plan_pub = self.create_publisher(Int32MultiArray, "nav/plan", 1)
        
        #TODO subscribe to other plans
        

        self.create_timer(1.0, self.timer_cb)
        
    def timer_cb(self):
        if self.state is None:
            return
        if self.goal is None:
            return
        self.create_plan()
        self.publish_plan()

    def goal_cb(self, msg):
        self.goal = msg.data
        self.get_logger().info(f"received goal {self.goal}")

    def state_cb(self, msg):
        self.state = msg.data
        self.get_logger().info(f"received state {self.state}")
    
    def create_plan(self):
        self.plan =  nx.shortest_path(self.env.g, self.state, self.goal)
        self.get_logger().info(f"plan: {self.plan}")
    
    def publish_plan(self):
        msg = Int32MultiArray()
        msg.data = self.plan
        self.plan_pub.publish(msg)


def main():
    main_fn("ccr_global_planner", CCRGlobalPlanner)

if __name__ == '__main__':
    main()