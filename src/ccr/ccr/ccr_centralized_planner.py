
from std_msgs.msg import Int32, Int32MultiArray, String
from driving_swarm_messages.msg import BeliefState as BeliefStateMsg

import yaml
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
from polygonal_roadmaps.planning import CBSPlanner, Plans

class CCRCentralizedPlanner(DrivingSwarmNode):

    def __init__(self):
        super().__init__('centralized_planner')
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        
        # get graph stucture
        self.declare_parameter('graph_file', 'graph.yaml')
        self.declare_parameter('x_min', -2.0)
        self.declare_parameter('x_max', 3.0)
        self.declare_parameter('y_min', -2.0)
        self.declare_parameter('y_max', 1.0)
        self.declare_parameter('inflation_size', 0.2)
        map_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.get_logger().info(f'loading graph from {map_file}')
        wx = (self.get_parameter('x_min').get_parameter_value().double_value, self.get_parameter('x_max').get_parameter_value().double_value)
        wy = (self.get_parameter('y_min').get_parameter_value().double_value, self.get_parameter('y_max').get_parameter_value().double_value)
        self.declare_parameter('grid_type', 'square')
        tiling = self.get_parameter('grid_type').get_parameter_value().string_value
        self.declare_parameter('grid_size', .5)
        grid_size = self.get_parameter('grid_size').get_parameter_value().double_value

        points = None
        if map_file.endswith(".yaml"):
            self.get_logger().info(f'generating {tiling} graph with grid size {grid_size} and working area {wx}x{wy}')
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


        # Subscriber and Publisher dictionaries for each robot
        self.subscribers = {}
        self.publishers = {}

        for robot in self.robot_names:
            self.subscribers[robot] = {
                'state': self.create_subscription(Int32, f'/{robot}/nav/current_node', self.create_state_callback(robot), 10),
                'goal': self.create_subscription(Int32, f'/{robot}/nav/goal_node', self.create_goal_callback(robot), 10)
            }
            self.publishers[robot] = self.create_publisher(Int32MultiArray, f'/{robot}/nav/plan', 10)

	    # initialize dictionary for state and goal information
        self.central_plan = {}
        for robot in self.robot_names:
            self.central_plan[robot]['state'] = -1
            self.central_plan[robot]['goal'] = -1

        # Set up timer to distribute plans periodically
        self.timer = self.create_timer(1.0, self.distribute_plans)

    def create_state_callback(self, robot_name):
        def callback(msg):
            # Handle state update from robot
            self.get_logger().info(f'Received state {msg.data} from {robot_name}')
            # Example of updating internal plan state
            self.central_plan[robot_name]['state'] = msg.data
        return callback

    def create_goal_callback(self, robot_name):
        def callback(msg):
            # Handle goal update from robot
            self.get_logger().info(f'Received goal {msg.data} from {robot_name}')
            # Example of updating internal plan goal
            self.central_plan[robot_name]['goal'] = msg.data
        return callback
    

    def generate_plan_for_robots(self):
        # TODO : calculate plan and add them to the central_plan dictionary
        cbs = CBSPlanner(self.env)
        all_plans = cbs.create_plan(self.env)

        # match plans to robots
        for robot in self.robot_names :
            for i in range(len(all_plans)) :
                if (all_plans[i][0] == self.central_plan[robot]['state'] and all_plans[i][len(all_plans[i])] == self.central_plan[robot]['goal']):
                    self.central_plan[robot]['plan'] = all_plans[i]

    def distribute_plans(self):
        # Example logic to generate and distribute plans to all robots
        
        # check if all robots have a state and goal
        for robot in self.robot_names :
            if (self.central_plan[robot]['state']== -1 or self.central_plan[robot]['goal']== -1) :
                return
        
        self.generate_plan_for_robots()
        
        for robot in self.robot_names:
            plan = self.central_plan[robot]['plan']
            msg = Int32MultiArray()
            msg.data = plan
            self.publishers[robot].publish(msg)
            self.get_logger().info(f'Sent plan {plan} to {robot}')



def main():
    main_fn('ccr_centralized_planner', CCRCentralizedPlanner)

if __name__ == '__main__':
    main()

