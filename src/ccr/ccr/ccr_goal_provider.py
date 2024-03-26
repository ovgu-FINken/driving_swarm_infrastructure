import yaml
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment
import functools
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from termcolor import colored

class CCRGoalProvider(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)
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
        self.get_list_of_robot_names()

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

        # get start state and goal states
        self.declare_parameter('waypoints', '')
        waypoints = self.get_parameter('waypoints').get_parameter_value().string_value
        self.get_logger().info(f'read waypoints from {waypoints}')
        with open(waypoints, 'r') as stream:
             goal_file = yaml.safe_load(stream)

        # here, we need a list of goals for each robot
        # the goals are converted to cell ids using the roadmap
        assert len(goal_file) >= len(self.robots), f'not enough goals for all robots, got {len(goal_file)} goals for {len(self.robots)} robots'
        
        def convert_pos_to_cell_id(index):
            cells = []
            for x, y, *_ in goal_file[index]['waypoints']:
                cells.append(self.env.find_nearest_node((x, y)))
            return cells
        
        self.goals = { robot: convert_pos_to_cell_id(i) for i, robot in enumerate(self.robots) }
        self.get_logger().info(f'goals: {self.goals}')
        self.goal_index = { robot: 0 for robot in self.robots }
        self.goals_completed = {robot: 0 for robot in self.robots}

        # subscribe cells messages
        self.cells = { robot: None for robot in self.robots }
        for robot in self.robots:
            self.create_subscription(Int32,
                                     f'/{robot}/nav/current_node',
                                     functools.partial(self.cell_cb, robot),
                                     10)
        
        # setup timer for publishing goals
        self.goal_publishers = { robot: self.create_publisher(Int32, f'/{robot}/nav/goal_node', 10) for robot in self.robots }
        self.completed_publishers = { robot: self.create_publisher(Int32, f'/{robot}/nav/goal_completed', 10) for robot in self.robots }
        self.position_publishers = { robot: self.create_publisher(PoseStamped, f'/{robot}/nav/goal_position', 10) for robot in self.robots }
        self.create_timer(0.1, self.timer_cb)
        
        
    def cell_cb(self, robot, msg):
        self.cells[robot] = msg.data
        # advance goals if applicable
        if self.cells[robot] == self.goals[robot][self.goal_index[robot]]:
            self.advance_goal(robot)
    
    def advance_goal(self, robot):
        self.goal_index[robot] = (self.goal_index[robot] + 1) % len(self.goals[robot])
        self.goals_completed[robot] += 1
        output = colored('goal completed:', 'green') + f' {robot} reached goal {self.cells[robot]}, going to goal {self.goal_index[robot]} within state {self.goals[robot][self.goal_index[robot]]}'
        self.get_logger().info(output)

    def timer_cb(self):
        # publish goals
        for robot in self.robots:
            data = Int32()
            data.data = int(self.goals[robot][self.goal_index[robot]])
            self.goal_publishers[robot].publish(data)
        # publish number of goals completed
        # publish goal position
        for robot in self.robots:
            data = Int32()
            data.data = self.goals_completed[robot]
            self.completed_publishers[robot].publish(data)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            coords = self.env.g.nodes(data=True)[self.goals[robot][self.goal_index[robot]]]['geometry'].center.coords
            pose.pose.position.x, pose.pose.position.y = coords[0][0], coords[0][1]
            self.position_publishers[robot].publish(pose)

        
def main():
    main_fn('ccr_goal_provider', CCRGoalProvider)

if __name__ == '__main__':
    main()