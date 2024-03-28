import yaml
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
import networkx as nx
import numpy as np
import functools
from std_msgs.msg import Int32MultiArray, Int32
from termcolor import colored

class CCRFakeExecution(DrivingSwarmNode):
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

        self.get_list_of_robot_names()

        # get robot poses
        self.declare_parameter('poses', '')
        poses_file = self.get_parameter('poses').get_parameter_value().string_value
        if poses_file == '':
            self.get_logger().warn('no poses file specified')
            return
        self.get_logger().info(f'read poses from {poses_file}')
        with open(poses_file, 'r') as stream:
            poses = yaml.safe_load(stream)
        
        # convert starting pose to state
        self.states = {}
        for robot, pose in zip(self.robots, poses):
            state = self.env.find_nearest_node(pose[:2])
            self.get_logger().info(f'{robot} at position {pose} = state {state}')
            self.states[robot] = state
        
        self.state_pub = {}
        self.plans = {}
        for robot in self.robots:
            self.create_subscription(Int32MultiArray, f"/{robot}/nav/plan", functools.partial(self.plan_cb, robot), 10)
            self.state_pub[robot] = self.create_publisher(Int32, f'/{robot}/nav/current_node', 10)

        self.create_timer(1.0, self.timer_cb) 


    def timer_cb(self):
        self.advance_plans()
        self.publish_state()
    
    def plan_cb(self, robot, msg):
        plan = list(msg.data)
        self.get_logger().info(f'got plan for {robot}: {plan}')
        self.plans[robot] = plan
        
    def advance_plans(self):
        for robot, plan in self.plans.items():        
            if len(plan) < 2:
                continue
            if plan[0] != self.states[robot]:
                self.get_logger().warn(f'{robot} is not at the expected state {plan[0]}, instead at {self.state[robot]}')
                continue
            self.get_logger().info(f'{robot}: {self.states[robot]} -> {plan[1]}')
            self.states[robot] = plan[1]
    
    def publish_state(self):
        for robot, state in self.states.items():
            data = Int32()
            data.data = int(state)
            self.state_pub[robot].publish(data)

        
def main():
    main_fn('ccr_fake_execution', CCRFakeExecution)

if __name__ == '__main__':
    main()