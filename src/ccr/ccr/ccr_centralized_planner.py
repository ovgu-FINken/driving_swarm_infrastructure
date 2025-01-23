
from std_msgs.msg import Int32, Int32MultiArray, String
from driving_swarm_messages.msg import BeliefState as BeliefStateMsg
from geometry_msgs.msg import Twist
import yaml
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
from polygonal_roadmaps.planning import CBSPlanner, Plans, PriorityAgentPlanner, compute_all_k_conflicts
from polygonal_roadmaps.planning import PBSPlanner
import networkx as nx
import time

import copy

class CCRCentralizedPlanner(DrivingSwarmNode):

    def __init__(self, name):
        super().__init__(name)
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
        self.publishers_ = {}
        self.velocity_publishers_ = {}

        for robot in self.robot_names:
            self.subscribers[robot] = {
                'state': self.create_subscription(Int32, f'/{robot}/nav/current_node', self.create_state_callback(robot), 10),
                'goal': self.create_subscription(Int32, f'/{robot}/nav/goal_node', self.create_goal_callback(robot), 10)
            }
            self.publishers_[robot] = self.create_publisher(Int32MultiArray, f'/{robot}/nav/plan', 10)
            self.velocity_publishers_[robot] = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)

            # global stop service
            ##self.global_stop_client[robot] = self.create_client(bool, f'/{robot}/global_stop')

	    # initialize dictionary for state and goal information
        self.central_plan = {}
        for robot in self.robot_names:
            self.central_plan[robot] = {}
            self.central_plan[robot]['state'] = -1
            self.central_plan[robot]['goal'] = -1
            self.central_plan[robot]['plan'] = (0,0)
            self.central_plan[robot]['origin_plan'] = []

        self.stop_flag = False
        self.timer_start = time.time() - 100
        
        

        # Set up timer to distribute plans periodically
        self.timer = self.create_timer(1.0, self.distribute_plans)
        

        self.use_optimization = True
        #self.use_optimization = False

        self.get_logger().info(f'------------------')
        self.get_logger().info(f'\n\n   using optimization : {self.use_optimization}\n\n')
        self.get_logger().info(f'------------------')
        


        

    def create_state_callback(self, robot_name):
        def callback(msg):
            # Handle state update from robot
            self.get_logger().info(f'Received state {msg.data} from {robot_name}')
            # Example of updating internal plan state
            self.central_plan[robot_name]['state'] = msg.data
            self.env
        return callback

    def create_goal_callback(self, robot_name):
        def callback(msg):
            # Handle goal update from robot
            if msg.data == self.central_plan[robot_name]['goal']:
                return
            self.get_logger().info(f'Received goal {msg.data} from {robot_name}')
            # Example of updating internal plan goal
            self.central_plan[robot_name]['goal'] = msg.data

            self.got_new_goal = True

        return callback
    



    def generate_plan_for_robots(self):


        # TODO : calculate plan and add them to the central_plan dictionary

        self.env.state= [self.central_plan[robot]['state']  for robot in self.robot_names]
        self.env.goal = [self.central_plan[robot]['goal']  for robot in self.robot_names]

        cbs = CBSPlanner(self.env, max_iter = 1_000_000)
        pp = PriorityAgentPlanner(self.env, priority_method="longest" ,max_iter = 1_000_000)
        pbs = PBSPlanner(self.env, max_iter = 1_000_000)

        all_plans = []
        try:
            all_plans = pp.create_plan(self.env)
        except nx.NetworkXNoPath:
            self.get_logger().info("################# Error ##################")
            exit()
            
        # match plans to robots
        for robot in self.robot_names :
            for i in range(len(all_plans.plans)) :
                if (all_plans.plans[i][0] == self.central_plan[robot]['state'] and all_plans.plans[i][len(all_plans[i])-1] == self.central_plan[robot]['goal']):
                    self.central_plan[robot]['plan'] = all_plans.plans[i]
                    self.central_plan[robot]['origin_plan'] = copy.deepcopy(all_plans.plans[i])
                    

    def global_stop(self):
        # sending the current state as a plan to each robot 
        self.stop_flag = True
        self.timer_start = time.time()

        for robot in self.robot_names:
            plan = [self.central_plan[robot]['state']]
            msg = Int32MultiArray()
            msg.data = plan
            self.publishers_[robot].publish(msg)

            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.velocity_publishers_[robot].publish(stop_msg)

        self.get_logger().info(f'global stop : active')


    def activate_global_stop(self):
        self.stop_flag = True

        for robot in self.robot_names:
            msg = std_srvs.SetBool
            msg.request = True
            self.global_stop_client[robot].call(msg)

        self.get_logger().info(f'global stop : activated')

    def deactivate_global_stop(self):
        self.stop_flag = False

        for robot in self.robot_names:
            msg = std_srvs.SetBool
            msg.request = False
            self.global_stop_client[robot].call(msg)

        self.get_logger().info(f'global stop : deactivated')


    def distribute_plans(self):

        # Example logic to generate and distribute plans to all robots
        
        if time.time() - self.timer_start <= 1:
            return

        # check if all robots have a state and goal
        for robot in self.robot_names :
            if (self.central_plan[robot]['state']== -1 or self.central_plan[robot]['goal']== -1) :
                return
    

        if (self.stop_flag):
            #self.get_logger().info(f'global stop : not active')

            ##self.deactivate_global_stop()

            self.generate_plan_for_robots()

            self.stop_flag = False



            for robot in self.robot_names:
                plan = self.central_plan[robot]['plan']
                msg = Int32MultiArray()
                msg.data = plan
                self.publishers_[robot].publish(msg)
                self.get_logger().info(f'Sent plan {plan} to {robot}')

            
            return


        if (self.use_optimization) :

            already_replaned = False

            if (self.got_new_goal) :
                already_replaned = True
                self.get_logger().info("\n\n replaned due to new goal  Runde 9\n")
                #self.generate_plan_for_robots()
                self.global_stop()
                
                ##self.activate_global_stop()

            
            self.got_new_goal = False


            if not(already_replaned) :

                min_timestep = -1
                max_timestep = -1

                wrong_localization = False

                # shorten paths if state progressed since last call
                # also calculate max time difference 
                for robot in self.robot_names :
                    
                    # robot moved to second step of plan
                    if (self.central_plan[robot]['plan'][1] == self.central_plan[robot]['state']) :
                        self.central_plan[robot]['plan'].pop(0)
                    
                    # robot moved, but outside of considered range 
                    elif (self.central_plan[robot]['plan'][0] != self.central_plan[robot]['state']) :
                        self.get_logger().info("plan progressed far more than expected (current state not first or second step in plan)")
                        wrong_localization = True
                        break
                    

                    a = len(self.central_plan[robot]['origin_plan'])
                    b =  len(self.central_plan[robot]['plan'])
                    temp_timestep = a - b




                    # is timestep new maximum?
                    if (temp_timestep > max_timestep or max_timestep == -1) :
                        max_timestep = temp_timestep 

                    # is timestep new minimum?
                    if (temp_timestep < min_timestep or min_timestep == -1) :
                        min_timestep = temp_timestep

                
                all_conflicts = list(compute_all_k_conflicts([self.central_plan[i]['plan'] for i in self.robot_names]))

                unique_robots = []
                for i in range(len(all_conflicts)):
                    unique_robots = set(val.agent for val in all_conflicts[i].conflicting_agents)
                    if (len(unique_robots) > 1) :
                        break
                            
                

                total_time_diff = max_timestep - min_timestep

                if (len(unique_robots) > 1) :
                    self.get_logger().info(f"\n\n replaned due to k-1 conflict in current plan \n")
                    #self.generate_plan_for_robots()
                    ##self.activate_global_stop()
                    self.global_stop()
                elif (total_time_diff > 1) :
                    self.get_logger().info(f"\n\n replaned due to time difference limit : {total_time_diff} \n")
                    #self.generate_plan_for_robots()
                    ##self.activate_global_stop()
                    self.global_stop()
                elif (wrong_localization) :
                    self.get_logger().info(f"\n\n replaned due to unexpected movement or wrong localization \n")
                    #self.generate_plan_for_robots()
                    ##self.activate_global_stop()
                    self.global_stop()
                else : 
                    self.get_logger().info("\n\n did not replan \n")


        else :
            self.generate_plan_for_robots()
        
        if (self.stop_flag):
            return

        for robot in self.robot_names:
            plan = self.central_plan[robot]['plan']
            msg = Int32MultiArray()
            msg.data = plan
            self.publishers_[robot].publish(msg)
            self.get_logger().info(f'Sent plan {plan} to {robot}')



def main():
    main_fn('ccr_centralized_planner', CCRCentralizedPlanner)

if __name__ == '__main__':
    main()

