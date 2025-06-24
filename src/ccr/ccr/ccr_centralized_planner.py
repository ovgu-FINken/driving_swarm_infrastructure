
from std_msgs.msg import Int32, Int32MultiArray, String
from driving_swarm_messages.msg import BeliefState as BeliefStateMsg
from geometry_msgs.msg import Twist
import yaml
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment, planning
from polygonal_roadmaps.planning import CBSPlanner, Plans, PriorityAgentPlanner, compute_all_k_conflicts, has_k1_collision
from polygonal_roadmaps.planning import PBSPlanner
import networkx as nx
import time
from std_srvs.srv import SetBool

import functools

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
        self.global_stop_client = {}

        for robot in self.robot_names:
            self.subscribers[robot] = {
                'state': self.create_subscription(Int32, f'/{robot}/nav/current_node', self.create_state_callback(robot), 10),
                'goal': self.create_subscription(Int32, f'/{robot}/nav/goal_node', self.create_goal_callback(robot), 10)
            }
            self.publishers_[robot] = self.create_publisher(Int32MultiArray, f'/{robot}/nav/plan', 10)
            self.velocity_publishers_[robot] = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)

            # global stop service
            self.global_stop_client[robot] = self.create_client(SetBool, f'/{robot}/global_stop')

	    # initialize dictionary for state and goal information
        self.central_plan = {}
        self.waiting_list = {robot : [] for robot in self.robot_names}
        self.is_waiting = {robot : False for robot in self.robot_names}
        self.stop_response = {robot : False for robot in self.robot_names}

        for robot in self.robot_names:
            self.central_plan[robot] = {}
            self.central_plan[robot]['state'] = -1
            self.central_plan[robot]['goal'] = -1
            self.central_plan[robot]['plan'] = (0,0)
            self.central_plan[robot]['origin_plan'] = []

        self.stop_flag = False
        self.timer_start = time.time() 
        
        self.first_plan = True

        # Set up timer to distribute plans periodically
        self.timer = self.create_timer(1.0, self.update_plans)
        


        self.get_logger().info(f'------------------')
        self.get_logger().info(f'\n\n Runde 14\n')
        self.get_logger().info(f'------------------')
        
    

    def create_state_callback(self, robot):
        def callback(msg):
            #self.get_logger().warn(f'\nhey we got some info\n')
            # Handle state update from robot
            self.get_logger().info(f'Received state {msg.data} from {robot}')
            # Example of updating internal plan state
            self.central_plan[robot]['state'] = msg.data
            #self.env

            # should the robot stop and wait for other robots?
            if not(self.is_waiting[robot]):
                self.send_single_wait_command(robot)

        return callback

    def create_goal_callback(self, robot):
        def callback(msg):
            # Handle goal update from robot
            if msg.data == self.central_plan[robot]['goal']:
                return
            self.get_logger().info(f'Received goal {msg.data} from {robot}')
            # Example of updating internal plan goal
            self.central_plan[robot]['goal'] = msg.data

            self.got_new_goal = True

            self.get_logger().info("\n\n replaned due to new goal \n")
            #self.generate_plan_for_robots()
            
            self.activate_global_stop()

            self.got_new_goal = False

        return callback
    

    def generate_plan_for_robots(self):

        cur_state = []

        for robot in self.robot_names:
            cur_state.append(self.central_plan[robot]['state'])
            if (cur_state.count(self.central_plan[robot]['state']) > 1):
                self.get_logger().info(f"\n\noh no, multiple robots are in the same cell")
                for robot in self.robot_names:
                    self.get_logger().info(f"robot {robot} : {str(self.central_plan[robot]['state'])}")
                return

        self.env.state= [self.central_plan[robot]['state']  for robot in self.robot_names]
        self.env.goal = [self.central_plan[robot]['goal']  for robot in self.robot_names]

        cbs = CBSPlanner(self.env, max_iter = 1_000_000)
        pp = PriorityAgentPlanner(self.env, priority_method="longest" ,max_iter = 1_000_000)
        pbs = PBSPlanner(self.env,"width")


        self.get_logger().info("\n\nCALCULATING NEW PLANS\n")
        all_plans = []
        try:
            all_plans = pbs.create_plan(self.env)
        except nx.NetworkXNoPath:
            self.get_logger().info("Error : no plan with compatible paths was found")
            self.get_logger().info(f"current positions of robots :")
            for robot in self.robot_names:
                self.get_logger().info(f"robot {robot} : {str(self.central_plan[robot]['state'])}")

            self.get_logger().info(f"\n\n\n")
            return

            exit()

        #all_plans.plans[0].insert(1,all_plans.plans[0][1])
            
        # match plans to robots
        for robot in self.robot_names :
            for i in range(len(all_plans.plans)) :
                if (all_plans.plans[i][0] == self.central_plan[robot]['state'] and all_plans.plans[i][len(all_plans[i])-1] == self.central_plan[robot]['goal']):
                
                    self.central_plan[robot]['plan'] = all_plans.plans[i]
                    self.central_plan[robot]['origin_plan'] = copy.deepcopy(all_plans.plans[i])


        self.identify_waiting_times()



    def identify_waiting_times(self):

        for robot in self.robot_names :
            i = 1
            while i < len(self.central_plan[robot]['plan']):
                if (self.central_plan[robot]['plan'][i] == self.central_plan[robot]['plan'][i-1]):
                    for j in range(i, len(self.central_plan[robot]['plan'])):
                        if (self.central_plan[robot]['plan'][j] != self.central_plan[robot]['plan'][j-1]):
                            self.waiting_list[robot].append((i-1,j-1))
                            i = j
                            break
                i += 1


    def activate_global_stop(self):
        self.stop_flag = True
        
        future_response = {}
        for robot in self.robot_names:
            request = SetBool.Request()
            request.data = True
            extra_func = functools.partial(self.handle_stop_response,robot)
            try:
                future_response[robot] = self.global_stop_client[robot].call_async(request)
                future_response[robot].add_done_callback(extra_func)
                #future_response[robot].add_done_callback(self.handle_stop_response)
            except Exception as e:
                self.get_logger().error(f"Failed to send the stop signal via service to Roboter {robot} : {str(e)}")
        

        self.get_logger().info(f'global stop : activated')


    def deactivate_global_stop(self):
        self.stop_flag = False

        future_response = {}
        for robot in self.robot_names:
            request = SetBool.Request()
            request.data = False
            extra_func = functools.partial(self.handle_stop_response,robot)
            try:
                future_response[robot] = self.global_stop_client[robot].call_async(request)
                future_response[robot].add_done_callback(extra_func)
                #future_response[robot].add_done_callback(self.handle_stop_response)

            except Exception as e:
                self.get_logger().error(f"Failed to send the stop signal via service to Robot {robot} : {str(e)}")

        self.get_logger().info(f'global stop : deactivated')


    def handle_stop_response(self,robot,future):
        robot_id = robot
        additional = ""
        if (robot_id==""):
            additional = "a random "
        else:
            self.stop_response[robot]=True


        try:
            response = future.result()

            if response.success:
                self.get_logger().info(f"service call response sucessful for {additional}robot {robot_id}")
            else :
                self.get_logger().warn(f"service call response not sucessful for {additional} robot {robot_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to handle the service call response for {additional} robot {robot_id} : {str(e)}")
        

    def send_plans_to_robots(self):
        
        #self.get_logger().info(f"\n\n an updated plan got sent to the robots \n")

        for robot in self.robot_names:
            plan = self.central_plan[robot]['plan']
            msg = Int32MultiArray()
            msg.data = plan
            self.publishers_[robot].publish(msg)
            self.get_logger().info(f'Sent plan {plan} to {robot}')


    def update_plans(self):

        # check if all robots have a state and goal
        for robot in self.robot_names :
            if (self.central_plan[robot]['state']== -1 or self.central_plan[robot]['goal']== -1) :
                return

        if (self.first_plan):
            self.get_logger().info(f'used this for the first time to plan')
            self.generate_plan_for_robots()

            self.first_plan = False

            self.send_plans_to_robots()

            self.deactivate_global_stop()


            self.got_new_goal = False
            return


        if (self.stop_flag):
            #self.get_logger().info(f'global stop : not active')

            # check if all robots acknowledged the stop signal
            for robot in self.robot_names :
                if (self.stop_response[robot]==False):
                    self.get_logger().info(f' global stop is active BUT robot {robot} is not responsive\n\n')
                    return

            self.generate_plan_for_robots()

            self.send_plans_to_robots()

            self.deactivate_global_stop()

            return

        self.reasons_to_replan()

        if (self.stop_flag):
            return
        
        self.get_logger().info(f"\n\n did not replan \n")

        self.send_plans_to_robots()

    
    def has_k1(self,plans):
        prev_timestep = [val[0] for val in plans]

        for i in range(1,4):
            temp = []
            for j, robot in enumerate(self.robot_names):
                if len(plans[j])<i+1:
                    continue
                if self.is_waiting[robot]:
                    continue
                if prev_timestep.count(plans[j][i]) > 0 and not(plans[j][i]==plans[j][i-1]):
                    return True
                if temp.count(plans[j][i])>0:
                    return True
                temp.append(plans[j][i])
            prev_timestep = temp.copy()
        return False

    
    def has_k1_oo(self, plans):
        for indx1, plan1 in enumerate(plans):
            for indx2, plan2 in enumerate(plans):
                if plan1 == plan2:
                    continue
                for i in range(len(plan1)):
                    try:
                        if plan1[i] == plan2[i] or plan1[i] == plan2[i+1]:
                            return True, (indx1+1,indx2+1)
                    except IndexError:
                        pass
        return False, (0,0)




    def reasons_to_replan(self):

        min_timestep = -1
        max_timestep = -1

        wrong_localization = False

        # shorten paths if state progressed since last call
        # also calculate max time difference 
        total_time_diff = 0
        for robot in self.robot_names :
            
            # robot moved to second step of plan
            #self.get_logger().info(f"comparing data :{self.central_plan[robot]['plan'][1]} and {self.central_plan[robot]['state']} ")
            if (self.central_plan[robot]['plan'][1] == self.central_plan[robot]['state']) :
                self.central_plan[robot]['plan'].pop(0)
            
            # robot moved, but outside of considered range 
            elif (self.central_plan[robot]['plan'][0] != self.central_plan[robot]['state']) :
                self.get_logger().info("plan progressed far more than expected (current state not first or second step in plan)")
                wrong_localization = True
                break

            if self.is_waiting[robot]:
                continue
                                
            a = len(self.central_plan[robot]['origin_plan'])
            b = len(self.central_plan[robot]['plan'])
            temp_timestep = a - b

            # is timestep new maximum?
            if (temp_timestep > max_timestep or max_timestep == -1) :
                max_timestep = temp_timestep 

            # is timestep new minimum?
            if (temp_timestep < min_timestep or min_timestep == -1) :
                min_timestep = temp_timestep

            total_time_diff = max_timestep - min_timestep

        all_conflicts_preprocessed = []
        for robot in self.robot_names :
            
            # shorten plan if length to long
            if len(self.central_plan[robot]['plan']) < 4:
                all_conflicts_preprocessed.append(self.central_plan[robot]['plan'])
            else:
                all_conflicts_preprocessed.append(self.central_plan[robot]['plan'][:3])

        
        all_conflicts = list(compute_all_k_conflicts(all_conflicts_preprocessed, limit=None, k=1))


        unique_robots = []
        k1_conf = False
        for i in range(len(all_conflicts)):
            unique_robots = set(val.agent for val in all_conflicts[i].conflicting_agents)

            if (len(unique_robots) > 1) :
                #self.get_logger().info(f"\n\n\n WORKS: {unique_robots} \n\n\n")
                k1_conf = True
                break


        new_k1_conf = self.has_k1(all_conflicts_preprocessed)
        newer_k1_conf, _ = self.has_k1_oo(all_conflicts_preprocessed)

        if (k1_conf or new_k1_conf) :
            self.get_logger().info(f"\n\n replaned due to k-1 conflict in current plan \n specific reason: old: {k1_conf} | new: {new_k1_conf} | newer: {newer_k1_conf}\n")
            #self.generate_plan_for_robots()
            self.activate_global_stop()

        elif (total_time_diff > 1) :
            self.get_logger().info(f"\n\n replaned due to time difference limit : {total_time_diff} \n")
            #self.generate_plan_for_robots()
            self.activate_global_stop()
        
        elif (wrong_localization) :
            self.get_logger().info(f"\n\n replaned due to unexpected movement or wrong localization \n")
            #self.generate_plan_for_robots()
            self.activate_global_stop()

        else : 
            #self.get_logger().info("\n\n did not replan \n")
            self.send_single_go_command(min_timestep)


    def send_single_go_command(self, min_timestep):
        for robot in self.robot_names :
            if not(self.is_waiting[robot]):
                continue
            
            if (min_timestep >= self.waiting_list[robot][0][1]):
                self.is_waiting[robot] = False

                future_response = {}
                request = SetBool.Request()
                request.data = False
                extra_func = functools.partial(self.handle_stop_response,robot)
                try:
                    future_response[robot] = self.global_stop_client[robot].call_async(request)
                    future_response[robot].add_done_callback(extra_func)
                    #future_response[robot].add_done_callback(self.handle_stop_response)
                except Exception as e:
                    self.get_logger().error(f"Failed to send the stop signal via service to Robot {robot} : {str(e)}")


    def send_single_wait_command(self,robot):
        if (len(self.waiting_list[robot])>0):
            a = len(self.central_plan[robot]['origin_plan'])
            b = len(self.central_plan[robot]['plan'])
            timestep = a - b

            if (timestep == self.waiting_list[robot][0][0]):
                self.is_waiting[robot] = True

                future_response = {}
                request = SetBool.Request()
                request.data = True
                extra_func = functools.partial(self.handle_stop_response,robot)
                try:
                    future_response[robot] = self.global_stop_client[robot].call_async(request)
                    future_response[robot].add_done_callback(extra_func)
                    #future_response[robot].add_done_callback(self.handle_stop_response)
                except Exception as e:
                    self.get_logger().error(f"Failed to send the stop signal via service to Roboter {robot} : {str(e)}")


def main():
    main_fn('ccr_centralized_planner', CCRCentralizedPlanner)

if __name__ == '__main__':
    main()

    # TODO :
    #  - k1-conflict with waiting robot
    #       - either ignore waiting robots in k1-calculation or dont update the plan of waiting robots
    #  - do something in generate_new_plans when 2 (or more) robots are in the same starting location
    #       - hopefully they will just drive according to current plan and get out of the same cell