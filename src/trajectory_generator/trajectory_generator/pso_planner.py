#!/usr/bin/env python

import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
import numpy as np
from scipy import optimize
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from driving_swarm_messages.srv import VehicleModel
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, OccupancyGrid
from trajectory_generator.vehicle_model_node import TrajectoryGenerator, Vehicle

def granularity(x):
    return int(np.count_nonzero(~np.isnan(x)) / 3)

class PSOPlanner(Node):
    def __init__(self):
        super().__init__('pso_planner')
        self.get_logger().info('Starting')
        self.own_frame = 'base_link'
        self.reference_frame = 'map'
        self.current_gh = None

        self.w_time = 1.0
        self.w_length = 0.0
        self.w_danger = 100

        self.declare_parameter('vehicle_model')
        self.declare_parameter('step_size')
        self.declare_parameter('turn_radius')
        self.vm = TrajectoryGenerator(
            model = Vehicle(self.get_parameter('vehicle_model').get_parameter_value().integer_value),
            step = self.get_parameter('step_size').get_parameter_value().double_value,
            r = self.get_parameter('turn_radius').get_parameter_value().double_value
        )

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.costmap_initalised = False
        self.create_subscription(OccupancyGrid, 'nav/obstacles', self.obstacle_cb, 9)

        self.follow_action_client = ActionClient(self, FollowPath, 'nav/follow_path')
        self.vm_client_futures = []
        self.action_client_futures = []
        self.action_result_futures = []
        self.follow_action_client.wait_for_server()
        self.get_logger().info('connected to trajectory follower service')
        self.goal = None
        f = self.tfBuffer.wait_for_transform_async(self.own_frame, self.reference_frame, rclpy.time.Time().to_msg())
        self.get_logger().info('waiting for transform map -> baselink')
        rclpy.spin_until_future_complete(self, f)
        while not self.costmap_initalised:
            rclpy.spin_once(self)

        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 9)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('setup done')
        
    def goal_cb(self, msg):
        if self.goal is None:
            self.get_logger().info(f'got goal')#: {msg}')
            self.goal = msg
            self.compute_path()
        elif self.goal.pose != msg.pose:
            self.get_logger().info('got new goal')
            self.goal = msg
            self.cancel_all_current_actions()
            self.compute_path()
        else:
            self.get_logger().debug('got old goal')
            
    def obstacle_cb(self, msg):
        self.costmap_initalised = True
        self.vm.set_costmap(msg)

    def cancel_all_current_actions(self):
        self.get_logger().info('cancel current actions')
        self.vm_client_futures = []
        self.action_client_futures = []
        if self.current_gh is not None:
            self.current_gh.cancel_goal_async()
        self.action_result_futures = []

    # TODO: it would be much nicer to create a callback attached to the future, however this is not as simple (?) 
    # probably needs a reentrant callback
    def timer_cb(self):
        #self.get_logger().info('timer cb')
        for future in self.action_client_futures:
            if future.done():
                self.get_logger().info('action submit complete')
                if self.current_gh is not None:
                    self.get_logger().info('Cancelling old goal')
                    self.current_gh.cancel_goal_async()
                self.action_client_futures.remove(future)
                self.current_gh = future.result()
                self.get_logger().info(f'goal accepted: {self.current_gh.accepted}')
                self.action_result_futures.append(self.current_gh.get_result_async())
        for future in self.action_result_futures:
            if future.done():
                self.get_logger().info('action complete')
                self.action_result_futures.remove(future)
                self.current_gh = None
                self.compute_path()
            
    def weighted_sum(self, traj):
        objectives = self.vm.compute_trajectory_objectives( traj )
        return self.w_time * objectives['time'] +\
            self.w_length * objectives['length']+ \
            self.w_danger * objectives['danger']

    def evaluate_solution(self, wps):
        trajectory = self.vm.tuples_to_path([self.start_wp] +  wps + [self.goal_wp])
        return self.weighted_sum( trajectory )
    
    def fitness(self, x):
        x = x[~np.isnan(x)] 
        wps = [(x,y,t,1.0) for x,y,t in zip(x[0::3], x[1::3], x[2::3])]
        return self.evaluate_solution(wps)
    
    def mg_pso(self):
        # Multi-Granularity PSO
        pop_size = 40
        n_gen = 200
        c_social = 1.4
        c_cognitive = 1.4
        w = 0.5
        granularity_0 = 1
        population = np.random.rand(pop_size, granularity_0*3)

        v_t = np.zeros_like(population)
        pbest = population.copy()
        pbest_fitness = []
        for p in population:
            pbest_fitness.append(self.fitness(p))
        gbest_fitness = np.min(pbest_fitness)
        gbest = pbest[pbest_fitness.index(gbest_fitness)]
#dfs.append(sol_to_df(pbest, poly, generation=0, ndim))
        print(gbest_fitness)
        for _ in range(n_gen):
            v_cognitive = c_cognitive * np.random.rand(*population.shape) * np.nan_to_num(pbest - population)
            #print("cognitive:")
            #print(v_cognitive)
            v_social = c_social * np.random.rand(*population.shape) * np.nan_to_num(gbest - population)
            #print("social:")
            #print(v_social)
            population_ = w * v_t + v_cognitive + v_social + population #+ 0.001 * (np.random.rand(*population.shape) - 0.5)
            np.clip(population_, -5.0, 5.0, population_)
            v_t = np.nan_to_num(population_ - population)
            #print("v_t:")
            #print(v_t)
            population = population_

            # if there is no room for granularity boost, make room
            if not np.isnan(population).any(axis=1).all():
                nan_array = np.zeros((pop_size,3))
                nan_array[:]=np.nan
                population = np.hstack([population, nan_array])
                pbest = np.hstack([pbest, nan_array])
                v_t = np.hstack([v_t,np.zeros((pop_size,3))])
            
            p_g_inc = 0.33 - len([1 for p in population if granularity(p) > granularity(gbest)]) / pop_size
            p_g_inc *= 0.25
            for i, p in enumerate(population):
                # if previous best has lower granularity than current state, don't increase granularity
                if granularity(gbest) < granularity(p):
                    #if np.random.rand() < 0.01:
                        #granularity drop
                    #    x = np.min(np.argwhere(np.isnan(p)).flatten()) - 2
                    #    if x <= 0:
                    continue
                    #    population[i,x] = np.random.rand()
                    #    population[i,x] = np.random.rand()
                elif np.random.rand() < p_g_inc or granularity(p) < granularity(gbest):
                    x = np.min(np.argwhere(np.isnan(p)).flatten())
                    w = np.random.rand()
                    cut_off = np.random.randint(x)
                    cut_off -= cut_off % 3
                    p_0 = p[:cut_off]
                    p_1 = p[cut_off:]
                    p_x = self.start_wp[0]
                    p_y = self.start_wp[1]
                    p_t = self.start_wp[2]
                    if cut_off > 0:
                        p_x = p_0[-3] * w
                        p_y = p_0[-2] * w
                        p_t = p_0[-1] * w
                    if np.isnan(p_1[0]):
                        p_x += self.goal_wp[0] * (1 - w)
                        p_y += self.goal_wp[1] * (1 - w)
                        p_t += self.goal_wp[2] * (1 - w)
                    else:
                        p_x += p_1[0] * (1-w)
                        p_y += p_1[1] * (1-w)
                        p_t += p_1[2] * (1-w)
                    population[i] = np.hstack([p_0, [p_x, p_y, p_t], p_1[:-3]])
                    q = pbest[i]
                    q_0 = q[:cut_off]
                    q_1 = q[cut_off:]
                    q_x = self.start_wp[0]
                    q_y = self.start_wp[1]
                    q_t = self.start_wp[2]
                    if cut_off > 0:
                        q_x = q_0[-3] * w
                        q_y = q_0[-2] * w
                        q_t = q_0[-1] * w
                    if np.isnan(q_1[0]):
                        q_x += self.goal_wp[0] * (1 - w)
                        q_y += self.goal_wp[1] * (1 - w)
                        q_t += self.goal_wp[2] * (1 - w)
                    else:
                        q_x += q_1[0] * (1-w)
                        q_y += q_1[1] * (1-w)
                        q_t += q_1[2] * (1-w)
                    pbest[i] = np.hstack([q_0, [q_x, q_y, q_t], q_1[:-3]])
    
            for i,p in enumerate(population):
                f = self.fitness(p)
                if f < pbest_fitness[i]:
                    pbest_fitness[i] = f
                    pbest[i] = p
            gbest_fitness = np.min(pbest_fitness)
            gbest = pbest[pbest_fitness.index(gbest_fitness)]
        x = gbest[~np.isnan(gbest)] 
        wps = [(x,y,t,1.0) for x,y,t in zip(x[0::3], x[1::3], x[2::3])]
        return wps

    def compute_path(self):
        waypoints = self.get_waypoints()
        waypoint_tuples = [(p.x, p.y, p.theta, 1.0) for p in waypoints]
        trivial = self.vm.tuples_to_path(waypoint_tuples)
        self.start_wp = waypoint_tuples[0]
        self.goal_wp = waypoint_tuples[1]
        
        opt = self.mg_pso()
        #optimize.minimize(self.evaluate_solution, [.0, .0, .0, 1.0],method='Nelder-Mead')
        self.get_logger().info(f'best is: {opt}')
        best = self.vm.tuples_to_path([self.start_wp] + opt + [self.goal_wp])
        # check if better than trivial solution
        if self.weighted_sum(trivial) < self.weighted_sum(best): 
            best = trivial
            self.get_logger().info(f'could not find better solution than trivial.')
        self.get_logger().info(f'fitness: {self.vm.compute_trajectory_objectives(best)}')
        #self.vm.plot_trajectory_in_costmap(best)
        self.send_path([Pose2D(x=x, y=y, theta=theta) for x,y,theta in best])

    def send_path(self, trajectory):
        # convert trajectory to correct space
        path = Path()
        path.header.frame_id = self.reference_frame
        for pose in trajectory:
            pose3d = PoseStamped()
            pose3d.header.frame_id = self.reference_frame
            pose3d.header.stamp = self.get_clock().now().to_msg()
            pose3d.pose.position.x = pose.x
            pose3d.pose.position.y = pose.y
            pose3d.pose.position.z = 0.0
            pose3d.pose.orientation = yaw_to_orientation(pose.theta) 
            
            path.poses.append(pose3d)

        
        # create follow trajectory action goal
        path.header.stamp = self.get_clock().now().to_msg()
        action_goal = FollowPath.Goal(path=path)
        self.get_logger().info('sending path to action server')
        self.action_client_futures.append(self.follow_action_client.send_goal_async(action_goal))
    
    def get_waypoints(self):
        start = Pose2D()
        try:
            trans = self.tfBuffer.lookup_transform(self.reference_frame, self.own_frame, rclpy.time.Time().to_msg(), timeout=rclpy.time.Duration(seconds=3.0))
            frame = tf2_kdl.transform_to_kdl(trans)
            start.theta = frame.M.GetRPY()[2]
            start.x = frame.p.x()
            start.y = frame.p.y()
            #self.get_logger().debug(f"got the following for ego position: {start}")
        except Exception as e: 
            self.get_logger().info(f"Exception in tf transformations\n{e}")
            
        goal = Pose2D()
        goal.x = self.goal.pose.position.x
        goal.y = self.goal.pose.position.y
        goal.theta = yaw_from_orientation(self.goal.pose.orientation)
        
        return [start, goal]    

    def convert_map_data_to_image(self, map_data):
        image = np.array(map_data.data, dtype=int)
        image = np.reshape(image, (map_data.info.width, map_data.info.height))
        image[image < 50] = 0
        image[image >= 50] = 100
        return image

def yaw_from_orientation(orientation):
    rot = PyKDL.Rotation.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
    return rot.GetRPY()[2]

def yaw_to_orientation(yaw):
    q = PyKDL.Rotation.RPY(0,0,yaw).GetQuaternion()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    

def main():
    rclpy.init()
    node = PSOPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting Down')
        node.destroy_node()

if __name__ == '__main__':
    main()