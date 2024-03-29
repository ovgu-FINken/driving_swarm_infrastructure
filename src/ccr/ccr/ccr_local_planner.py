from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Int32, Int32MultiArray
from std_srvs.srv import Empty
from driving_swarm_messages.srv import SaveToFile, UpdateTrajectory
from sensor_msgs.msg import LaserScan
from trajectory_generator.vehicle_model_node import TrajectoryGenerator, Vehicle
import numpy as np
import rclpy
from shapely import Polygon, simplify, LineString
from shapely import Point as ShapelyPoint
import shapely
from termcolor import colored


class CCRLocalPlanner(DrivingSwarmNode):
    """This node will execute the local planner for the CCR. It will use the map or a given graph file to generate a roadmap and convert local coordinates to graph nodes.
    The local planner will publish the next waypoint, current node and the graph to the global planner, which in turn is able to generate a discrete plan by uing CCR.
    This node (the local planner) is then able to compute waypoints using a vehicle model in the feasible region of the workspace and generate waypoints for execution.    
    """

    def __init__(self, name):
        super().__init__(name)
        self.setup_command_interface(autorun=False)
        self.get_frames()
        self.setup_tf()

        # parse a map file to generate navigation graph
        self.declare_parameter('graph_file', 'graph.yaml')
        self.declare_parameter('x_min', -2.0)
        self.declare_parameter('x_max', 3.0)
        self.declare_parameter('y_min', -2.0)
        self.declare_parameter('y_max', 1.0)
        map_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.get_logger().info(f'loading graph from {map_file}')
        self.cutoff_amount = 2
        
        self.declare_parameter('robot_names', ['invalid_name'])
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        
        
        wx = (self.get_parameter('x_min').get_parameter_value().double_value, self.get_parameter('x_max').get_parameter_value().double_value)
        wy = (self.get_parameter('y_min').get_parameter_value().double_value, self.get_parameter('y_max').get_parameter_value().double_value)
        self.declare_parameter('grid_type', 'square')
        tiling = self.get_parameter('grid_type').get_parameter_value().string_value
        self.declare_parameter('grid_size', .5)
        grid_size = self.get_parameter('grid_size').get_parameter_value().double_value
        self.declare_parameter('inflation_size', 0.2)
        self.declare_parameter('laser_inflation_size', 0.2)
        self.laser_inflation_size = self.get_parameter('laser_inflation_size').get_parameter_value().double_value

        self.wall = None
        self.initial_pos = None
        self.state = None 
        self.last_state = None
        self.goal = None
        self.plan = None
        self.scan_poly = None
        self.path_poly = None
        self.path_poly2 = None
        self.trajectory = None
        self.stop_when_done = False
        points = None 
        self.poly_pub = self.create_publisher(MarkerArray, '/cells', 10)
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
        
        self.declare_parameter("vehicle_model", int(Vehicle.RTR))
        self.declare_parameter("step_size", 0.1)
        self.declare_parameter("turn_radius", .2)
        self.declare_parameter("turn_speed", 0.2)
        # set up vehicle model with parameters
        self.vm = TrajectoryGenerator(
            model=Vehicle(
                self.get_parameter("vehicle_model")
                .get_parameter_value()
                .integer_value
            ),
            step=self.get_parameter("step_size").get_parameter_value().double_value,
            r=self.get_parameter("turn_radius")
            .get_parameter_value()
            .double_value,
            r_step=self.get_parameter("turn_speed").get_parameter_value().double_value,
        )

        # set up publishers and subscribers
        self.create_service(SaveToFile, 'save_graph', self.save_graph)
        #self.create_subscription(PoseStamped, "nav/goal", self.goal_cb, 10)
        
        self.create_subscription(Path, "nav/trajectory", self.trajectory_cb, 10)
        self.create_service(Empty, "nav/replan", self.replan_callback)
        #self.goal_pub = self.create_publisher(Int32, "nav/goal_node", 10)
        self.state_pub = self.create_publisher(Int32, "nav/current_node", 10)
        self.plan_sub = self.create_subscription(Int32MultiArray, "nav/plan", self.plan_cb, 10)
        self.follow_client = self.create_client(
            UpdateTrajectory, "nav/follow_trajectory"
        )

        self.get_logger().info(f"graph generated {map_file}")
        self.wait_for_tf()
        self.create_subscription(LaserScan, 'scan', self.scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.follow_client.wait_for_service()
        self.get_logger().info("connected to trajectory follower service")
        self.initial_pos = self.get_tf_pose()
        self.create_timer(1.0, self.timer_cb)
        self.set_state_ready()

    def graph_to_marker_array(self):
        poly_msg = MarkerArray()
        for i, v in self.env.g.nodes(data=True):
            marker = Marker(action=Marker.ADD, ns="inner", id=i, type=Marker.LINE_STRIP)
            marker.header.frame_id = 'map'
            marker.scale.x = 0.02
            coords = v['geometry'].inner.exterior.coords
            marker.points = [Point(x=point[0], y=point[1], z=0.0) for point in coords]
            marker.colors = [ColorRGBA(r=0.5, g=0.8, b=0.5, a=0.3) for _ in coords]
            poly_msg.markers.append(marker)
            marker = Marker(action=Marker.ADD, ns="label", id=i, type=Marker.TEXT_VIEW_FACING)
            marker.header.frame_id = 'map'
            coords = v['geometry'].center.coords
            marker.pose.position = Point(x=coords[0][0], y=coords[0][1], z=0.0)
            marker.text = str(i)
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            poly_msg.markers.append(marker)

        return poly_msg

    def publish_polygon_marker(self, polygon, ns="feasible", id=1, color=None):
        if color is None:
            color = ColorRGBA(r=0.5, g=1.0, b=0.5, a=0.4)
        poly_msg = MarkerArray()
        if polygon.geom_type == 'MultiPolygon':
            self.get_logger().warn(f'got multi polygon for "{ns}", will not show all components')
            polygon = polygon.geoms[0]
        marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.LINE_STRIP)
        marker.header.frame_id = 'map'
        marker.scale.x = 0.01
        coords = polygon.exterior.coords
        marker.points = [Point(x=point[0], y=point[1], z=0.0) for point in coords]
        marker.colors = [color for _ in coords]
        poly_msg.markers.append(marker)
        return poly_msg
    
    def publish_line_marker(self, line, ns="path", id=1, color=None):
        if color is None:
            color = ColorRGBA(r=0.5, g=1.0, b=0.5, a=0.4)
        line_msg = MarkerArray()
        marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.LINE_STRIP)
        marker.header.frame_id = 'map'
        marker.scale.x = 0.01
        coords = line.coords
        marker.points = [Point(x=point[0], y=point[1], z=0.0) for point in coords]
        marker.colors = [color for _ in coords]
        line_msg.markers.append(marker)
        return line_msg

    def replan_callback(self, _, res):
        if not self.plan:
            return
        self.get_logger().info('got replanning request, which we will do')
        self.execute_plan(use_cutoff=False)
        return res
        
    def save_graph(self, req, res):
        self.g.save(req.filename)
        return res
    

    def publish_state(self):
        tf_pose = self.get_tf_pose()
        if tf_pose is None:
            return
        state = self.env.find_nearest_node(tf_pose[:2])
        self.state_pub.publish(Int32(data=int(state))) 
        if state != self.state:
            self.last_state = self.state
            self.state = state
            self.get_logger().info(f'new state {self.state}')
            
    def timer_cb(self):
        # functional
        # do not publish state and goal before all planners are ready
        if self._command not in ["go", "reset"]:
            return
        if self._command == "reset":
            self.goal = self.initial_pos
            self.get_logger().info(f'Reset activated, going back to {self.goal}', once=True)
            self.set_state("resetting")
        # self.publish_goal()
        self.publish_state()
        if self.plan:
            self.execute_plan(use_cutoff=False)
            
        # debug information
        self.poly_pub.publish(self.graph_to_marker_array())
        if self.path_poly is not None:
            self.poly_pub.publish(self.publish_polygon_marker(self.path_poly,ns="{}_feasible".format(self.robot_name), color=ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.4)))
        if self.path_poly2 is not None and False:
            self.poly_pub.publish(self.publish_polygon_marker(self.path_poly2,ns="{}_feasible2".format(self.robot_name)))
        # self.poly_pub.publish(self.publish_polygon_marker(self.scan_poly, ns=f"{self.robot_name}_scan"))
        if self.plan and len(self.plan) > 1:
            plan = [self.env.g.nodes()[i]['geometry'].center for i in self.plan]
            self.poly_pub.publish(self.publish_line_marker(LineString(plan), ns=f"{self.robot_name}_plan"))
        if self.goal:
            goal_line = LineString([self.get_tf_pose()[:2], self.goal[:2]])
            self.poly_pub.publish(self.publish_line_marker(goal_line, ns=f"{self.robot_name}_goal", color=ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.1)))
        if self.trajectory and len(self.trajectory.poses) > 1:
            trajectory_line = LineString([self.pose_stamped_to_tuple(p)[:2] for p in self.trajectory.poses])
            self.poly_pub.publish(self.publish_line_marker(trajectory_line, ns=f"{self.robot_name}_trajectory", color=ColorRGBA(r=0.0, g=.0, b=0.3, a=0.4)))
    
    def plan_cb(self, msg):
        if list(msg.data) == self.plan:
            return
        plan = list(msg.data)
        if self.plan is None and plan:
            self.set_state_running()
        # if the next node in the plan changes, we do not use the cutoff
        # self.get_logger().info(f'got new plan {plan}')
        full_plan = plan
        plan = self.compute_unblocked_path(plan)
            
        # self.get_logger().info(f'unblocked is {plan}')
        for n1, n2 in zip(plan[:-1], plan[1:]):
            if (n1, n2) not in self.env.g.edges():
                self.get_logger().warn(f'edge ({n1}, {n2}) is not in graph')
        if self.plan == plan:
            return
        
        if len(plan) < len(full_plan):
            self.get_logger().info(f'new plan: {plan}, remainder: {full_plan[len(plan):]}')
        else:
            self.get_logger().info(f'new plan: {plan}')

        use_cutoff = False
        if self.plan and len(self.plan) > 1 and len(plan) > 2:
            # plan starts with wait action
            if plan[0] == plan[1]:
                use_cutoff = False
            # plan update is because robot drove to next node in plan
            elif self.plan[1] == plan[0]:
                # use the cutoff point, if the next node stays the same
                # we do not need cutoff, because we go into the transition area
                use_cutoff = False
            elif self.plan[0] == plan[0]:
                # if the first next node stays the same, we do not need to use the cutoff
                use_cutoff = self.plan[1] == plan[1]
                
        self.plan = plan
        if len(self.plan) == 1 and self.stop_when_done:
            self.set_state("done")
            return
        self.execute_plan(use_cutoff=use_cutoff)
    
    def compute_unblocked_path(self, plan):
        visited = set()
        out = []
        for node in plan:
            if node not in visited:
                visited.add(node)
                out.append(node)
            else:
                break
        return out[:3]
        
    def execute_plan(self, use_cutoff=True):
        # if there is a wait action within the plan, only execute the plan up to the wait action
        # self.get_logger().info(f'executing plan {self.plan}')
        
        # include last state, so transition area is within feasible region, while the robot is still with in the transition area
        # compute feasible area
        if self.plan is None:
            self.get_logger().warn(colored('plan is None', 'red'))
            return
        if len(self.plan) < 1:
            self.get_logger().warn(colored('plan is empty', 'red'))
            return
        previous = None
        if self.plan[0] == self.state:
            previous = self.last_state
        if self.plan[0] != self.state:
            previous = self.state
        if (previous, self.plan[0]) not in self.env.g.edges():
            previous = None
        self.path_poly = geometry.poly_from_path(self.env.g, self.plan, eps=0.01, previous_node=previous)
        if not self.path_poly.is_valid:
            #self.get_logger().warn(colored('path_poly is not valid', 'red'))
            #self.get_logger().info(f'plan is: {self.plan}')
            #self.get_logger().info(f'path_poly is: {self.path_poly}')
            buffer_distance = 0.001
            buffer_poly = self.path_poly.buffer(buffer_distance)
            # Create a new polygon from the buffer
            self.path_poly = Polygon(buffer_poly.exterior.coords)
    
        # assert self.path_poly.geom_type == 'Polygon' 
        # assert self.path_poly.is_valid
        # assert not self.path_poly.is_empty, f'path_poly is empty, plan is {plan}'
        # self.path_poly2 = self.path_poly
        self.path_poly = shapely.intersection(self.path_poly, self.scan_poly)
        self.path_poly = simplify(self.path_poly, 0.01)
        self.path_poly = self.resolve_multi_polygon(self.path_poly)

        if self.path_poly is None or self.path_poly.is_empty:
            self.get_logger().warn(colored('feasible_area is empty, will not send path', 'red'))
            self.get_logger().info(f'plan is: {self.plan}')
            self.send_path([], ti=0)
            return

        # print warning if robot is not in feasible area
        tf_pose = self.get_tf_pose()
        if tf_pose is not None:
            position = ShapelyPoint(tf_pose[:2])
            if not self.path_poly.contains(position):
                distance = self.path_poly.distance(position)
                if distance > 0.1:
                    self.get_logger().warn(colored(f'robot is not in feasible area, d={distance:.2f}m', 'red'))
        
        # compute cutoff point
        start = None
        cutoff = None
        if self.trajectory is None or not len(self.trajectory.poses):
            use_cutoff = None
        if use_cutoff:
            now_index = self.get_now_index()
            cutoff = self.get_cutoff_point()
            traj = self.trajectory.poses[now_index:cutoff]
            traj = [self.pose_stamped_to_tuple(p)[:2] for p in traj]
            dist = max([self.path_poly.distance(ShapelyPoint(p)) for p in traj])
            if dist > 0.05:
                cutoff = None
        
        if cutoff is not None:
            s = self.trajectory.poses[cutoff]
            start = self.pose_stamped_to_tuple(s)
        else:
            start = self.get_tf_pose()
        end = self.env.g.nodes()[self.plan[-1]]['geometry'].center

        result_path = geometry.find_shortest_path(self.path_poly, start, end, eps=0.01, goal_outside_feasible=False)
        if len(result_path) < 2:
            self.get_logger().warn(f'len(result_path) < 2, will not send path, len(result_path)={len(result_path)}')
        wps = [start] + [(i.x,i.y,np.nan) for i in result_path]
        trajectory = self.vm.tuples_to_path(wps)
        if not len(trajectory):
            self.get_logger().warn('trajectory is empty')
            
        if len(trajectory) == 1 and len(self.plan) > 1:
            self.get_logger().warn('trajectory has only one point')
            self.get_logger().info(f'path: {self.plan}')
            self.get_logger().info(f'wps: {wps}')
            self.get_logger().info(f'result_path: {result_path}')
        self.send_path(trajectory, ti=cutoff)

    def send_path(self, trajectory, ti=0):
        if not self._state in ["running", "resetting"]:
            return
        # convert trajectory to correct space
        if not len(trajectory):
            if len(self.plan) > 1:
                self.get_logger().info(colored('sending empty trajectory', 'red'))
            tf_pose = self.get_tf_pose()
            if tf_pose is None:
                return
            trajectory = [tf_pose]
            ti = 0
        if len(trajectory) == 1 and len(self.plan) > 1:
            self.get_logger().info(colored('sending single point trajectory', 'red'))

        if ti is None:
            ti = 0
        path = Path()
        path.header.frame_id = self.reference_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in trajectory:
            path.poses.append(self.tuple_to_pose_stamped_msg(*pose))

        # self.get_logger().info(f"sending path with {len(path.poses)}s")
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=int(ti))
        self.follow_client.call_async(request)
        
    def scan_cb(self, msg):
        # convert laser scan to polygon
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        
        tf_pose = self.get_tf_pose()
        if tf_pose is None:
            return
        
        px, py, pt = tf_pose
        # convert scan ranges to xy coordinates
        points = []
        for i, r in enumerate(r):
            angle = msg.angle_min + i * msg.angle_increment + pt
            x = r * np.cos(angle) + px
            y = r * np.sin(angle) + py
            points.append((x,y))

        self.scan_poly = Polygon(points).buffer(-self.laser_inflation_size)
        self.scan_poly = self.resolve_multi_polygon(self.scan_poly)
        
    def trajectory_cb(self, msg):
        self.trajectory = msg
        # if we do have a trajectory, but do not have a plan, we should stop (this should not happen)
        if not len(self.plan):
            self.get_logger().info("no plan, stopping trajectory")
            self.send_path([], ti=0)
            return
        
        if not len(self.trajectory.poses) > 1:
            if len(self.plan) > 1:
                self.get_logger().info(f"empty trajectory for plan {self.plan}")
                # self.execute_plan(use_cutoff=False)
            return
    
    def get_now_index(self):
        if self.trajectory is None:
            return None
        trajectory_time = self.trajectory.header.stamp.sec + self.trajectory.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().to_msg()
        now_time = now.sec + now.nanosec * 1e-9
        now_index = int(np.floor((now_time - trajectory_time) * 10))
        if now_index < 0:
            return None

    def get_cutoff_point(self):
        if self.trajectory is None:
            return None
        now_index = self.get_now_index()
        if now_index is None:
            return None
        if len(self.trajectory.poses) <= now_index + self.cutoff_amount:
            return None
        #self.get_logger().info(f'start: {trajectory_time}, now: {now_time}, now_index: {now_index}')
        return now_index + self.cutoff_amount

    def resolve_multi_polygon(self, mp):
        if mp is None:
            return None
        
        position = self.get_tf_pose()
        if position is None:
            return None
        robot_point = ShapelyPoint(position[0], position[1])

        if mp.geom_type != 'MultiPolygon':
            return mp
        
        poly = [poly for poly in mp.geoms if not poly.is_empty and poly.is_valid]


        def score_polygon(poly):
            return robot_point.distance(poly)

        target_polygon = min(poly, key=score_polygon)
        assert target_polygon.geom_type == 'Polygon'
        return target_polygon



def main():
    main_fn('ccr_local_planner', CCRLocalPlanner)


if __name__ == '__main__':
    main()