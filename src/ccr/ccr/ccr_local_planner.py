from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from polygonal_roadmaps import geometry, environment

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Int32, Int32MultiArray, String
from std_srvs.srv import Empty
from driving_swarm_messages.srv import SaveToFile, UpdateTrajectory
from sensor_msgs.msg import LaserScan
from trajectory_generator.vehicle_model_node import TrajectoryGenerator, Vehicle
import numpy as np
import rclpy
from shapely import Polygon, simplify, LineString
from shapely import Point as ShapelyPoint
from math import atan2, degrees
import shapely


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
        
        self.declare_parameter('robot_names', ['invalid_name'])
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        
        
        wx = (self.get_parameter('x_min').get_parameter_value().double_value, self.get_parameter('x_max').get_parameter_value().double_value)
        wy = (self.get_parameter('y_min').get_parameter_value().double_value, self.get_parameter('y_max').get_parameter_value().double_value)
        self.declare_parameter('grid_type', 'square')
        self.declare_parameter('grid_size', .5)
        self.declare_parameter('inflation_size', 0.2)
        self.declare_parameter('laser_inflation_size', 0.2)
        self.laser_inflation_size = self.get_parameter('laser_inflation_size').get_parameter_value().double_value

        self.wall = None
        self.initial_pos = None
        self.allow_goal_publish = True
        self.state = None 
        self.last_state = None
        self.goal = None
        self.plan = None
        self.path_poly = None
        self.path_poly2 = None
        self.trajectory = None
        points = None 
        self.poly_pub = self.create_publisher(MarkerArray, '/cells', 10)
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
        
        self.declare_parameter("vehicle_model", int(Vehicle.RTR))
        self.declare_parameter("step_size", 0.15)
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
        self.create_subscription(PoseStamped, "nav/goal", self.goal_cb, 1)
        
        self.create_subscription(Path, "nav/trajectory", self.trajectory_cb, 1)
        self.create_service(Empty, "nav/replan", self.replan_callback)
        self.goal_pub = self.create_publisher(Int32, "nav/goal_node", 1)
        self.wall_pub = self.create_publisher(String, "nav/wall", 1)
        self.state_pub = self.create_publisher(Int32, "nav/current_node", 1)
        self.plan_sub = self.create_subscription(Int32MultiArray, "nav/plan", self.plan_cb, 1)
        self.follow_client = self.create_client(
            UpdateTrajectory, "nav/follow_trajectory"
        )

        self.get_logger().info(f"graph generated {map_file}")
        self.wait_for_tf()
        self.create_subscription(LaserScan, 'scan', self.scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.wall_cb, rclpy.qos.qos_profile_sensor_data)
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

        idx = 0
        for i, j, e in self.env.g.edges(data=True):
            if e['geometry'].borderPoly is None:
                continue
            if  e['geometry'].borderPoly.geom_type != 'Polygon':
                continue
            idx += 1
            marker = Marker(action=Marker.ADD, ns="transition", id=idx, type=Marker.LINE_STRIP)
            marker.header.frame_id = 'map'
            marker.scale.x = 0.01
            coords = e['geometry'].borderPoly.exterior.coords
            marker.points = [Point(x=point[0], y=point[1], z=0.0) for point in coords]
            marker.colors = [ColorRGBA(r=0.5, g=0.5, b=0.8, a=0.3) for _ in coords]
            poly_msg.markers.append(marker)
            idx += 1
            marker = Marker(action=Marker.ADD, ns="edge", id=idx, type=Marker.LINE_STRIP)
            marker.header.frame_id = 'map'
            marker.scale.x = 0.01
            coords = e['geometry'].connection.coords
            marker.points = [Point(x=point[0], y=point[1], z=0.0) for point in coords]
            marker.colors = [ColorRGBA(r=0.5, g=0.3, b=0.3, a=0.3) for _ in coords]
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
    
    def publish_goal(self):
        if self.goal is None:
            return
        goal = self.env.find_nearest_node(self.goal[:2])
        #self.get_logger().info(str(self.initial_pos)+" "+str(self.goal))
        self.goal_pub.publish(Int32(data=int(goal)))

    def publish_state(self):
        state = self.env.find_nearest_node(self.get_tf_pose()[:2])
        self.state_pub.publish(Int32(data=int(state))) 
        if state != self.state:
            self.last_state = self.state
            self.state = state
            self.get_logger().info(f'new state {self.state}')
            
    def timer_cb(self):
        # functional
        if self._command == "reset":
            self.allow_goal_publish = False
            self.goal = self.initial_pos
            self.get_logger().info(f'Reset activated, going back to {self.goal}', once=True)
        self.publish_goal()
        self.publish_state()
        if self.plan:
            self.execute_plan()
            
        # debug information
        self.poly_pub.publish(self.graph_to_marker_array())
        if self.path_poly is not None:
            self.poly_pub.publish(self.publish_polygon_marker(self.path_poly,ns="{}_feasible".format(self.robot_name), color=ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.4)))
        if self.path_poly2 is not None and False:
            self.poly_pub.publish(self.publish_polygon_marker(self.path_poly2,ns="{}_feasible2".format(self.robot_name)))
        # self.poly_pub.publish(self.publish_polygon_marker(self.scan_poly, ns=f"{self.robot_name}_scan"))
        if self.plan:
            plan = [self.env.g.nodes()[i]['geometry'].center for i in self.plan]
            self.poly_pub.publish(self.publish_line_marker(LineString(plan), ns=f"{self.robot_name}_plan"))
        if self.goal_cb:
            goal_line = LineString([self.get_tf_pose()[:2], self.goal[:2]])
            self.poly_pub.publish(self.publish_line_marker(goal_line, ns=f"{self.robot_name}_goal", color=ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.4)))
    
    def goal_cb(self, msg):
        if self.allow_goal_publish:
            goal = self.pose_stamped_to_tuple(msg)
            if self.goal != goal:
                self.goal = goal
                self.get_logger().info(f'new goal {self.goal}')
                self.publish_goal()

    def plan_cb(self, msg):
        if list(msg.data) == self.plan:
            return
        plan = list(msg.data)
        if self.plan is None and plan:
            self.set_state_running()
        # if the next node in the plan changes, we do not use the cutoff
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
        if len(self.plan) == 1 and not self.allow_goal_publish:
            self.set_state("done")
        self.execute_plan(use_cutoff=use_cutoff)
        
    def execute_plan(self, use_cutoff=True):
        # if there is a wait action within the plan, only execute the plan up to the wait action
        visited = set()
        plan = []
        for node in self.plan:
            if node not in visited:
                visited.add(node)
                plan.append(node)
            else:
                break
        if not len(plan):
            self.get_logger().info('empty plan')
            self.send_path([], ti=0)
            return
        remainder = self.plan[len(plan):]
        self.get_logger().debug(f'executing plan {self.plan}')
        if remainder:
            self.get_logger().debug(f'\t remaining plan after wait actions {remainder}')
        
        start = None
        cutoff = None
        if use_cutoff:
            cutoff = self.get_cutoff_point()
        if cutoff is not None:
            s = self.trajectory.poses[cutoff]
            start = self.pose_stamped_to_tuple(s)
        else:
            start = self.get_tf_pose()
        end = self.env.g.nodes()[plan[-1]]['geometry'].center
        # include last state, so transition area is within feasible region, while the robot is still with in the transition area

        previous = None
        if self.plan[0] == self.state:
            previous = self.last_state
        if self.plan[0] != self.state:
            previous = self.state
        if (previous, plan[0]) not in self.env.g.edges():
            previous = None
        self.path_poly = geometry.poly_from_path(self.env.g, self.plan, eps=0.01, previous_node=previous)
        # assert self.path_poly.geom_type == 'Polygon' 
        # assert self.path_poly.is_valid
        # assert not self.path_poly.is_empty, f'path_poly is empty, plan is {plan}'
        self.path_poly2 = self.path_poly
        self.path_poly = shapely.intersection(self.path_poly, self.scan_poly)
        self.path_poly = simplify(self.path_poly, 0.01)
        self.path_poly = self.resolve_multi_polygon(self.path_poly)
        if self.path_poly.is_empty:
            self.get_logger().warn('feasible_area is empty, will not send path')
            self.get_logger().info(f'plan is: {plan}')
            self.send_path([], ti=0)
            return
        result_path = geometry.find_shortest_path(self.path_poly, start, end, eps=0.01, goal_outside_feasible=False)
        wps = [start]
        wp = [(i.x,i.y,np.nan) for i in result_path]
        wps += wp
        trajectory = self.vm.tuples_to_path(wps)
        if not len(trajectory):
            self.get_logger().warn('trajectory is empty')
        self.send_path(trajectory, ti=cutoff)

    def send_path(self, trajectory, ti=0):
        # convert trajectory to correct space
        if not len(trajectory):
            trajectory = [self.get_tf_pose()]
            ti = 0
        if ti is None:
            ti = 0
        path = Path()
        path.header.frame_id = self.reference_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in trajectory:
            path.poses.append(self.tuple_to_pose_stamped_msg(*pose))

        self.get_logger().debug("sending path")
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=int(ti))
        self.follow_client.call_async(request)
        
    def scan_cb(self, msg):
        # convert laser scan to polygon
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        
        px, py, pt = self.get_tf_pose()
        # convert scan ranges to xy coordinates
        points = []
        for i, r in enumerate(r):
            angle = msg.angle_min + i * msg.angle_increment + pt
            x = r * np.cos(angle) + px
            y = r * np.sin(angle) + py
            points.append((x,y))

        self.scan_poly = Polygon(points).buffer(-self.laser_inflation_size)
        self.scan_poly = self.resolve_multi_polygon(self.scan_poly)
        
    def wall_cb(self, msg):
        r = msg.ranges
        r = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in r]
        if r[80] < 0.3 and r[100] < 0.3:
            self.wall ="Left"
        elif r[260] < 0.3 and r[280] < 0.3:
            self.wall ="Right"
        else:
            self.wall =None
        self.wall_pub.publish(String(data=str(self.wall)))
    
    def trajectory_cb(self, msg):
        self.trajectory = msg
        # if we do have a trajectory, but do not have a plan, we should stop (this should not happen)
        if not len(self.plan):
            self.get_logger().info("no plan, stopping trajectory")
            self.send_path([], ti=0)
            return
        
        if not len(self.trajectory.poses) > 1:
            self.get_logger().info("empty trajectory, stopping replan and send new trajectory")
            self.get_logger().info(f"plan is: {self.plan}")
            self.execute_plan(use_cutoff=False)
            return

        # TODO check if the trajectory is valid with respect to our plan
        # in case the cut-off point was set wrongly, or some other problem happens, we could end up with a trajectory that is not valid
        # in that case, we will send a new path
        
    def get_cutoff_point(self):
        if self.trajectory is None:
            return None
        trajectory_time = self.trajectory.header.stamp.sec + self.trajectory.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().to_msg()
        now_time = now.sec + now.nanosec * 1e-9
        now_index = int(np.floor((now_time - trajectory_time) * 10))
        if now_index < 0:
            return None
        if len(self.trajectory.poses) <= now_index + 10:
            return None
        #self.get_logger().info(f'start: {trajectory_time}, now: {now_time}, now_index: {now_index}')
        return now_index + int(10 * 0.4)

    def resolve_multi_polygon(self, mp):
        
        def angle_between_points(p1, p2):
            return degrees(atan2(p2[1] - p1[1], p2[0] - p1[0]))

        def normalize(value, max_value, min_value):
            return (value - min_value) / (max_value - min_value)
        
        position = self.get_tf_pose()
        robot_point = ShapelyPoint(position[0], position[1])
        robot_orientation = position[2]

        if mp.geom_type != 'MultiPolygon':
            return mp
        
        poly = [poly for poly in mp.geoms if not poly.is_empty and poly.is_valid]

        distances = [p.distance(robot_point) for p in poly]
        max_distance = max(distances)
        min_distance = min(distances)

        def score_polygon(poly):
            return robot_point.distance(poly)
            centroid = poly.centroid
            angle_to_polygon = angle_between_points((robot_point.x, robot_point.y), (centroid.x, centroid.y))
            angle_difference = abs(robot_orientation - angle_to_polygon)

            normalized_distance = normalize(poly.distance(robot_point), max_distance, min_distance)
            normalized_angle_difference = normalize(angle_difference, 180, 0)  # angles can vary between 0 and 180 degrees
            self.get_logger().info(f"angle: {angle_difference}, distance: {poly.distance(robot_point)}")
            self.get_logger().info(f"angle: {normalized_angle_difference}, distance: {normalized_distance}")

            # Define weights
            distance_weight = 0.5
            angle_weight = 0.5

            return distance_weight * normalized_distance + angle_weight * normalized_angle_difference

        target_polygon = min(poly, key=score_polygon)
        assert target_polygon.geom_type == 'Polygon'
        return target_polygon



def main():
    main_fn('ccr_local_planner', CCRLocalPlanner)


if __name__ == '__main__':
    main()