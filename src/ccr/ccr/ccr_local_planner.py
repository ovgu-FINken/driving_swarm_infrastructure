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
from shapely import Polygon
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
        
        wx = (self.get_parameter('x_min').get_parameter_value().double_value, self.get_parameter('x_max').get_parameter_value().double_value)
        wy = (self.get_parameter('y_min').get_parameter_value().double_value, self.get_parameter('y_max').get_parameter_value().double_value)
        self.declare_parameter('grid_type', 'square')
        self.declare_parameter('grid_size', .5)
        self.declare_parameter('inflation_size', 0.2)

        self.state = None 
        self.goal = None
        self.plan = None
        self.path_poly = None
        points = None 
        self.poly_pub = self.create_publisher(MarkerArray, 'cells', 10)
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
        self.declare_parameter("step_size", 0.1)
        self.declare_parameter("turn_radius", 0.1)
        self.declare_parameter("turn_speed",0.1)
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
        self.create_service(Empty, "nav/replan", self.replan_callback)
        self.goal_pub = self.create_publisher(Int32, "nav/goal_node", 1)
        self.state_pub = self.create_publisher(Int32, "nav/current_node", 1)
        self.plan_sub = self.create_subscription(Int32MultiArray, "nav/plan", self.plan_cb, 1)
        self.follow_client = self.create_client(
            UpdateTrajectory, "nav/follow_trajectory"
        )

        self.get_logger().info(f"graph generated {map_file}")
        self.wait_for_tf()
        self.create_subscription(LaserScan, 'scan', self.scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.follow_client.wait_for_service()
        self.get_logger().info("connected to trajectory follower service")
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
            if  e['geometry'].borderPoly.geometryType() != 'Polygon':
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

    def publish_polygon_marker(self, polygon, ns="feasible", id=0):
        poly_msg = MarkerArray()
        marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.LINE_STRIP)
        marker.header.frame_id = 'map'
        marker.scale.x = 0.01
        coords = polygon.exterior.coords

        marker.points = [Point(x=point[0], y=point[1], z=0.0) for point in coords]
        marker.colors = [ColorRGBA(r=0.5, g=0.3, b=0.3, a=0.3) for _ in coords]
        poly_msg.markers.append(marker)
        return poly_msg

    def replan_callback(self, _, res):
        if not self.plan:
            return
        self.get_logger().info('got replanning request, which we will do')
        self.execute_plan()
        return res
        
    def save_graph(self, req, res):
        self.g.save(req.filename)
        return res
    
    def publish_goal(self):
        if self.goal is None:
            return
        goal = self.env.find_nearest_node(self.goal[:2])
        self.goal_pub.publish(Int32(data=int(goal)))

    def publish_state(self):
        state = self.env.find_nearest_node(self.get_tf_pose()[:2])
        self.state_pub.publish(Int32(data=int(state)))  

    def timer_cb(self):
        self.publish_goal()
        self.publish_state()
        self.poly_pub.publish(self.graph_to_marker_array())
        try:
            self.poly_pub.publish(self.publish_polygon_marker(self.path_poly))
        except Exception:
            pass
        self.poly_pub.publish(self.publish_polygon_marker(self.scan_poly, ns="scan", id=0))

    def goal_cb(self, msg):
        goal = self.pose_stamped_to_tuple(msg)
        if self.goal != goal:
            self.goal = goal
            self.get_logger().info(f'new goal {self.goal}')
            self.publish_goal()

    def plan_cb(self, msg):
        if list(msg.data) == self.plan:
            return
        self.plan = list(msg.data)
        self.get_logger().info(f'new plan {self.plan}')
            
        self.execute_plan()
        
    def execute_plan(self):
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
            return
        
        remainder = self.plan[len(plan):]
        self.get_logger().info(f'executing plan {self.plan}')
        if remainder:
            self.get_logger().info(f'\t remaining plan after wait actions {remainder}')
        
        start = self.get_tf_pose()
        end = self.env.g.nodes()[plan[-1]]['geometry'].center
        self.path_poly = geometry.poly_from_path(self.env.g, self.plan, eps=0.01)
        self.path_poly = shapely.intersection(self.path_poly,self.scan_poly)
        result_path = geometry.find_shortest_path(self.path_poly, start, end, eps=0.01)
        wps = [self.get_tf_pose()]
        wp = [(i.x,i.y,np.nan) for i in result_path]
        wps += wp
        trajectory = self.vm.tuples_to_path(wps)
        self.send_path(trajectory)

    def send_path(self, trajectory, ti=0):
        # convert trajectory to correct space
        if trajectory is None:
            return
        path = Path()
        path.header.frame_id = self.reference_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in trajectory:
            path.poses.append(self.tuple_to_pose_stamped_msg(*pose))

        self.get_logger().info("sending path")
        if ti == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path, update_index=ti)
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

        self.scan_poly = Polygon(points).buffer(-0.1)
        

def main():
    main_fn('ccr_local_planner', CCRLocalPlanner)


if __name__ == '__main__':
    main()