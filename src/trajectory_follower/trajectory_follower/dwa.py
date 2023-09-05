
import rclpy
import tf_transformations

# modules need to be imported as plugins for tf2
import tf2_ros
import tf2_py # noqa F401
import tf2_geometry_msgs # noqa F401
import numpy as np
from std_msgs.msg import ColorRGBA, Int32, Int32MultiArray, String
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, Quaternion
from geometry_msgs.msg import Point as PointMsg
from nav_msgs.msg import Path
from driving_swarm_messages.srv import UpdateTrajectory
from std_srvs.srv import Empty
from driving_swarm_utils.node import DrivingSwarmNode
from termcolor import colored
from shapely import Polygon, Point, LineString, union_all
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.duration import Duration


class TrajectoryFollower(DrivingSwarmNode):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.get_frames()
        self.declare_parameter("dt", 1.0)
        self.dt = self.get_parameter("dt").get_parameter_value().double_value
        self.declare_parameter("w1", 1.0)
        self.w1 = self.get_parameter("w1").get_parameter_value().double_value
        self.declare_parameter("w2", 1.0)
        self.w2 = self.get_parameter("w2").get_parameter_value().double_value
        self.declare_parameter("w3", 1.0)
        self.w3 = self.get_parameter("w3").get_parameter_value().double_value
        self.declare_parameter("w4", 1.0)
        self.w4 = self.get_parameter("w4").get_parameter_value().double_value
        self.declare_parameter("fail_radius", 1.0)
        self.fail_radius = self.get_parameter("fail_radius") \
                               .get_parameter_value().double_value
        self.declare_parameter("n_samples_linear", 7)
        self.n_samples_linear = self.get_parameter("n_samples_linear").get_parameter_value().integer_value
        self.declare_parameter("n_samples_angular", 7)
        self.n_samples_angular = self.get_parameter("n_samples_angular").get_parameter_value().integer_value
        self.declare_parameter("laser_inflation_size", 0.15)
        self.laser_inflation_size = self.get_parameter("laser_inflation_size").get_parameter_value().double_value
        self.declare_parameter("obstacle_threshold", 0.1)
        self.obstacle_threshold = self.get_parameter("obstacle_threshold").get_parameter_value().double_value
        self.declare_parameter("tb_radius", 0.23)
        self.tb_radius = self.get_parameter("tb_radius").get_parameter_value().double_value
        self.vel = 0.0
        self.rot = 0.0
        self.trajectory = None
        self.current_goal = None
        self.name = self.get_namespace()[1:]
        self.angular = None
        self.cluster_size_threshold = 8
        self.cluster_linkage_threshold = 0.2
        self.cluster_range_threshold = 1.0
        self.workspace = Polygon([
            (-10, -10),
            (-10, 10),
            (10, 10),
            (10, -10)
        ])
        

        self.setup_tf()
        self.create_subscription(LaserScan, 'scan', self.scan_cb, rclpy.qos.qos_profile_sensor_data)
    
        self.scan_poly = None
        self.cmd_vel = Twist()
        self.max_accel_x = 0.025
        self.max_accel_z = 0.075
        self.max_vel = 0.15
        self.min_vel = -0.15
        self.max_rot = 0.5
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 9)
        self.desired_pose_publisher = \
            self.create_publisher(PoseStamped, 'nav/desired', 9)
        self.co0 = self.create_publisher(PoseStamped, 'nav/cutoff_0', 9)
        self.co1 = self.create_publisher(PoseStamped, 'nav/cutoff_1', 9)
        self.path_publisher = self.create_publisher(Path, 'nav/trajectory', 9)
        self.local_pub = self.create_publisher(MarkerArray, 'nav/dwa', 9)
        self.wait_for_tf()
        self.create_service(
            UpdateTrajectory,
            'nav/follow_trajectory',
            self.update_trajectory_cb
        )
        self.replan_client = self.create_client(Empty, 'nav/replan')

        self.create_timer(0.1, self.timer_cb)
        self.create_timer(1.0, self.slow_timer_cb)

    def update_trajectory_cb(self, request, response):
        # todo: check if trajectories submitted are valid
        # self.get_logger().info(colored('updating trajectory', 'blue')+f': {request.trajectory}')
        if self.trajectory is None or request.update_index == 0:
            self.trajectory = request.trajectory
        else:
            if request.update_index > len(self.trajectory.poses):
                self.get_logger().warn(
                    'update index is larger than trajectory length'
                )
                response.accepted = False
                self.set_trajectory_fail()
                return response
            self.co0.publish(self.trajectory.poses[request.update_index - 1])
            self.co1.publish(request.trajectory.poses[0])
            self.trajectory.poses = \
                self.trajectory.poses[:request.update_index] + \
                request.trajectory.poses
        response.accepted = True
        response.trajectory = self.trajectory
        self.path_publisher.publish(self.trajectory)

        return response

    def pose2D_to_PoseStamped(self, pose2d, header_id=None):
        return self.tuple_to_pose_stamped_msg(pose2d.x, pose2d.y, pose2d.theta, frame=header_id)

    def get_target_pose(self, offset: float = 0.0) -> Pose2D:
        """ get the target poset at time t + offset in ego coordinates
        """
        return self.get_desired_pose(offset=offset)
    
    def set_trajectory_fail(self):
        self.get_logger().info(colored('replan request', 'red'))
        self.trajectory = None
        request = Empty.Request()
        self.replan_client.call_async(request)
        self.cmd_vel = Twist()
        self.cmd_publisher.publish(self.cmd_vel)

        # get the target pose at the correct time in ego_coordinates
    def alignment_error(self, _, rot, diff_pose, dt):
        # the orientation after dt * rot should be the same as the relative target pose
        alignment_error = diff_pose[2] - dt * rot

        # Calculate the absolute alignment term (smaller is better)
        alignment_term = abs(alignment_error)
        return alignment_term
    
    def position(self, vel, rot, dt):
        rot = 0.5*rot
        mat_rot = np.array([
            [np.cos(rot*dt), -np.sin(rot*dt), 0],
            [np.sin(rot*dt), np.cos(rot*dt), 0],
            [0, 0, 1],
        ])
        mat_trans = np.array([
            [0, 0, vel*dt],
            [0, 0, 0],
            [0, 0, 1]
        ])
        mat_pose = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 1.0],
        ])
        p = mat_rot @ mat_trans @ mat_pose
        return p[0,2], p[1,2]
    
    def position_error(self, vel, rot, diff_pose, dt):
        dx, dy = self.position(vel, rot, dt)
        diff = [diff_pose[0] - dx, diff_pose[1] - dy]
        return np.linalg.norm(diff) / dt
    
    def velocity_error(self, vel, rot, diff_pose, dt):
        dx, dy = self.position(vel, rot, dt)
        dist = np.linalg.norm([diff_pose[0] - dx, diff_pose[1] - dy])
        return np.abs(dist / dt - vel)
    
    def get_obstacle_distance(self, x, y):
        return min([self.occupied.distance(Point(x,y)) - self.laser_inflation_size] + [np.linalg.norm(np.array([x, y]) - np.array(p)) - self.tb_radius for p in self.tb_center_points])

    def obstacle_error(self, vel, rot, dt):
        p = [self.position(vel, rot, t) for t in [0.5*dt, dt]]
        dist = [self.get_obstacle_distance(x, y) for x, y in p]
        dist = min(dist)
        #ls = LineString(p)
        #dist = self.occupied.distance(ls)
        # should be the case anyway
        #if self.occupied.contains(ls):
        #    dist = 0

        return np.clip(self.obstacle_threshold - dist, 0.0, self.obstacle_threshold)
        
    def value(self, vel, rot, dt, diff_pose):
        return self.w1 * self.alignment_error(vel, rot, diff_pose, dt) + self.w2 * self.position_error(vel, rot, diff_pose, dt) + self.w3 * self.obstacle_error(vel, rot, dt) + self.w4 * self.velocity_error(vel, rot, diff_pose, dt)
        
    def compute_dwa_output(self, dt) -> tuple:
        if self.trajectory is None:
            return None
        
        diff_pose = self.get_target_pose(offset=dt)
        diff_pose = diff_pose.x, diff_pose.y, diff_pose.theta

        
        admissable_min = max(self.min_vel, self.vel - self.max_accel_x)
        admissable_max = min(self.max_vel, self.vel + self.max_accel_x)
        admissable_vel = np.linspace(admissable_min, admissable_max, self.n_samples_linear)
        admissable_min = max(-self.max_rot, self.rot - self.max_accel_z)
        admissable_max = min(self.max_rot, self.rot + self.max_accel_z)
        admissable_rot = np.linspace(admissable_min, admissable_max, self.n_samples_angular)
        
        admissable_cmd = [(x, y) for x in admissable_vel for y in admissable_rot]
        cmd = min(admissable_cmd, key=lambda x: self.value(*x, dt, diff_pose))
        # self.get_logger().info(f'calculate_dwa output: vel:{cmd[0]}, rot: {cmd[1]}')
        # self.get_logger().info(f'alignment:\t{self.w1 * self.alignment_error(cmd[0], cmd[1], diff_pose, dt)}')
        # self.get_logger().info(f'position:\t{self.w2 * self.position_error(cmd[0], cmd[1], diff_pose, dt)}')
        # self.get_logger().info(f'obstacle:\t{self.w3 * self.obstacle_error(cmd[0], cmd[1], dt)}')
        # self.get_logger().info(f'velocity:\t{self.w4 * self.velocity_error(cmd[0], cmd[1], diff_pose, dt)}')
        
        return cmd[0], cmd[1]
    
    def set_cmd_vel(self, control):
        self.cmd_vel = control
        self.vel = control.linear.x
        self.rot = control.angular.z
    
    def timer_cb(self):
        # when no trajectory is given, don't do anything
        if self.trajectory is None:
            self.cmd_vel = Twist()
            self.cmd_publisher.publish(self.cmd_vel)
            return

        diff_pose = self.get_target_pose(offset=0.0)
        if diff_pose.x**2 + diff_pose.y**2 > self.fail_radius**2:
            self.get_logger().warn(colored('to far from planned pose', 'red') + f'difference: {diff_pose}')
            self.set_trajectory_fail()
            return
        
        # compute and publish control output
        cmd = self.compute_dwa_output(dt=self.dt)
        c = Twist()
        c.linear.x, c.angular.z = cmd
        if c is None:
            self.set_trajectory_fail()
            self.get_logger().warn(colored('control is could not be computed', 'red') + f'diff_pose is: {diff_pose} ')
            return
        
        self.set_cmd_vel(c)
        self.cmd_publisher.publish(self.cmd_vel)
        
    def slow_timer_cb(self):
        # publish topics for vizual debugging and analysis
        if self.trajectory is not None:
            self.path_publisher.publish(self.trajectory)
        else:
            self.get_logger().warn('no trajectory')
        msg = MarkerArray()
        marker = Marker(action=Marker.ADD, ns="traj", id=0, type=Marker.LINE_STRIP)
        marker.header.frame_id = self.own_frame
        marker.scale.x = 0.02
        points = [self.position(self.vel, self.rot, t) for t in [0.25*self.dt, 0.5*self.dt, 0.75*self.dt, self.dt]]
        marker.points = [PointMsg(x=float(x), y=float(y), z=0.0) for x, y in points]
        marker.colors = [ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.8) for _ in marker.points]
        marker.lifetime = Duration(seconds=2.0).to_msg()
        msg.markers.append(marker)
        self.local_pub.publish(msg)
        self.local_pub.publish(self.publish_polygon_marker(self.scan_poly.buffer(-self.laser_inflation_size), ns="scan", id=0))
        self.tb_polygons = [Point(x,y).buffer(self.tb_radius, quad_segs=3) for x,y in self.tb_center_points]
        for i, tb in enumerate(self.tb_polygons):
            self.local_pub.publish(self.publish_polygon_marker(tb, ns="tb", id=i, color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)))

        if self.trajectory is None:
            return None
        diff_pose = self.get_target_pose(offset=self.dt)
        self.desired_pose_publisher.publish(self.tuple_to_pose_stamped_msg(diff_pose.x, diff_pose.y, diff_pose.theta, frame=self.own_frame))
        
    def publish_polygon_marker(self, polygon, ns="feasible", id=1, color=None):
        if color is None:
            color = ColorRGBA(r=0.5, g=1.0, b=0.5, a=0.4)
        poly_msg = MarkerArray()
        if polygon is None:
            return poly_msg
        if polygon.geom_type == 'MultiPolygon':
            # self.get_logger().warn(f'got multi polygon for "{ns}", will not show all components')
            for g in polygon.geoms:
                if g.geom_type == 'Polygon':
                    polygon = g
                    break
        marker = Marker(action=Marker.ADD, ns=ns, id=id, type=Marker.LINE_STRIP)
        marker.header.frame_id = self.own_frame
        marker.scale.x = 0.01
        coords = polygon.exterior.coords
        marker.points = [PointMsg(x=point[0], y=point[1], z=0.0) for point in coords]
        marker.colors = [color for _ in coords]
        poly_msg.markers.append(marker)
        return poly_msg

    def get_desired_pose(self, offset: float = 0.0) -> Pose2D:
        # get last pose, next pose and dt
        current_stamp = self.get_clock().now()
        trajectory_start = \
            rclpy.time.Time.from_msg(self.trajectory.header.stamp)
        rate = 1.0
        t = ((current_stamp - trajectory_start).nanoseconds * 1e-9 + offset)
        t *= rate
        if t < 0.0:
            self.get_logger().warn('trajectory start is in the future')
            t = 0.0
        last_index = int(np.floor(t))
        if last_index+2 >= len(self.trajectory.poses):
            pose = self.pose_stamped_to_tuple(self.trajectory.poses[-1], frame=self.own_frame, reset_time=True)
            return Pose2D(x=pose[0], y=pose[1], theta=pose[2])
        
        dt = (t - np.floor(t))
        last_pose = self.pose_stamped_to_tuple(self.trajectory.poses[last_index], frame=self.own_frame, reset_time=True)
        next_pose = self.pose_stamped_to_tuple(self.trajectory.poses[last_index + 1], frame=self.own_frame, reset_time=True)
        pose = (1-dt) * np.array(last_pose) + dt * np.array(next_pose)
        return Pose2D(x=pose[0], y=pose[1], theta=pose[2])
    
    def detect_tb(self, ranges, px=0.0, py=0.0, pt=0.0, angle_min=0.0, angle_increment=1.0):
        # detect turtlebot in scan ranges
        clusters = np.zeros_like(ranges, dtype=np.int32)
        
        # compute single-linkage clusters, where points are connected if they are closer than cluster_threshold
        last_range = None
        last_cluster = 0
        for i, r in enumerate(ranges):
            if last_range is None:
                last_range = r
                continue
            if r > 9.0:
                clusters[i] = last_cluster
                continue
            if np.abs(r - last_range) < self.cluster_linkage_threshold:
                clusters[i] = last_cluster
            else:
                last_cluster += 1
                clusters[i] = last_cluster
            last_range = r
        
        # if r[-1] and r[0] are close, connect the clusters
        if np.abs(ranges[-1] - ranges[0]) < self.cluster_linkage_threshold:
            clusters[clusters == clusters[-1]] = clusters[0]
        
        # compute cluster sizes
        cluster_sizes = np.bincount(clusters)
        
        # for each cluster, compute the mean angle and range
        cluster_center_index = np.zeros_like(cluster_sizes, dtype=np.int32)
        tb_center_points = []
        for i in range(len(cluster_sizes)):
            if cluster_sizes[i] > 30:
                continue
            min_index = np.min(np.where(clusters == i)[0])
            cluster_center_index[i] = (min_index + int(cluster_sizes[i] / 2)) % len(ranges)
            cluster_ranges = [ranges[ci] for ci in range(min_index, min_index + cluster_sizes[i] % len(ranges))]
            cr = np.mean(cluster_ranges)
            if cr > self.cluster_range_threshold:
                continue
            if cr * cluster_sizes[i] > 30:
                continue
            if cluster_sizes[i] < 2:
                continue
            cluster_positions = [self.get_xy_from_scan(ci, ranges[ci], px, py, pt, angle_min, angle_increment) for ci in range(min_index, min_index + cluster_sizes[i] % len(ranges))]
            cluster_center = np.mean(cluster_positions, axis=0)
            mean_distance_to_center = np.mean([np.linalg.norm(p - cluster_center) for p in cluster_positions])
            if mean_distance_to_center > 0.1:
                continue
            tb_center_points.append(cluster_center)

        
        self.tb_center_points = tb_center_points


    def scan_cb(self, msg):
        # convert laser scan to polygon
        ranges = msg.ranges
        ranges = [x if x > msg.range_min and x < msg.range_max else 10.0 for x in ranges]
        

        
        # px, py, pt = self.get_tf_pose()
        # here, we use local coordinates
        px, py, pt = 0.0, 0.0, 0.0

        # convert scan ranges to xy coordinates
        points = []

        for i, r in enumerate(ranges):
            x, y = self.get_xy_from_scan(i, r, px, py, pt, msg.angle_min, msg.angle_increment)
            points.append((x,y))

        self.scan_poly = Polygon(points) # .buffer(-self.laser_inflation_size)
        self.occupied = self.workspace.difference(self.scan_poly)

        self.detect_tb(ranges, px, py, pt, msg.angle_min, msg.angle_increment)
        #self.occupied = union_all(self.tb_polygons + [occupied])

    def get_xy_from_scan(self, i, r, px=0.0, py=0.0, pt=0.0, angle_min=0.0, angle_increment=1.0):
        angle = angle_min + i * angle_increment + pt
        x = r * np.cos(angle) + px
        y = r * np.sin(angle) + py
        return x,y


def main():
    rclpy.init()
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down after user interrupt')
    node.destroy_node()



if __name__ == '__main__':
    main()
