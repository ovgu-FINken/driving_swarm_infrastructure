
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
        self.declare_parameter("fail_radius", 1.0)
        self.fail_radius = self.get_parameter("fail_radius") \
                               .get_parameter_value().double_value
        self.declare_parameter("n_samples", 7)
        self.n_samples = 4 # self.get_parameter("n_samples").get_parameter_value().integer_value
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
        self.max_rot = 1.0
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
        return response

    def pose2D_to_PoseStamped(self, pose2d, header_id=None):
        pose_stamped = PoseStamped()
        if header_id is None:
            pose_stamped.header.frame_id = self.reference_frame
        else:
            pose_stamped.header.frame_id = header_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = pose2d.x
        pose_stamped.pose.position.y = pose2d.y
        pose_stamped.pose.position.z = 0.0
        q = Quaternion()
        q.x, q.y, q.z, q.w = tf_transformations.quaternion_from_euler(.0, .0, pose2d.theta)
        pose_stamped.pose.orientation = q
        return pose_stamped

    def poseStamped_to_Pose2D(self, pose_stamped):
        """A function that transforms tha pose_stamped from a plan to a Pose2D in EGO coordinates

        :param pose_stamped: pose from a plan
        :return: Pose2D in EGO coordinates
        """
        pose2d = Pose2D()
        try:
            # get the transform so we can read at wich time it was performed
            t = self.tf_buffer.lookup_transform(
                pose_stamped.header.frame_id,
                self.reference_frame,
                rclpy.time.Time().to_msg()
            )
            # set the time to the most recent transform
            pose_stamped.header.stamp = t.header.stamp
            pose3d = self.tf_buffer.transform(
                pose_stamped,
                self.own_frame
            )
            pose2d.x = pose3d.pose.position.x
            pose2d.y = pose3d.pose.position.y
            q = pose3d.pose.orientation
            pose2d.theta = \
                tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        except Exception as e:
            self.get_logger().warn(
                'could not transform pose' +
                f' in frame {pose_stamped.header.frame_id}' +
                f' to reference frame {self.own_frame} \n {e}'
            )
        return pose2d
 
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
        #if rot*dt < 0.0000001:
        #    return vel * dt, 0
        #r = vel / rot
        #dx = np.abs(r) * (1.0 - np.cos(rot * dt)) * np.sign(vel)
        #dy = r * (np.sin(rot * dt))
        #return dx, dy
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
    
    def obstacle_error(self, vel, rot, dt):
        p = [self.position(vel, rot, t) for t in np.linspace(0.25*dt, dt, 4)]
        ls = LineString(p)
        dist = self.occupied.distance(ls)
        # should be the case anyway
        #if self.occupied.contains(ls):
        #    dist = 0

        return np.clip(self.obstacle_threshold - dist, 0.0, self.obstacle_threshold)
        
    def value(self, vel, rot, dt, diff_pose):
        return self.w1 * self.alignment_error(vel, rot, diff_pose, dt) + self.w2 * self.position_error(vel, rot, diff_pose, dt) + self.w3 * self.obstacle_error(vel, rot, dt)
        
    def compute_dwa_output(self, dt) -> tuple:
        if self.trajectory is None:
            return None
        
        diff_pose = self.get_target_pose(offset=dt)
        diff_pose = diff_pose.x, diff_pose.y, diff_pose.theta

        
        admissable_min = max(self.min_vel, self.vel - self.max_accel_x)
        admissable_max = min(self.max_vel, self.vel + self.max_accel_x)
        admissable_vel = np.linspace(admissable_min, admissable_max, self.n_samples)
        admissable_min = max(-self.max_rot, self.rot - self.max_accel_z)
        admissable_max = min(self.max_rot, self.rot + self.max_accel_z)
        admissable_rot = np.linspace(admissable_min, admissable_max)
        
        admissable_cmd = [(x, y) for x in admissable_vel for y in admissable_rot]
        cmd = min(admissable_cmd, key=lambda x: self.value(*x, dt, diff_pose))
        self.get_logger().debug(f'calculate_dwa output: vel:{cmd[0]}, rot: {cmd[1]}')
        self.get_logger().debug(f'alignment: {self.alignment_error(cmd[0], cmd[1], diff_pose, dt)}')
        self.get_logger().debug(f'position: {self.position_error(cmd[0], cmd[1], diff_pose, dt)}')
        self.get_logger().debug(f'obstacle: {self.obstacle_error(cmd[0], cmd[1], dt)}')
        
        return cmd[0], cmd[1]
    
    def set_cmd_vel(self, control):
        # todo: respect limits (acceleration + max vel)
        # self.cmd_vel = control
        self.cmd_vel.linear.x = np.clip(
            control.linear.x,
            self.cmd_vel.linear.x - self.max_accel_x,
            self.cmd_vel.linear.x + self.max_accel_x
        )
        self.cmd_vel.angular.z = np.clip(
            control.angular.z,
            self.cmd_vel.angular.z - self.max_accel_z,
            self.cmd_vel.angular.z + self.max_accel_z
        )
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
        self.local_pub.publish(self.publish_polygon_marker(self.scan_poly, ns="scan", id=0))
        for i, tb in enumerate(self.tb_polygons):
            self.local_pub.publish(self.publish_polygon_marker(tb, ns="tb", id=i, color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)))

        if self.trajectory is None:
            return None
        self.desired_pose_publisher.publish(
            self.pose2D_to_PoseStamped(
                self.get_desired_pose(), header_id=self.name
            )
        )
        self.path_publisher.publish(self.trajectory)
        
    def publish_polygon_marker(self, polygon, ns="feasible", id=1, color=None):
        if color is None:
            color = ColorRGBA(r=0.5, g=1.0, b=0.5, a=0.4)
        poly_msg = MarkerArray()
        if polygon is None:
            return poly_msg
        if polygon.geom_type == 'MultiPolygon':
            self.get_logger().warn(f'got multi polygon for "{ns}", will not show all components')
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
            return self.poseStamped_to_Pose2D(self.trajectory.poses[-1])
        dt = (t - np.floor(t))
        last_pose = self.poseStamped_to_Pose2D(
            self.trajectory.poses[last_index]
        )
        next_pose = self.poseStamped_to_Pose2D(
            self.trajectory.poses[last_index + 1]
        )
        x = (1 - dt) * last_pose.x + dt * next_pose.x
        y = (1 - dt) * last_pose.y + dt * next_pose.y
        theta = (1 - dt) * last_pose.theta + dt * next_pose.theta
        return Pose2D(x=x, y=y, theta=theta)
    
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
            if cr * cluster_sizes[i] > 8:
                continue
            cluster_positions = [self.get_xy_from_scan(ci, ranges[ci], px, py, pt, angle_min, angle_increment) for ci in range(min_index, min_index + cluster_sizes[i] % len(ranges))]
            cluster_center = np.mean(cluster_positions, axis=0)
            mean_distance_to_center = np.mean([np.linalg.norm(p - cluster_center) for p in cluster_positions])
            if mean_distance_to_center > 0.5:
                continue
            tb_center_points.append(cluster_center)

        
        self.tb_polygons = [Point(x,y).buffer(self.tb_radius, quad_segs=4) for x,y in tb_center_points]


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

        self.scan_poly = Polygon(points).buffer(-self.laser_inflation_size)
        occupied = self.workspace.difference(self.scan_poly)

        self.detect_tb(ranges, px, py, pt, msg.angle_min, msg.angle_increment)
        self.occupied = union_all(self.tb_polygons + [occupied])

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
