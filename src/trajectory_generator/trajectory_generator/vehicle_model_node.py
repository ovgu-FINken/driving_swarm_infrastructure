#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from driving_swarm_messages.srv import VehicleModel as VehicleModelService
from geometry_msgs.msg import Pose2D
from enum import IntEnum
from skimage import io
from trajectory_generator.bezier_path import calc_bezier_path
from trajectory_generator.dubins_path_planner import plan_dubins_path
from trajectory_generator.reeds_shepp_path_planning import reeds_shepp_path_planning

class Vehicle(IntEnum):
    DUBINS = 1
    STRAIGHT = 2
    RTR = 3
    REEDS_SHEPP = 4
    BEZIER = 5
    DUBINS_ADAPTIVE = 6
    REEDS_SHEPP_ADAPTIVE = 7

def normalize_angle(phi: float):
    return (phi + np.pi) % (2*np.pi) - np.pi


def angle_dist(phi1: float, phi2: float):
    d = normalize_angle(phi1 - phi2)
    if np.abs(d) < np.pi:
        return d
    return normalize_angle(2*np.pi - d)


def short_angle_range(phi1: float, phi2: float, r_step:float=0.2):
    phi2 = phi1 + angle_dist(phi2, phi1)
    if phi1 == phi2:
        return [phi1]
    return np.arange(phi1, phi2, np.sign(angle_dist(phi2, phi1)) * r_step)
    

def waypoints_to_path(waypoints, r=1, step=0.1, r_step=0.2, model=Vehicle.DUBINS, FIX_ANGLES=False, spline_degree=3):
    """Create a of poses from the given waypoints, adhering to the given vehicle model.

    :param waypoints: list of waypoint tuples (x, y, phi, v) -- v is optional
    :param r: radius of the curves (in dubins model), defaults to 1
    :param step: step size between poses, defaults to 0.1
    :param r_step: step size for rotation only movements, defaults to 0.2
    :param model: vehicle model to be used, defaults to Vehicle.DUBINS
    :param FIX_ANGLES: recompute the angles, defaults to False
    :param spline_degree: degree of splines for bezier model, defaults to 3
    :return: list of poses (x, y, phi)
    """
    path = []
    for wp1, wp2 in zip(waypoints[:-1], waypoints[1:]):
        #set step-size for this segment
        s = step
        #set step-size by speed for last wp of the segment
        v = 1.0
        if len(wp2) > 3:
            v = wp2[3]
        #if the segment is the last in the path, the wp will have no speed, use the speed from the first wp of the segment
        elif len(wp1) > 3:
            v = wp1[3]
            
        v = np.clip(v, 0.3, 1.0)
        s = step*v
        
        if model == Vehicle.DUBINS or model == Vehicle.DUBINS_ADAPTIVE:
            if model == Vehicle.DUBINS_ADAPTIVE:
                curvature = 1/(r*v)
            else:
                curvature = 1 / r
            # change format of waypoints from multiple np arrays to tuples in a list
            # we use 1/r as the curvature, as the dubins path planner expects the curvature instead of the radius
            sx, sy, sphi = wp1[0:3]
            gx, gy, gphi = wp2[0:3]
            xs, ys, phis, *_ = plan_dubins_path(sx, sy, sphi, gx, gy, gphi, curvature, step_size=step)
            path = path + list((x, y, phi) for x, y, phi in zip(xs, ys, phis))

        elif model==Vehicle.REEDS_SHEPP or model==Vehicle.REEDS_SHEPP_ADAPTIVE:
            if model == Vehicle.REEDS_SHEPP_ADAPTIVE:
                curvature = 1/(r*v)
            else:
                curvature = 1 / r
            sx, sy, sphi = wp1[0:3]
            gx, gy, gphi = wp2[0:3]
            xs, ys, phis, *_ = reeds_shepp_path_planning(sx, sy, sphi, gx, gy, gphi, curvature, step_size=step)
            path = path + list((x, y, phi) for x, y, phi in zip(xs, ys, phis))
            
            
        elif model == Vehicle.RTR or model == Vehicle.STRAIGHT:
            # rotate (1)
            dist = np.linalg.norm(np.array(wp1[0:2]) - np.array(wp2[0:2]))
            if dist < step:
                if not np.isnan(wp1[2]):
                    path.append(wp1)
                continue
            x = wp1[0]
            y = wp1[1]
            # if no orientation is set, use the last orientation
            # -- first orientation must be set, otherwise this will not work!
            if np.isnan(wp1[2]):
                phi = path[-1][2]
            else:
                phi = wp1[2] % (2 * np.pi)
            if dist > s or np.isnan(wp2[2]):
                dx = wp2[0] - wp1[0]
                dy = wp2[1] - wp1[1]
                # as per https://docs.scipy.org/doc/numpy/reference/generated/numpy.arctan2.html
                # Note the role reversal: the “y-coordinate” is the first function parameter, the “x-coordinate” is the second.
                phi_goal = np.arctan2(dy, dx)
            else:
                phi_goal = wp2[2]
            if model == Vehicle.RTR:
                for a in short_angle_range(phi, phi_goal, r_step=r_step):
                    path.append( (x, y, a) )
            
            # translate
            steps = dist / s
            if steps < 1:
                steps = 1.0
            dx = (wp2[0] - wp1[0]) / steps
            dy = (wp2[1] - wp1[1]) / steps
            for _ in range(int(steps)):
                x += dx
                y += dy
                path.append( (x, y, phi_goal) )
                
            if model == Vehicle.RTR and not np.isnan(wp2[2]):
                # rotate (2)
                x = wp2[0]
                y = wp2[1]
                phi = phi_goal
                phi_goal = wp2[2] % (2 * np.pi)
                for a in short_angle_range(phi, phi_goal, r_step=r_step):
                    path.append( (x, y, a) )
                    
        elif model == Vehicle.BEZIER:
            control_point1 = wp1[0] + np.sin(wp1[2])*r, wp1[1] + np.cos(wp1[2])*r
            control_point2 = wp2[0] - np.sin(wp2[2])*r, wp2[1] - np.cos(wp2[2])*r
            points = []
            nodes = [ wp1[0:2], control_point1, control_point2, wp2[0:2] ]
            curve = calc_bezier_path(nodes, degree=3)
            l = np.linspace(0.0, 1.0, num=int(curve.length / s))
            points = curve.evaluate_multi(l)
            angles = []
            if FIX_ANGLES:
                angles = [curve.evaluate_hodograph(i) for i in l]
                angles = [np.arctan2(x[1], x[0])[0] for x in angles]
            else:
                angles = [0 for _ in l]
                
            for i, (x, y) in enumerate(points.transpose()):
                path.append( (x, y, angles[i]) )
            
        else:
            print("NO VEHICLE MODEL!")
    
    if waypoints[-1] != path[-1] and not np.isnan(waypoints[-1][2]):
        path.append(waypoints[-1][0:3])
    return path

class TrajectoryGenerator:
    def __init__(self, model=Vehicle.DUBINS_ADAPTIVE, r=1, step=0.1, r_step=0.1):
        self.model = model
        self.r = r
        self.step = step
        self.costmap = None
        self.r_step = r_step

    def tuples_to_path(self, waypoints_tuples):
        tuples = waypoints_to_path(waypoints_tuples, r=self.r, model=self.model, step=self.step, r_step=self.r_step)
        if np.isnan(tuples[-1][2]):
            return tuples[:-1]
        return tuples

    def convert_map_data_to_image(self, map_data):
        image = np.array(map_data.data, dtype=int)
        image = np.reshape(image, (map_data.info.width, map_data.info.height))
        return image
    
    def metric_to_px_coorditaes(self, x, y):
        x = (x - self.raw_map.info.origin.position.x ) / self.raw_map.info.resolution
        y = (y - self.raw_map.info.origin.position.y ) / self.raw_map.info.resolution
        return int(x), int(y)

    def set_costmap(self, costmap):
        self.image = self.convert_map_data_to_image(costmap)
        self.raw_map = costmap
    
    def compute_trajectory_objectives(self, trajectory):
        danger = np.max([ self.image[self.metric_to_px_coorditaes(x,y)] * 20 / (i + 20) for i, (x, y, _) in enumerate(trajectory) ])
        length = 0
        for p, q in zip(trajectory[:-1], trajectory[1:]):
            length += np.sqrt( (p[0]-q[0])**2 + (p[1]-q[1])**2 )

        return {'length':length, 'time': len(trajectory), 'danger': danger}
    
    def plot_trajectory_in_costmap(self, trajectory):
        img = self.image.copy()
        for x, y, _ in trajectory:
           img[self.metric_to_px_coorditaes(x,y)] = -100

        io.imshow(img)
        io.show() 
    

class VehicleModelNode(Node):
    def __init__(self):
        super().__init__('vehicle_model_node')
        self.get_logger().info("Starting")
        self.create_service(VehicleModelService, 'nav/vehicle_model', self.vm_callback)
        self.declare_parameter('vehicle_model')
        self.declare_parameter('step_size')
        self.declare_parameter('turn_radius')
        self.vm = TrajectoryGenerator(
            model = Vehicle(self.get_parameter('vehicle_model').get_parameter_value().integer_value),
            step = self.get_parameter('step_size').get_parameter_value().double_value,
            r = self.get_parameter('turn_radius').get_parameter_value().double_value
        )
        
        
    def vm_callback(self, request, response):
        self.get_logger().info(f'got request {request}')
        waypoints = []
        for wp, v in zip(request.waypoints, request.speeds):
            waypoints.append((wp.x, wp.y, wp.theta, v))
        path = self.vm.tuples_to_path(waypoints)
        for pose in path:
            xyt = Pose2D()
            xyt.x, xyt.y, xyt.theta = pose
            response.trajectory.append(xyt)
        return response


def main():
    rclpy.init()
    node = VehicleModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'got keyboard interrupt, shutting down')
        node.destroy_node()


if __name__ == '__main__':
    main()
