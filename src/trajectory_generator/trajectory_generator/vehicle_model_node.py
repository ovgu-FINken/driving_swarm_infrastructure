#!/usr/bin/env python
import dubins
import reeds_shepp
import bezier

import rclpy
import numpy as np
from rclpy.node import Node
from driving_swarm_messages.srv import VehicleModel as VehicleModelService
from geometry_msgs.msg import Pose2D
from enum import IntEnum
from skimage import io

class Vehicle(IntEnum):
    DUBINS = 1
    STRAIGHT = 2
    RTR = 3
    REEDS_SHEPP = 4
    BEZIER = 5
    DUBINS_ADAPTIVE = 6
    REEDS_SHEPP_ADAPTIVE = 7


def short_angle_range(phi1, phi2, r_step=0.2):
    if np.abs(phi1 - (2*np.pi + phi2)) < np.abs(phi1 - phi2):
        phi2 = 2*np.pi + phi2
    if np.abs(phi1 - (-2*np.pi + phi2)) < np.abs(phi1 - phi2):
        phi2 = -2*np.pi + phi2
        
    if phi1 < phi2:
        return np.arange(phi1, phi2, r_step)
    return np.arange(phi1, phi2, -r_step)
    

def waypoints_to_path(waypoints, r=1, step=0.1, r_step=0.2, model=Vehicle.DUBINS, FIX_ANGLES=True, spline_degree=3):
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
        
        if model == Vehicle.DUBINS:
            dbp = dubins.shortest_path(wp1, wp2, r)
            path = path + dbp.sample_many(s)[0]
        elif model == Vehicle.DUBINS_ADAPTIVE:
            dbp = dubins.shortest_path(wp1, wp2, r*v)
            path = path + dbp.sample_many(s)[0]
            
        elif model == Vehicle.RTR or model == Vehicle.STRAIGHT:
            # rotate (1)
            dist = np.linalg.norm(np.array(wp1[0:2]) - np.array(wp2[0:2]))
            x = wp1[0]
            y = wp1[1]
            phi = wp1[2] % (2 * np.pi)
            if dist > s:
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
                
            if model == Vehicle.RTR:
                # rotate (2)
                x = wp2[0]
                y = wp2[1]
                phi = phi_goal
                phi_goal = wp2[2] % (2 * np.pi)
                for a in short_angle_range(phi, phi_goal, r_step=r_step):
                    path.append( (x, y, a) )
                    
#                if len(path) < 3:
#                    print("OH NO")
#                    print(f"{wp1}, {wp2}, {path}, {phi}")
        elif model==Vehicle.REEDS_SHEPP or model==Vehicle.REEDS_SHEPP_ADAPTIVE:
            part = []
            r_temp = r
            if model == Vehicle.REEDS_SHEPP_ADAPTIVE:
                r_temp = r * v
            sample = reeds_shepp.path_sample(wp1, wp2, r_temp, s)
            for s in sample:
                part.append( (s[0], s[1], s[2]) )
            # cleanup angles
            if FIX_ANGLES:
                for i, xy in enumerate(part[:-1]):
                    dx = xy[0] - part[i+1][0]
                    dy = xy[1] - part[i+1][1]

                    phi = np.arctan2(dy, dx)
                    path.append( (xy[0], xy[1], phi) ) 
                path.append(wp2[0:3])
            else:
                path = path + part
        elif model == Vehicle.BEZIER:
            control_point1 = wp1[0] + np.sin(wp1[2])*r, wp1[1] + np.cos(wp1[2])*r
            control_point2 = wp2[0] - np.sin(wp2[2])*r, wp2[1] - np.cos(wp2[2])*r
            points = []
            nodes = np.asfortranarray([
                [wp1[0], control_point1[0], control_point2[0], wp2[0]],
                [wp1[1], control_point1[1], control_point2[1], wp2[1]]
            ])
            curve = bezier.Curve(nodes, degree=3)
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
    
        
    if waypoints[-1] != path[-1]:
        path.append(waypoints[-1][0:3])
    return path

class TrajectoryGenerator:
    def __init__(self, model=Vehicle.DUBINS_ADAPTIVE, r=1, step=0.1):
        self.model = model
        self.r = r
        self.step = step
        self.costmap = None

    def tuples_to_path(self, waypoints_tuples):
        return waypoints_to_path(waypoints_tuples, r=self.r, model=self.model, step=self.step)

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
