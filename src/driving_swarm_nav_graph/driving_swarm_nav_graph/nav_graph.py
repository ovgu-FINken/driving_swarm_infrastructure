import rclpy
from rclpy.node import Node

import shapely
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from driving_swarm_messages.srv import SaveToFile
from polygonal_roadmaps import pathfinding, geometry, polygonal_roadmap


class NavGraphNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.declare_parameter('graph_file')
        self.declare_parameter('x_min', -1.0)
        self.declare_parameter('x_max', 4.0)
        self.declare_parameter('y_min', -1.0)
        self.declare_parameter('y_max', 4.0)
        map_file = self.get_parameter('graph_file').get_parameter_value().string_value
        
        wx = (self.get_parameter('x_min').get_parameter_value().double_value, self.get_parameter('x_max').get_parameter_value().double_value)
        wy = (self.get_parameter('y_min').get_parameter_value().double_value, self.get_parameter('y_max').get_parameter_value().double_value)
        self.declare_parameter('grid_type', 'hex')
        self.declare_parameter('grid_size', .6)
         
            
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
        self.env = polygonal_roadmap.RoadmapEnvironment(map_file,
                                                        None,
                                                        None,
                                                        generator_points=points,
                                                        wx=wx,
                                                        wy=wy,
                                                        offset=0.2)
        self.create_service(SaveToFile, 'save_graph', self.save_graph)

        self.get_logger().info(f"graph generated {map_file}")
        self.create_timer(1.0, self.visualization_timer_cb)
    
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
        
    def save_graph(self, req, res):
        self.g.save(req.filename)
        return res

    def visualization_timer_cb(self):
        self.poly_pub.publish(self.graph_to_marker_array())


def main():
    rclpy.init()
    node = NavGraphNode('roadmap')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
