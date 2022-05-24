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
        map_file = self.get_parameter('graph_file').get_parameter_value().string_value
        wx = (-0.75, 4)
        wy = (-3, 0.75)
        self.declare_parameter('tiling')
            
        if map_file.endswith(".yaml"):
            tiling = self.get_parameter('tiling').get_parameter_value().string_value
            if tiling == 'hex':
                points = geometry.hexagon_tiling(0.60, working_area_x=wx, working_area_y=wy)
            elif tiling == 'square':
                points = square_tiling(0.60, **self.wa)
            elif tiling == 'random':
                points = geometry.random_tiling(50, working_area_x=wx, working_area_y=wy)
            else:
                self.get_logger().warn('no tiling specified, using hex')
                points = geometry.hexagon_tiling(0.60, working_area_x=wx, working_area_y=wy)
        self.env = polygonal_roadmap.RoadmapEnvironment(map_file,
                                                        None,
                                                        None,
                                                        generator_points=points,
                                                        wx=wx,
                                                        wy=wy)
        self.create_service(SaveToFile, 'save_graph', self.save_graph)

        self.poly_pub = self.create_publisher(MarkerArray, 'cells', 10)
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
