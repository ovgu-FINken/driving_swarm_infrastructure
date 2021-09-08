import rclpy
from rclpy.node import Node

from driving_swarm_nav_graph.utils import *
import shapely
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from driving_swarm_messages.srv import SaveToFile
from graph_tool import load_graph



class NavGraphNode(Node):
    def __init__(self):
        super().__init__('nav_graph')
        self.declare_parameter('graph_file')
        map_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.wa = {
            'working_area_x': (-0.75, 4),
            'working_area_y': (-3, 0.75),
        }
        self.declare_parameter('tiling')
            
        if map_file.endswith(".yaml"):
            tiling = self.get_parameter('tiling').get_parameter_value().string_value
            if tiling == 'hex':
                points = hexagon_tiling(0.60, **self.wa)
            elif tiling == 'square':
                points = square_tiling(0.40, **self.wa)
            elif tiling == 'random':
                points = random_tiling(50, **self.wa)
            else:
                self.get_logger().warn('no tiling specified, using hex')
                points = hexagon_tiling(0.60, **self.wa)
            _, occupied_space = read_obstacles(map_file)
            assert(occupied_space is not None)
            self.g = create_graph(points, offset=0.15,
                                        occupied_space=occupied_space, **self.wa)
        elif map_file.endswith(".xml.gz") or map_file.endswith(".xml"):
            self.g = load_graph(map_file)
        self.g.set_vertex_filter(self.g.vp['traversable'])
        self.g.set_edge_filter(self.g.ep['traversable'])
        self.create_service(SaveToFile, 'save_graph', self.save_graph)

        self.poly_pub = self.create_publisher(MarkerArray, 'cells', 10)
        self.get_logger().info(f"graph generated {map_file}")
        self.create_timer(1.0, self.visualization_timer_cb)
    
    def graph_to_marker_array(self):
        poly_msg = MarkerArray()
        for i, v in enumerate(self.g.vertices()):
            marker = Marker(action=Marker.ADD, ns="inner", id=i, type=Marker.LINE_STRIP)
            marker.header.frame_id = 'map'
            marker.scale.x = 0.02
            coords = self.g.vp['geometry'][v].inner.exterior.coords
            marker.points = [Point(x=point[0], y=point[1], z=0.0) for point in coords]
            marker.colors = [ColorRGBA(r=0.5, g=0.8, b=0.5, a=0.3) for _ in coords]
            poly_msg.markers.append(marker)

        for i, e in enumerate(self.g.edges()):
            if self.g.ep['geometry'][e].borderPoly is None:
                continue
            marker = Marker(action=Marker.ADD, ns="transition", id=i, type=Marker.LINE_STRIP)
            marker.header.frame_id = 'map'
            marker.scale.x = 0.01
            coords = self.g.ep['geometry'][e].borderPoly.exterior.coords
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
    node = NavGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
