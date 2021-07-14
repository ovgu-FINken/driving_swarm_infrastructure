import numpy as np
from scipy.spatial import Voronoi
import shapely.geometry
from shapely.geometry import Polygon, Point, MultiLineString, LineString
import shapely.ops
import matplotlib.pyplot as plt
from dataclasses import dataclass
from graph_tool.all import * #noqa
from skimage import io, measure
from deprecation import deprecated
import yaml
import os

@dataclass
class NavNode:
    """class for keeping node data"""
    center: Point
    outer: Polygon
    inner: Polygon
    name: str


@dataclass
class NavEdge:
    """ class for edges between nodes """
    borderLine: MultiLineString
    connection: MultiLineString
    borderPoly: Polygon


def create_graph(generator_points: np.array,
                 working_area_x=(0.0, 1.0),
                 working_area_y=(0.0, 1.0),
                 offset: float=0.02,
                 occupied_space=None):
    limits = Polygon([
        (working_area_x[0], working_area_y[0]),
        (working_area_x[1], working_area_y[0]),
        (working_area_x[1], working_area_y[1]),
        (working_area_x[0], working_area_y[1])
    ])
    if occupied_space is not None:
        limits = limits.difference(occupied_space)
    
    # we add the outer points in order to get closed cells (not ending in infinity)
    padded_generators = np.vstack([generator_points, np.array([[0, 100], [0, -100], [-100, 0], [100, 0]])])
    vor = Voronoi(padded_generators)

    lines = [shapely.geometry.LineString(vor.vertices[line]) for line in vor.ridge_vertices if -1 not in line]
    polys = [p for p in shapely.ops.polygonize(lines)]
    nodes = []
    for i, poly in enumerate(polys):
        x, y = None, None
        for p in generator_points:
            if shapely.geometry.Point(p).within(poly):
                x, y = p[0], p[1]
                break
        outer = poly.intersection(limits)

        # in case polygon is split by obstacles -- use largest polygon as shape
        if outer.geometryType() == 'MultiPolygon':
            outer = sorted(outer, key=lambda p: p.area, reverse=True)[0]
        inner = outer.buffer(-offset)
        if inner.geometryType() == 'MultiPolygon':
            inner = sorted(inner, key=lambda p: p.area, reverse=True)[0]
        nodes.append(NavNode(center=Point(x, y), outer=outer, inner=inner, name=f'N{i}'))

    edges = []
    for i, n1 in enumerate(nodes):
        for j, n2 in enumerate(nodes):
            if i >= j:
                continue
            x = shapely.ops.shared_paths(n1.outer.exterior, n2.outer.exterior)
            if x.is_empty:
                continue
            e = [a for a in x if not a.is_empty and a.length > 0]
            if len(e) > 0:
                if n1.inner is None or n2.inner is None or n1.inner.is_empty or n2.inner.is_empty \
                        or not n1.inner.is_valid or not n2.inner.is_valid:
                    edges.append(
                                    (n1, n2, NavEdge(
                                        borderLine=e[0],
                                        connection=LineString([n1.center, n2.center]),
                                        borderPoly=None),
                                        i,
                                        j
                                    )
                                )
                    continue
                try:
                    bp = n1.outer.union(n2.outer).buffer(-offset).difference(n1.inner).difference(n2.inner)
                    if bp.is_valid:
                        if bp.geometryType() == 'MultiPolygon':
                            x = None
                            for p in bp:
                                if p.touches(n1.inner) and p.touches(n2.inner):
                                    x = p
                            bp = x
                    
                    edges.append(
                        (
                            n1, n2, NavEdge(
                            borderLine=e[0],
                            connection=LineString([n1.center, n2.center]),
                            borderPoly=bp),
                            i, j
                        )
                    )
                except NotImplementedError:
                    drawGraph([n1, n2], [], show=False)
                    for l in e[0]:
                        plt.plot(*l.xy)
                    plt.show()
                    
    return gen_graph(nodes, edges)
    

def square_tiling(dist, working_area_x=(0.0,1.0), working_area_y=(0.0,1.0)):
    XX, YY = np.meshgrid(np.arange(working_area_x[0]+dist/2, working_area_x[1], dist), np.arange(working_area_y[0]+dist/2, working_area_y[1], dist), indexing='xy')
    return np.dstack([ XX, YY]).reshape(-1, 2)


def hexagon_tiling(dist, working_area_x=(0.0,1.0), working_area_y=(0.0,1.0)):
    dy = dist * np.sqrt(0.75)
    XX, YY = np.meshgrid(np.arange(working_area_x[0]+dist*0.1, working_area_x[1]-dist/2, dist), np.arange(working_area_y[0]+dist/2, working_area_y[1], dy), indexing='xy')
    XX[::2] += dist * 0.5
    return np.dstack([ XX, YY]).reshape(-1, 2)


def random_tiling(n, working_area_x=(0.0,1.0), working_area_y=(0.0, 1.0)):
    t = np.random.random((n, 2))
    t[:,0] = t[:,0] * (working_area_x[1] - working_area_x[0]) + working_area_x[0]
    t[:,1] = t[:,1] * (working_area_y[1] - working_area_y[0]) + working_area_y[0]
    return t


def read_map(info_file):
    with open(info_file, 'r') as stream:
        info = yaml.safe_load(stream)
    img_file = info['image']
    if img_file[0] not in ['/', '~']:
        img_file = os.path.join(os.path.dirname(info_file), img_file)
    img = io.imread(img_file).transpose()
    return img, info

def convert_coordinates(poly, resolution:float, oX:float, oY:float):
    poly = poly * resolution + np.array([oX, oY+resolution])
    poly[:,1] *= -1
    return Polygon(poly)

def read_obstacles(file_name):
    img, info = read_map(file_name)
    thresh = 100
    contours = measure.find_contours(img, level=thresh)
    unclassified = contours
    obstacles, free = [], []

    # classify different levels of objects
    # holes need to be filled by opposite value
    while unclassified:
        for poly in unclassified:
            if np.max(img[measure.grid_points_in_poly(img.shape, poly)]) <= thresh:
                img[measure.grid_points_in_poly(img.shape, poly)] = 200
                unclassified.remove(poly)
                p = convert_coordinates(poly, info['resolution'], info['origin'][0], info['origin'][1])
                for f in free:
                    if p.contains(f):
                        p = p.difference(f)
                obstacles.append(p)
                continue

            if np.min(img[measure.grid_points_in_poly(img.shape, poly)]) >= thresh:
                img[measure.grid_points_in_poly(img.shape, poly)] = 30
                unclassified.remove(poly)
                p = convert_coordinates(poly, info['resolution'], info['origin'][0], info['origin'][1])
                for o in obstacles:
                    if p.contains(o):
                        p = p.difference(o)
                free.append(p)

    return shapely.ops.unary_union(free), shapely.ops.unary_union(obstacles)


def gen_graph(nodes, edges):
    g = Graph(directed=False)
    g.vertex_properties['center'] = g.new_vertex_property('vector<double>')
    g.vertex_properties['traversable'] = g.new_vertex_property('bool')
    g.vertex_properties['geometry'] = g.new_vertex_property('object')
    for n in nodes:
        v = g.add_vertex() 
        g.vp['center'][v] = n.center.x, n.center.y
        g.vp['traversable'][v] = n.inner is not None
        g.vp['geometry'][v] = n
    
    g.ep['dist'] = g.new_edge_property('double')
    g.ep['traversable'] = g.new_edge_property('bool')
    g.ep['geometry'] = g.new_edge_property('object')
    for le in edges:
        e = g.add_edge(le[-2], le[-1])
        g.ep['dist'][e] = le[0].center.distance(le[1].center)
        borderPoly = le[2].borderPoly
        if borderPoly is None or not borderPoly.is_valid or borderPoly.is_empty:
            g.ep['traversable'][e] = False
        else:
            bigger_poly = borderPoly.buffer(1e-8)
            g.ep['traversable'][e] = bigger_poly.intersects(le[0].inner) and bigger_poly.intersects(le[1].inner)

        g.ep['geometry'][e] = le[2]

    return g


def drawGraph(g, inner=True, outer=True, center=True, connections=True, borderPolys=True, show=True, edge_weight=None, node_weight=None):
    wax = g.gp['wa']['working_area_x']
    way = g.gp['wa']['working_area_y']
    width = 6
    height = width * (way[1] - way[0]) / (wax[1] - wax[0])
    
    plt.figure(figsize=(width, height))
    for v in g.iter_vertices():
        n = g.vp['geometry'][v]
        if n.inner and n.inner is not None:
            plt.plot(*n.inner.exterior.xy, ':', color='black')
        if outer:
            plt.plot(*n.outer.exterior.xy, '--', color='black')
        if center:
            s = 5
            if node_weight is not None:
                s *= 2.0 * g.vp[node_weight][v] / g.vp[node_weight].a.max()

            plt.plot(*n.center.xy, 'o', markersize=s, color='black')
    
    for ge in g.edges():
        e = g.ep['geometry'][ge]
        if connections:
            lw = None
            if edge_weight is not None:
                lw = 4.0 * g.ep[edge_weight][ge] / g.ep[edge_weight].a.max()
            plt.plot(*e.connection.xy, lw=lw, color='b', alpha=0.3)
        if borderPolys and e.borderPoly is not None and not e.borderPoly.is_empty:
            if e.borderPoly.geometryType() == 'MultiPolygon':
                for p in e.borderPoly:
                    plt.plot(*p.exterior.xy)
            else:
                plt.fill(*e.borderPoly.exterior.xy, alpha=0.1)
    if show:
        plt.show()



def find_nearest_node(g, p):
    dist = [np.linalg.norm(np.array(p) - np.array(g.vp['center'][n])) for n in g.iter_vertices()]
    return np.argmin(dist)


class NavVisitor(AStarVisitor):
    def __init__(self, touched_v, touched_e, goal):
        self.touched_e = touched_e
        self.touched_v = touched_v
        self.target = goal

    def discover_vertex(self, u):
        self.touched_v[u] = True
    
    def examine_edge(self, e):
        self.touched_e[e] = True
    
    def edge_relaxed(self, e):
        if e.target() == self.target:
            raise StopSearch()


class SpaceTimeVisitor(AStarVisitor):
    def __init__(self, generating_graph, goal, limit=100, node_constraints=None, edge_constraints=None):
        self.timeless = generating_graph
        self.g = graph_tool.Graph(directed=True)
        self.g.vp['index'] = self.g.new_vertex_property('int')
        self.g.vp['t'] = self.g.new_vertex_property('int')
        self.g.ep['dist'] = self.g.new_edge_property('double')
        self.g.vp['cost'] = self.g.new_vertex_property('double')
        self.g.vp['dist'] = self.g.new_vertex_property('double')

        self.target = goal
        for v in self.timeless.vertices():
            n = self.g.add_vertex()
            self.g.vp['index'][n] = int(v)
            self.g.vp['t'][n] = 0
        
        self.touched_e = self.g.new_edge_property('bool')
        self.touched_v = self.g.new_vertex_property('bool')
        self.limit=limit
        self.nc = node_constraints if node_constraints else {}
        self.ec = edge_constraints if edge_constraints else {}

    def get_node(self, t, index):
        for v in self.g.vertices():
            if self.g.vp['index'][v] == index and self.g.vp['t'][v] == t:
                return v
        return None

    def add_vertex_if_not_exists(self, t, index):
        if t in self.nc:
            if index in self.nc[t]:
                return None
        n = self.get_node(t, index)
        if n is not None:
            return n
        n = self.g.add_vertex()
        self.g.vp['index'][n] = index
        self.g.vp['t'][n] = t
        self.g.vp['cost'][n] = np.inf
        self.g.vp['dist'][n] = np.inf
        return n

    def add_edge_if_feasible(self, t, v1, v2):
        """add an edge to the graph from v1 to v2 iff. the edge is not forbidden"""

        i1 = self.g.vp['index'][v1]
        i2 = self.g.vp['index'][v2]

        if i1 == i2:
            e = self.g.add_edge(v, vout)
            self.g.ep['dist'][e] = .1
            return e

        if t in self.ec:
            if (i1, i2) in self.ec[t]:
                return None
            if (i2, i1) in self.ec[t]:
                return None
        
        e = self.g.add_edge(v1, v2)
        self.g.ep['dist'][e] = self.timeless.ep['dist'][self.timeless.edge(i1, i2)]

        return e


    def add_outgoing_edges(self, v):
        index = self.g.vp['index'][v]
        t = self.g.vp['t'][v]
        for n in self.timeless.vertex(index).all_neighbors():
            vout = self.add_vertex_if_not_exists(t+1, int(n))
            if vout is not None:
                self.add_edge_if_feasible(t, v, vout)
        # add edge to self node (waiting action)
        
        vout = self.add_vertex_if_not_exists(t+1, index)
        if vout is not None:
            e = self.g.add_edge(v, vout)
            self.g.ep['dist'][e] = .1

    def discover_vertex(self, u):
        self.touched_v[u] = True
        if self.g.vp['t'][u] > self.limit:
            self.timed_target_node = None
            raise StopSearch()
        # add nodes and edges going out of this node
        self.add_outgoing_edges(u)

    def examine_edge(self, e):
        self.touched_e[e] = True
    
    def edge_relaxed(self, e):
        if self.g.vp['index'][e.target()] == int(self.target):
            self.timed_target_node = e.target()
            raise StopSearch()


def check_node_constraints(g, v, nc):
    t = g.vp['t'][v]
    i = g.vp['index'][v]
    if t not in nc:
        return False
    return i in nc[t]


def pred_to_list(g, pred, start, goal):
    p = goal
    l = [p]
    g.vp['visited'] = g.new_vertex_property("bool")
    while p != start:
        if pred[p] == p:
            break
        p = pred[p]
        l.append(p)
    l.reverse()
    return l

def add_edge_constraints(edge_constraints, path):
    """append the constraints resulting from particular path to the list of existing constraints"""
    ec = {t: [e] for t, e in enumerate(zip(path[:-1], path[1:]))}
    if edge_constraints is None:
        return ec
    for k, v in ec.items():
        if k in edge_constraints:
            edge_constraints[k] += v
        else:
            edge_constraints[k] = v
        
    return edge_constraints

def add_node_constraints(node_constraints, path):
    """append the constraints resulting from particular path to the list of existing constraints"""
    ec = {t: [n] for t, n in enumerate(path)}
    if node_constraints is None:
        return ec
    for k, v in ec.items():
        if k in node_constraints:
            node_constraints[k] += v
        else:
            node_constraints[k] = v
        
    return node_constraints

def prioritized_plans(g, start_goal, edge_constraints=None, node_constraints=None, limit=10):
    """compute a set of paths for multiple agents in the same graph.
    first agent is planned first, constraints are created for the remaining agents
    start_goal -- [(start, goal) for agent in agents]"""
    # plan first path with A-Star
    paths = []
    for sn, gn in start_goal:
        paths.append(find_constrained_path(g, sn, gn, edge_constraints=edge_constraints, node_constraints=node_constraints, limit=limit))
        edge_constraints = add_edge_constraints(edge_constraints, paths[-1])
        node_constraints = add_node_constraints(node_constraints, paths[-1])
    
    return paths

def find_constrained_path(g, sn, gn, edge_constraints=None, node_constraints=None, limit=10):
    if edge_constraints is None and node_constraints is None:
        return find_path_astar(g, sn, gn)

    nv = SpaceTimeVisitor(g, gn, node_constraints=node_constraints, edge_constraints=edge_constraints, limit=limit)
    _, pred = astar_search(nv.g,
        sn,
        nv.g.ep['dist'],
        nv,
        implicit=True,
        heuristic=lambda v: np.linalg.norm(np.array(g.vp['center'][nv.g.vp['index'][v]]) - g.vp['center'][gn]),
        cost_map=nv.g.vp['cost'],
        dist_map=nv.g.vp['dist']
    )

    l = pred_to_list(nv.g, pred, sn, nv.timed_target_node)
    return [nv.g.vp["index"][v] for v in l]


def find_path_astar(g, sn, gn):
    """find shortest path through graph g with a* algorithm"""
    visitor = NavVisitor(g.new_vertex_property("bool"), g.new_edge_property("bool"), gn)
    _, pred = astar_search(g,
        sn,
        g.ep['dist'],
        visitor,
        heuristic=lambda v: np.linalg.norm(np.array(g.vp['center'][v]) - np.array(g.vp['center'][gn]))
    )
    return pred_to_list(g, pred, sn, gn)


def poly_from_path(g, path, eps=0.1):
    poly = [g.vp['geometry'][p].inner.buffer(eps) for p in path]
    poly += [g.ep['geometry'][g.edge(*e)].borderPoly.buffer(eps) for e in zip(path[:-1], path[1:])]
    return shapely.ops.unary_union(poly).buffer(-eps).simplify(eps)


def path_from_positions(g, start, goal):
    g.set_edge_filter(None)
    g.set_vertex_filter(None)
    g.set_vertex_filter(g.vp['traversable'])
    g.set_edge_filter(g.ep['traversable'])
    sn = find_nearest_node(g, start)
    gn = find_nearest_node(g, goal)
    return find_path_astar(g, sn, gn)


@deprecated(details="use path_from_positions(g, start, goal) and poly_from_path(g, path, eps=...) instead")
def path_poly(g, start, goal, eps=0.1):
    l = path_from_positions(g, start, goal)
    return l, poly_from_path(g, l, eps=eps)


def waypoints_through_poly(g, poly, start, goal, eps=0.01):
    """ compute a waypoints of a linear line segement path through a polygon
    
    """
    # coords will hold final waypoints
    coords = [start]
    # recompute start point if start is outside polygon
    start = point_on_border(g, poly, start)
    if start is not None:
        coords.append(start)
    
    ep = point_on_border(g, poly, goal)
    # if endpoint is outside poly, use endpoint on border and append goal
    if ep is not None:
        straight_path = compute_straight_path(g, poly, coords[-1], ep)
        coords += shorten_path(g, poly, straight_path)
        coords.append(goal)
    else:
        straight_path = compute_straight_path(g, poly, coords[-1], goal)
        coords += shorten_path(g, poly, straight_path)
    coords = remove_close_points(coords, eps=eps)
    return LineString(coords)


def remove_close_points(coords, eps=0.01):
    return LineString(coords).simplify(eps).coords


def shorten_path(g, poly, coords):
    shorter = list(reversed(shorten_recursive(g, poly, coords)))
    shortest = list(reversed(shorten_recursive(g, poly, shorter)))
    return shortest


def shorten_recursive(g, poly, coords, eps=0.01):
    if len(coords) < 3: 
        return coords
    # if next segment can be dropped drop the segment
    if LineString([coords[0], coords[2]]).within(poly):
        coords = [coords[0]] + coords[2:]
        return shorten_recursive(g, poly, coords, eps=eps)
    
    if len(coords) < 4:
        return [coords[0]] + shorten_recursive(g, poly, coords[1:], eps=eps)
    
    straight_segment = compute_straight_path(g, poly, coords[1], coords[-1])
    return [coords[0]] + shorten_recursive(g, poly, straight_segment, eps=eps)


def shorten_direction(g, poly, coords):
    # stop recursion: cannot reduce 2 point line
    if len(coords) < 3: 
        return coords
    for i in range(1, len(coords)-1):
        if LineString([coords[i - 1], coords[i + 1]]).within(poly):
            jm = i + 1
            for j in range(i+2, len(coords)):
                if LineString([coords[i - 1], coords[j]]).within(poly):
                    jm = j
                else:
                    break
            ret = coords[:i]
            #ret += shorten_direction(g, poly, compute_straight_path(g, poly, coords[jm], coords[-1]))
            ret += shorten_direction(g, poly, coords[jm:])
            return ret
    # no reduction possible:
    return coords


def point_on_border(g, poly:Polygon, point:Point)->Point:
    """
    compute a linestring to the edge of the polygon
    if the point is in the polygone:
        return none
    if the point is outside:
        return point on edge
    """
    poe = None
    point = Point(point)
    if point.within(poly):
        return None
    else:
        sn = find_nearest_node(g, point)
        inner = g.vp['geometry'][sn].inner
        d = inner.exterior.project(point)
        return inner.exterior.interpolate(d)
        LS = LineString([point, g.vp['center'][sn]])
        poe = LS.intersection(poly.exterior)
        if poe.geometryType() == 'Point':
            return poe
        elif poe.geometryType() == 'LineString' and poe.is_valid:
            assert(not poe.is_empty)
            assert(poe.length < 0.01)
            return poe.representative_point()
        elif poe.geometryType() == 'MultiPoint':
            return poe[0]
        else:
            raise NotImplementedError(f"GeometryType {poe.geometryType()} not handled.")
        
    return poe


def compute_straight_path(g, poly, start, goal):
    line = LineString([start, goal])
    if line.length < 0.01 or line.within(poly):
        return [start, goal]
    if not line.intersects(poly):
        print("need to snap line")
        line = shapely.ops.snap(line, poly, 0.01)
    assert(line.intersects(poly))
    inner_line = poly.intersection(line)
    if inner_line.geometryType() == 'LineString':
        inner_line = MultiLineString([inner_line])
    elif inner_line.geometryType() == 'Point':
        inner_line = MultiLineString([])
    elif inner_line.geometryType() == 'MultiLineString':
        pass
    else:
        #print(inner_line.wkt)
        pass
    result = [i for i in inner_line if i.geometryType() == 'LineString']
    outer_line = line - poly
    if outer_line.geometryType() == 'LineString':
        outer_line = MultiLineString([outer_line])
    for ls in outer_line:
        i0 = poly.exterior.project(Point(ls.coords[0]))
        i1 = poly.exterior.project(Point(ls.coords[-1])) 
        outer_segment = shapely.ops.substring(poly.exterior, i0, i1)
        if outer_segment.geometryType() == 'LineString':
            outer_segment = MultiLineString([outer_segment])
        elif outer_segment.geometryType() == 'Point':
            continue
        result+= [i for i in outer_segment if i.geometryType() == 'LineString']

    try:
        line = shapely.ops.linemerge(MultiLineString(result))
    except AssertionError:
        print("ASSERTION ERROR")
        print(result)
    if isinstance(line, shapely.geometry.base.BaseMultipartGeometry):
        line = list(line)
        for i, ls in enumerate(line):
            for j, l2 in enumerate(line):
                if i == j:
                    continue
                if Point(ls.coords[0]).almost_equals(Point(l2.coords[-1])):
                    c = list(ls.coords)
                    c[0] = l2.coords[-1]
                    line[i] = LineString(c)
                    break
        line = shapely.ops.linemerge(line)

            
    if isinstance(line, shapely.geometry.base.BaseMultipartGeometry):
        print("failed to merge multiline-string")
        print(line.wkt)
        print(result)
        print("inner:")
        print(inner_line)
        print("outer:")
        print(outer_line)
        if not len(line):
            return [start, goal]
        assert(len(line))
        coords = [line[0].coords[0]]
        for segment in line:
            if len(segment.coords) > 1:
                coords += list(segment.coords)[1:]
        return coords

    return list(line.coords)