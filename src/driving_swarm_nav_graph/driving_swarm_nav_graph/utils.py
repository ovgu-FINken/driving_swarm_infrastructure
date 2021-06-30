import numpy as np
from scipy.spatial import Voronoi
import shapely.geometry
from shapely.geometry import Polygon, Point, MultiLineString, LineString
import shapely.ops
import matplotlib.pyplot as plt
from dataclasses import dataclass
from graph_tool.all import * #noqa
from skimage import io, measure
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

def path_poly(g, start, goal, eps=0.1):
    g.set_edge_filter(None)
    g.set_vertex_filter(None)
    g.set_vertex_filter(g.vp['traversable'])
    g.set_edge_filter(g.ep['traversable'])
    sn = find_nearest_node(g, start)
    gn = find_nearest_node(g, goal)
    visitor = NavVisitor(g.new_vertex_property("bool"), g.new_edge_property("bool"), gn)
    _, pred = astar_search(g,
        sn,
        g.ep['dist'],
        visitor,
        heuristic=lambda v: np.linalg.norm(np.array(g.vp['center'][v]) - np.array(g.vp['center'][gn]))
    )

    l = pred_to_list(g, pred, sn, gn)
    poly = [g.vp['geometry'][p].inner.buffer(eps) for p in l]
    poly += [g.ep['geometry'][g.edge(*e)].borderPoly.buffer(eps) for e in zip(l[:-1], l[1:])]
    poly = shapely.ops.unary_union(poly).buffer(-eps).simplify(eps)
    
    return l, poly

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