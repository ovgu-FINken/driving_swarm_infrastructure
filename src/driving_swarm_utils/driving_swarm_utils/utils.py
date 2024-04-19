#!/usr/bin/python3

import numpy as np

def detect_tb_from_ranges(ranges,
              px=0.0,
              py=0.0,
              pt=0.0,
              angle_min=0.0,
              angle_increment=1.0,
              cluster_linkage_threshold=0.2,
              cluster_range_threshold=1.0,
              cluster_size_threshold=30):
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
        if np.abs(r - last_range) < cluster_linkage_threshold:
            clusters[i] = last_cluster
        else:
            last_cluster += 1
            clusters[i] = last_cluster
        last_range = r
    
    # if r[-1] and r[0] are close, connect the clusters
    if np.abs(ranges[-1] - ranges[0]) < cluster_linkage_threshold:
        clusters[clusters == clusters[-1]] = clusters[0]
    
    # compute cluster sizes
    cluster_sizes = np.bincount(clusters)
    
    # for each cluster, compute the mean angle and range
    cluster_center_index = np.zeros_like(cluster_sizes, dtype=np.int32)
    tb_center_points = []
    for i in range(len(cluster_sizes)):
        if cluster_sizes[i] > cluster_size_threshold:
            continue
        min_index = np.min(np.where(clusters == i)[0])
        cluster_center_index[i] = (min_index + int(cluster_sizes[i] / 2)) % len(ranges)
        cluster_ranges = [ranges[ci] for ci in range(min_index, min_index + cluster_sizes[i] % len(ranges))]
        cr = np.mean(cluster_ranges)
        if cr > cluster_range_threshold:
            continue
        if cr * cluster_sizes[i] > 30:
            continue
        if cluster_sizes[i] < 2:
            continue
        cluster_positions = [get_xy_from_scan(ci, ranges[ci], px, py, pt, angle_min, angle_increment) for ci in range(min_index, min_index + cluster_sizes[i] % len(ranges))]
        cluster_center = np.mean(cluster_positions, axis=0)
        mean_distance_to_center = np.mean([np.linalg.norm(p - cluster_center) for p in cluster_positions])
        if mean_distance_to_center > 0.1:
            continue
        tb_center_points.append(cluster_center)
    return tb_center_points

def get_xy_from_scan(self, i, r, px=0.0, py=0.0, pt=0.0, angle_min=0.0, angle_increment=1.0):
    angle = angle_min + i * angle_increment + pt
    x = r * np.cos(angle) + px
    y = r * np.sin(angle) + py
    return x,y