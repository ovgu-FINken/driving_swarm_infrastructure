#!/usr/bin/python3
import math

import numpy as np


'''
Detect any TurtleBots using the LiDAR ranges.

Cluster linkage threshold -> How close two successive points must be to be considered in the same cluster
Cluster range threshold -> Points larger than this value are omitted from clusters
Cluster size threshold -> Any cluster with less than this number of points COULD be considered a TurtleBot
'''
def detect_tb_from_ranges(ranges,
                          px=0.0,
                          py=0.0,
                          pt=0.0,
                          angle_min=0.0,
                          angle_increment=1.0,
                          minimum_range=0.12,
                          object_radius=0.035,
                          cluster_linkage_threshold=0.15,
                          max_invalids_within_cluster=2,
                          cluster_range_threshold=1.5,
                          cluster_size_threshold=20):
    assert len(ranges)>0

    # Compute single-linkage clusters, where points are connected if they are closer than cluster_threshold
    last_range = None
    invalid_count = 0
    clusters = []
    cur_cluster = []
    for idx, cur_range in enumerate(ranges):
        # Record the most recently seen range
        if last_range is None and cur_range < cluster_range_threshold:
            last_range = cur_range

        # Append any points at or beyond the sensor's maximum range to the current cluster (erroneous range or range from gap in robot)
        if cur_range >= cluster_range_threshold: # Invalid
            if invalid_count > max_invalids_within_cluster:
                # Too many invalids, force a cluster split
                clusters.append(cur_cluster)
                cur_cluster = [idx]
                invalid_count = 0
            else:
                cur_cluster.append(idx)
                invalid_count += 1
            continue # Skip the threshold check between two points and add it to the cluster

        assert(last_range is not None)
        # Two successive valid ranges satisfy the cluster_linkage_threshold * (the number of invalids between them + 1)
        if np.abs(cur_range - last_range) < (cluster_linkage_threshold * (invalid_count + 1)):
            cur_cluster.append(idx)
        # If the distance is greater than cluster_linkage_threshold then start a new cluster
        else:
            clusters.append(cur_cluster)
            cur_cluster = [idx]
        invalid_count = 0 # Reset the invalid count as we have a valid range if we got here
        last_range = cur_range

    # Append the final cluster
    clusters.append(cur_cluster)

    # If the last cluster is close to the first cluster, connect them
    invalids_between = 0
    first_valid = -1
    for idx in range(len(clusters[0])):
        if ranges[clusters[0][idx]] < cluster_range_threshold:  # Valid
            first_valid = idx
            break  # Found
        else:
            invalids_between += 1

    last_valid = len(clusters[-1])
    for idx in range(len(clusters[-1]) - 1, -1, -1):
        if ranges[clusters[-1][idx]] < cluster_range_threshold:  # Valid
            last_valid = idx
            break  # Found
        else:
            invalids_between += 1

    # An entire cluster is all invalid or the first and last cluster are the same, skip connecting
    if len(clusters) < 2 or first_valid == -1 or last_valid == len(clusters[-1]):
        pass
    elif (invalids_between <= max_invalids_within_cluster
          and np.abs(ranges[clusters[0][first_valid]] - ranges[clusters[-1][last_valid]]) <
          (cluster_linkage_threshold * (invalids_between + 1))):
        clusters[0] = clusters[-1] + clusters[0]
        clusters = clusters[:-1]

    # Remove all invalid points from each cluster
    clusters = [[i for i in cluster if ranges[i] < cluster_range_threshold] for cluster in clusters]
    # Remove entirely invalid clusters
    clusters = [c for c in clusters if len(c) > 0]

    # For each cluster, compute the mean angle and range of the valid points in the cluster (the positions of the TurtleBots)
    tb_center_points = []
    for clust_idx in range(len(clusters)):
        # Ignore clusters that are too big
        if len(clusters[clust_idx]) > cluster_size_threshold:
            continue

        cluster_ranges = [ranges[idx] for idx in clusters[clust_idx]]
        cr = np.median(cluster_ranges)

        # The median range should be less than the cluster_range_threshold
        assert cr < cluster_range_threshold

        # The median range should be more than the minimum_range of the LiDAR sensor
        if cr < minimum_range:
            continue

        # The actual centre of the robot is a radius from the median range cr
        cr = cr + object_radius

        if len(clusters[clust_idx]) > 2 * object_radius * 180 / (math.pi * (cr-0.05)) + 3:
            continue
        if len(clusters[clust_idx]) < 2 * object_radius * 180 / (math.pi * cr) - 3:
            continue
        if len(clusters[clust_idx]) < 3:
            continue
        cluster_positions = [get_xy_from_scan(ci, ranges[ci], px, py, pt, angle_min, angle_increment) for ci in clusters[clust_idx] if ranges[ci] < 1.5]
        cluster_center = np.mean(cluster_positions, axis=0)
        mean_distance_to_center = np.mean([np.linalg.norm(p - cluster_center) for p in cluster_positions])

        if mean_distance_to_center > 0.05:
            continue
        tb_center_points.append(cluster_center)
    return tb_center_points


def get_xy_from_scan(i, r, px=0.0, py=0.0, pt=0.0, angle_min=0.0, angle_increment=1.0):
    angle = angle_min + i * angle_increment + pt
    x = r * np.cos(angle) + px
    y = r * np.sin(angle) + py
    return x, y
