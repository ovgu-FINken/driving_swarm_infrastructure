#!/usr/bin/python3
import math

import numpy as np


def detect_tb_from_ranges(ranges,
                          px=0.0,
                          py=0.0,
                          pt=0.0,
                          angle_min=0.0,
                          angle_increment=1.0,
                          cluster_linkage_threshold=0.15,
                          cluster_range_threshold=1.5,
                          cluster_size_threshold=30):
    # detect turtlebot in scan ranges
    # cluster_label = np.zeros_like(ranges, dtype=np.int32)

    # compute single-linkage clusters, where points are connected if they are closer than cluster_threshold
    last_range = None
    clusters = []
    cur_cluster = []
    for idx, r in enumerate(ranges):
        if last_range is None:
            last_range = r
        # include invalid measurements (more important for the real robots)
        # TODO: Think about how this might impact the centre of a cluster (perhaps switch this to None or NAN)
        # THING = float('nan')
        if r > 9.0:
            cur_cluster.append(idx)
            continue
        if np.abs(r - last_range) < cluster_linkage_threshold:
            cur_cluster.append(idx)
        else:  # if the distance is greater then start a new cluster
            clusters.append(cur_cluster)
            cur_cluster = [idx]
        last_range = r
    clusters.append(cur_cluster)

    # TODO: Offset from where the bots are detected and the actual robot is (due to the points being closer than the centre)

    # TODO: The robots have to be closer than the further away points to either side (error)

    # TODO: Add a edge case for invalid points appearing in the cluster
    # if r[-1] and r[0] are close, connect the clusters
    if np.abs(ranges[clusters[0][0]] - ranges[clusters[-1][-1]]) < cluster_linkage_threshold:
        clusters[0] = clusters[-1] + clusters[0]
        clusters = clusters[:-1]
        pass

    # compute cluster sizes
    # cluster_sizes = np.bincount(cluster_label)
    # cluster_sizes = [int(cluster_sizes[i]) for i in range(len(cluster_label)) if cluster_sizes[i] > 0]
    #
    # print(cluster_sizes)

    # We want to clean the clusters so that invalid points at the start or end are removed from the clusters
    invalid_cutoff = 1.5
    for clust_idx in range(len(clusters)):  # For each cluster
        # There are some invalid contained here
        if any(ranges[clusters[clust_idx][i]] > invalid_cutoff for i in range(len(clusters[clust_idx]))):
            # The entire cluster is invalid -> remove it
            if all(ranges[clusters[clust_idx][i]] > invalid_cutoff for i in range(len(clusters[clust_idx]))):
                clusters[clust_idx] = None
                continue

            # Some invalid, but not all
            # Let's check the start of the cluster
            first_valid = -1 # This must happen because there are some invalid, but not all
            for idx in range(len(clusters[clust_idx])): # For each idx in the cluster
                if ranges[clusters[clust_idx][idx]] <= invalid_cutoff: # Valid
                    first_valid = idx
                    break # Found
                pass
            clusters[clust_idx] = clusters[clust_idx][first_valid:]

            # Let's check the end of the cluster
            first_valid = len(clusters[clust_idx])
            for idx in range(len(clusters[clust_idx]) - 1, 0, -1): # For each idx in the cluster
                if ranges[clusters[clust_idx][idx]] <= invalid_cutoff: # Valid
                    first_valid = idx
                    break # Found
                pass
            clusters[clust_idx] = clusters[clust_idx][:first_valid + 1]
            pass
        else: # All valid -> check the next cluster
            continue

    clusters = [c for c in clusters if c is not None]

    # for each cluster, compute the mean angle and range
    # cluster_center_index = np.zeros(len(clusters), dtype=np.int32)
    tb_center_points = []
    # this stores the detected positions of the turtlebots
    for clust_idx in range(len(clusters)):
        # Ignore clusters that are too small
        if len(clusters[clust_idx]) > cluster_size_threshold:
            continue
        # Get the lowest(first) index of the cluster
        first_idx = clusters[clust_idx][0]
        # TODO: solve the edge case of the cluster wrapping around 0 degrees (min_idx should be first index)

        # cluster_center_index[clust_idx] = (first_idx + int(len(clusters[clust_idx]) / 2)) % len(ranges)
        cluster_ranges = [ranges[idx] for idx in clusters[clust_idx]]
        cr = np.median(cluster_ranges)
        # TODO: EDGE CASE, this might be invalid.
        if cr > cluster_range_threshold:
            continue
        # TODO: Modify this to be more smart
        if len(clusters[clust_idx]) > 0.0695 * 360 / (2 * math.pi * (cr-0.05)) + 3:
            continue
        if len(clusters[clust_idx]) < 0.0695 * 360 / (2 * math.pi * cr) - 3:
            continue
        if len(clusters[clust_idx]) < 3:
            continue
        cluster_positions = [get_xy_from_scan(ci, ranges[ci], px, py, pt, angle_min, angle_increment) for ci in clusters[clust_idx] if ranges[ci] < 1.5]
        cluster_center = np.mean(cluster_positions, axis=0)
        mean_distance_to_center = np.mean([np.linalg.norm(p - cluster_center) for p in cluster_positions])

        if mean_distance_to_center > 0.05:
            continue
        tb_center_points.append(cluster_center)
        # TODO: Edge case where there is an invalid point in the middle of the turtle bot cluster
    return tb_center_points


def get_xy_from_scan(i, r, px=0.0, py=0.0, pt=0.0, angle_min=0.0, angle_increment=1.0):
    angle = angle_min + i * angle_increment + pt
    x = r * np.cos(angle) + px
    y = r * np.sin(angle) + py
    return x, y
