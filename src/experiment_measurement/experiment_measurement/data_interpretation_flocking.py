# TODO: Behavior Observation

### Obstacle Avoidance
# * (min, average, max) inter-robot distance: x_pose, y_pose
# * (min, average, max) minimum distance to scan point, i.e. can be another robot or wall: min_obstacle_dist

### Flock Formation
# * cohesion, i.e. deviation of robots' positions: x_pose, y_pose
# * alignment, i.e. deviation of robots' orientations: pose_theta
# * mean number of neighbors: n_neighbors

### Navigation to Target
# * __target distance__, i.e. distance of swarm center (mean of the robots' positions) from target: x_pose, y_pose
# * Travelled distance of the swarm center: x_pose, y_pose (approximation over time)
# * accuracy, i.e. difference between avg direction of robots and __target direction__
# * Time until target reached ???

