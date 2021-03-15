# TODO: Behavior Observation

### Obstacle Avoidance
# * minimum distance to scan point, i.e. can be another robot or wall: min_obstacle_dist
# * minimum inter-robot distance: x_pose, y_pose
#    --> time until collision

### Flock Formation
# * cohesion, i.e. deviation of robots' positions: mean and deviation <-- swarm center (mean of positions) <-- x_pose, y_pose
# * alignment, i.e. deviation of robots' orientations: mean and deviation of swarm <-- pose_theta
# * mean number of neighbors: n_neighbors

### Navigation to Target
# * target distance, i.e. distance of swarm center from target: difference to target pos <-- swarm center (mean of the robots' positions) <-- x_pose, y_pose
#    --> MAYBE: Time until target reached: if target distance < threshold
# * Travelled distance of the robots and the swarm center: difference to previous swarm center <-- swarm center (mean of the robots' positions) <-- x_pose, y_pose (approximation over time)


# TODO: Visualisation 
# ! for each experiment configuration:

### Obstacle Avoidance
# critical obstacle distances & collisions over time

### Flock Formation
# cohesion, and alignment over time
# n_neighbors

### Navigation to Target
# target distance over time
# trajectories of robots, and the swarm center

### Forces
