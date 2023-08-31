#!/usr/bin/bash

COMMAND="ros2 launch ccr ccr_sim.launch.py"

SIMULATOR="gzserver"

# Loop from 1 to 10
for ((N_ROBOTS = 1; N_ROBOTS <= 10; N_ROBOTS++)); do
    echo "Running $COMMAND with N_ROBOTS=$N_ROBOTS"
    # Run the command with the current N_ROBOTS value
    $COMMAND --robots=$N_ROBOTS
done