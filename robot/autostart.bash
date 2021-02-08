#!/usr/bin/bash
# Checks for updates on the driving_swarm_repo and starts ros nodes

PROJECT_DIR="/home/turtle/driving_swarm_infrastructure"
ROBOT_DIR="${PROJECT_DIR}/robot"
ROBOT_BRANCH="robot-release"

source ~/.bashrc

cd "${PROJECT_DIR}"

git fetch
HEADHASH=$(git rev-parse HEAD)
UPSTREAMHASH=$(git rev-parse "${ROBOT_BRANCH}@{upstream}")

if [ "$HEADHASH" != "$UPSTREAMHASH"  ]
then
    # new changes
    git pull --force
    bash "${ROBOT_DIR}/install.bash"
fi

bash "${ROBOT_DIR}/run.bash"
