#!/usr/bin/bash
source ~/.bashrc

git fetch
HEADHASH=$(git rev-parse HEAD)
UPSTREAMHASH=$(git rev-parse robot-release@{upstream})

if [ "$HEADHASH" != "$UPSTREAMHASH"  ]
then
    # new changes
    git pull
    sh /home/turtle/driving_swarm_infrastructure/robot/install.sh
fi

sh /home/turtle/driving_swarm_infrastructure/robot/run.sh
