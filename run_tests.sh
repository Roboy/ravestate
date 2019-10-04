#!/bin/bash

# Note: This file is designed to be run inside the ravestate docker container.

export PYTHONPATH=$PYTHONPATH:$(pwd)/modules
neo4j start

. ~/melodic_ws/devel/setup.bash
roscore &

echo "-----------------------------------------------"
echo "Sleeping 10 s to wait for neo4j and roscore ..."
echo "-----------------------------------------------"
sleep 10
pytest --cov=modules -p no:warnings test

neo4j stop
killall roscore
