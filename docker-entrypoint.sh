#!/bin/bash

#################################################################################
#
# NOTE: This file is designed to be run inside the ravestate docker container via
#
#                  `docker-compose start rs-{macos|linux}`
#
#################################################################################

set -x

# Make sure ROS is sourced, also in future terminal sessions
. ~/melodic_ws/devel/setup.bash

# Prepare .bashrc
echo ". ~/melodic_ws/devel/setup.bash" >> /root/.bashrc
echo "echo '============================================================='" >> /root/.bashrc
echo "echo '>>>>>>>> Welcome to the Ravestate Docker Container <<<<<<<<<<'" >> /root/.bashrc
echo "echo '  This container is intended for development purposes only.  '" >> /root/.bashrc
echo "echo '============================================================='" >> /root/.bashrc

# Make sure Neo4j is advertised outside of the container
echo dbms.connectors.default_listen_address=0.0.0.0 >> /etc/neo4j/neo4j.conf

# Start Neo4j, Roscore, Redis
neo4j start
roscore &
cd /redis_db && redis-server &

echo "----------------------------------------------"
echo "Sleeping 10 s to wait for neo4j and roscore ..."
echo "----------------------------------------------"
sleep 10

# Start face_oracle server and client
cd /root/melodic_ws/src/face_oracle
python3 ws_server.py --redis-host "$REDIS_HOST" --redis-pass "$REDIS_PASSWORD" &
python webcam_video_processor.py -i $FACEORACLE_VIDEO_DEVICE -q ws://localhost:8765 &

# Start to keep the container open
bash
