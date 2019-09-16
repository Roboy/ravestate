#!/bin/bash

set -x

# Note: This file is designed to be run inside the ravestate docker container.

. ~/melodic_ws/devel/setup.bash

neo4j start
roscore &
cd /redis_db && redis-server &

echo "-----------------------------------------------"
echo "Sleeping 10 s to wait for neo4j and roscore ..."
echo "-----------------------------------------------"
sleep 10

cd /root/melodic_ws/src/face_oracle
python3 ws_server.py --redis-host "$REDIS_HOST" --redis-pass "$REDIS_PASSWORD" &
python webcam_video_processor.py -i $FACEORACLE_VIDEO_DEVICE -q ws://localhost:8765 &
bash
