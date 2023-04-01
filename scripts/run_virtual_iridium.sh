#!/bin/bash

SERVER_URL=${1:-127.0.0.1:8000}

# Make sure everything is killed on exit
trap "exit" INT TERM
trap "kill 0" EXIT

# Port environment variables are defined in $ROS_WORKSPACE/.devcontainer/base-dev/base-dev.Dockerfile
touch $LOCAL_TRANSCEIVER_TEST_PORT
touch $VIRTUAL_IRIDIUM_PORT

# Setup socat relay pair
socat -d -d -t 0 pty,raw,echo=0,link=$LOCAL_TRANSCEIVER_TEST_PORT pty,raw,echo=0,link=$VIRTUAL_IRIDIUM_PORT &

# Run Virtual Iridium
python2 $ROS_WORKSPACE/src/virtual_iridium/python/Iridium9602.py --webhook_server_endpoint $SERVER_URL \
    --http_server_port 8080 -d $VIRTUAL_IRIDIUM_PORT -m HTTP_POST
