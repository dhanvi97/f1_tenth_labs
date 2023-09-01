#!/bin/bash
xhost +local:root 
docker container prune -f 
docker run --privileged --rm -it \
    --name="lab1" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --network host \
    -v "$(pwd)/src:/home/ros_ws/src" \
    lab1 bash