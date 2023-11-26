#!/bin/bash
xhost +local:root
docker container prune -f
docker run --privileged --rm -it \
    --name="f1tenth-rl" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --network host \
    -v "$(pwd)/f1tenth-rl:/home/f1tenth-rl" \
    -v "$(pwd)/f1tenth_simulator:/home/ros_ws/src/f1tenth_simulator" \
    f1tenth-rl bash