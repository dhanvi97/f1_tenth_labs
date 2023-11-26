xhost +local:root 
docker container prune -f 
docker run --privileged --rm -it \
    --name="rrt_lab" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --network host \
    -v "$(pwd)lab6_pkg:/home/ros_ws/src/lab6_pkg" \
    rrt_lab bash
