FROM osrf/ros:noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash << ~/.bashrc"

RUN apt-get update && \
    apt-get install -y gnome-terminal

RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get install -y ros-noetic-ackermann-msgs && \
    apt-get install -y ros-noetic-joy && \
    apt-get install -y ros-noetic-geometry-msgs && \
    apt-get install -y ros-noetic-map-server

RUN python3 -m pip install --upgrade pip setuptools wheel

RUN python3 -m pip install tensorflow  blosc

ARG DEBIAN_FRONTEND=noninteractive

COPY f1tenth-rl /home/f1tenth-rl

COPY f1tenth_simulator /home/ros_ws/src/f1tenth_simulator

WORKDIR /home

CMD ["bash"]