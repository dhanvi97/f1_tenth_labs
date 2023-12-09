FROM osrf/ros:foxy-desktop
RUN echo "source /opt/ros/foxy/setup.bash << ~/.bashrc"

RUN apt-get update && \
    apt-get install -y gnome-terminal

RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get install -y ros-foxy-ackermann-msgs && \
    apt-get install -y ros-foxy-joy && \
    apt-get install -y ros-foxy-geometry-msgs

RUN python3 -m pip install --upgrade pip setuptools wheel

RUN python3 -m pip install tensorflow  blosc

ARG DEBIAN_FRONTEND=noninteractive

COPY f1tenth-rl /home/f1tenth-rl

WORKDIR /home

CMD ["bash"]