FROM osrf/ros:foxy-desktop

RUN echo "source /opt/ros/foxy/setup.bash << ~/.bashrc"

RUN apt-get update && \
    apt-get install -y gnome-terminal 

ARG DEBIAN_FRONTEND=noninteractive

COPY src /home/ros_ws/src

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; cd /home/ros_ws; colcon build"

RUN echo "source /home/ros_ws/install/setup.bash"

WORKDIR /home/ros_ws

CMD ["bash"]