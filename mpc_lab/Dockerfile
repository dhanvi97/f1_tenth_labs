FROM osrf/ros:foxy-desktop

RUN echo "source /opt/ros/foxy/setup.bash << ~/.bashrc"

RUN apt-get update && \
    apt-get install -y gnome-terminal && \
    apt-get install -y git && \
    apt-get install -y gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends && \
    apt-get install -y swig --install-recommends && \
    apt-get install coinor-libipopt-dev -y && \
    apt-get install python3-pip -y

ARG DEBIAN_FRONTEND=noninteractive

COPY mpc /home/ros_ws/src/mpc

RUN git clone https://github.com/casadi/casadi.git -b main casadi && \
    cd casadi && \
    mkdir build && \
    cd build && \
    cmake -D WITH_PYTHON=ON -D WITH_PYTHON3=ON -DWITH_IPOPT=ON -DWITH_THREAD=ON .. && \
    make install

RUN apt-get install libyaml-cpp-dev -y

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; cd /home/ros_ws; rosdep install --from-paths /home/ros_ws/src --ignore-src -r -y"

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; cd /home/ros_ws; colcon build"



RUN echo "source /home/ros_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/ros_ws

CMD ["bash"]