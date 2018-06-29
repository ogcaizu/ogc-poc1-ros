FROM ros:kinetic
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

RUN apt-get update && apt-get upgrade -y && \
    mkdir -p /opt/ros_ws/src && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /opt/ros_ws/src
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace"
WORKDIR /opt/ros_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make"
