FROM ros:kinetic
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

ENV PYTHONUNBUFFERED 1

COPY ./secrets/ca.crt /opt/secrets/ca.crt
COPY ./entrypoint.sh /opt/entrypoint.sh
COPY ./rosbridge /opt/ros_ws/src/rosbridge
COPY ./office_guide_robot /opt/ros_ws/src/office_guide_robot
COPY ./external_camera /opt/ros_ws/src/external_camera
RUN apt-get update && apt-get upgrade -y && apt-get install python-pip python-dev gettext-base -y && \
    cd /opt/ros_ws/src/rosbridge && \
    pip install -r requirements/common.txt && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /opt/ros_ws/src
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace"
WORKDIR /opt/ros_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make"
