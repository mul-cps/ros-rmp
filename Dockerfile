FROM osrf/ros:noetic-desktop-full AS rmp-base

SHELL ["/bin/bash", "-c"]

COPY ./30-nofilelimit.conf /etc/security/limits.d/30-nofilelimit.conf

WORKDIR /workspace

RUN apt update && \
    apt install -y \
    git \
    ros-noetic-rplidar-ros

RUN mkdir -p src && \
    cd src && \
    #git clone https://github.com/segwayrmp/segway_rmp.git src/
    git clone https://github.com/bjoernellens1/catkin_ws_for_RMP.git && \
    mv catkin_ws_for_RMP/RMP1.0_catkin_ws . && \
    rm -rf catkin_ws_for_RMP && \
    cd .. && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES='segway_msgs' && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES='segwayrmp'

FROM osrf/ros:noetic-desktop-full AS navigation-base
SHELL ["/bin/bash", "-c"]

WORKDIR /workspace

RUN source /opt/ros/noetic/setup.bash && \
    catkin_create_pkg segway_rmp_2dnav move_base segway_rmp_tf_configuration_dep segway_rmp_odom_configuration_dep segway_rmp_sensor_configuration_dep


FROM scratch AS export-stage

COPY --from=navigation-base /workspace .