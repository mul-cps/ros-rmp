
########################
ARG ROS_DISTRO=noetic-robot

# Base image with ROS
FROM ros:${ROS_DISTRO} AS base

ARG UNDERLAY_WS=/rmp_ws # needs to be set after image

# Use bash shell for consistency
SHELL ["/bin/bash", "-c"]

# Set nofile limits
COPY ./30-nofilelimit.conf /etc/security/limits.d/30-nofilelimit.conf

# Set working directory
WORKDIR ${UNDERLAY_WS}

# Debug: Verify ARG and ENV values
RUN echo "ROS_DISTRO=${ROS_DISTRO}" && echo "UNDERLAY_WS=${UNDERLAY_WS}"

# Install dependencies
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    git \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rviz* \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-amcl \
    ros-${ROS_DISTRO}-gmapping \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-move-base \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-robot-pose-ekf \
    ros-${ROS_DISTRO}-laser-filters \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-rqt* && \
    echo "Dependencies installed successfully."

# Clone repository and build workspace
RUN mkdir -p src && \
    cd src && \
    echo "Cloning repository into $(pwd)" && \
    git clone https://github.com/bjoernellens1/catkin_ws_for_RMP.git && \
    git clone https://github.com/bjoernellens1/cps_rmp220_support -b ${ROS_DISTRO} && \
    git clone https://github.com/bjoernellens1/rmp220_middleware.git -b ${ROS_DISTRO} && \
    mv catkin_ws_for_RMP/RMP1.0_catkin_ws . && \
    rm -rf catkin_ws_for_RMP && \
    cd .. && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    echo "Building workspace in ${UNDERLAY_WS}" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES='segway_msgs' && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES='segwayrmp' && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES='cps_rmp220_support' && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES='rmp220_middleware' && \
    catkin_make && \
    echo "Workspace built successfully."

# Copy rmp220_middleware package
# COPY ./src/packages/rmp220_middleware src/rmp220_middleware

# Build package
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
#     catkin_make -DCATKIN_WHITELIST_PACKAGES='rmp220_middleware' && \
#     catkin_make

# Copy entrypoint script
COPY ./docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh && echo "Entrypoint script copied and made executable."

# Set environment variables for runtime
ENV ROS_DISTRO=${ROS_DISTRO}
ENV UNDERLAY_WS=${UNDERLAY_WS}

ENTRYPOINT [ "/entrypoint.sh" ]

# Navigation stage
FROM ros:${ROS_DISTRO} AS navigation

SHELL ["/bin/bash", "-c"]

# Use the same workspace
WORKDIR ${UNDERLAY_WS}

# Debug: Verify ENV values in navigation stage
RUN echo "ROS_DISTRO=${ROS_DISTRO}" && echo "UNDERLAY_WS=${UNDERLAY_WS}"

# # Create a ROS package
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
#     echo "Creating ROS package in ${UNDERLAY_WS}" && \
#     catkin_create_pkg segway_rmp_2dnav move_base segway_rmp_tf_configuration_dep segway_rmp_odom_configuration_dep segway_rmp_sensor_configuration_dep && \
#     echo "ROS package created successfully."

# Use the same entrypoint
ENTRYPOINT [ "/entrypoint.sh" ]
