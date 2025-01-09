# ARG ROS_DISTRO=noetic-robot
# ARG UNDERLAY_WS=/rmp_ws
# ARG TARGETPLATFORM

# FROM ros:${ROS_DISTRO} AS base

# SHELL ["/bin/bash", "-c"]

# COPY ./30-nofilelimit.conf /etc/security/limits.d/30-nofilelimit.conf

# WORKDIR /{UNDERLAY_WS}

# RUN apt update && \
#     apt install -y \
#     git \
#     ros-noetic-rplidar-ros

# RUN mkdir -p src && \
#     cd src && \
#     #git clone https://github.com/segwayrmp/segway_rmp.git src/
#     git clone https://github.com/bjoernellens1/catkin_ws_for_RMP.git && \
#     mv catkin_ws_for_RMP/RMP1.0_catkin_ws . && \
#     rm -rf catkin_ws_for_RMP && \
#     cd .. && \
#     source /opt/ros/noetic/setup.bash && \
#     catkin_make -DCATKIN_WHITELIST_PACKAGES='segway_msgs' && \
#     catkin_make -DCATKIN_WHITELIST_PACKAGES='segwayrmp'

# # Set up the entrypoint
# COPY ./docker/entrypoint.sh /
# RUN chmod +x /entrypoint.sh

# ENTRYPOINT [ "/entrypoint.sh" ]

# FROM ros:${ROS_DISTRO} AS navigation
# SHELL ["/bin/bash", "-c"]

# WORKDIR /{UNDERLAY_WS}

# RUN source /opt/ros/noetic/setup.bash && \
#     catkin_create_pkg segway_rmp_2dnav move_base segway_rmp_tf_configuration_dep segway_rmp_odom_configuration_dep segway_rmp_sensor_configuration_dep

# ENTRYPOINT [ "/entrypoint.sh" ]

# # Base image with ROS
# ARG ROS_DISTRO=noetic-robot
# ARG UNDERLAY_WS=/rmp_ws
# FROM ros:${ROS_DISTRO} AS base

# SHELL ["/bin/bash", "-c"]

# ENV UNDERLAY_WS=${UNDERLAY_WS}

# # Set nofile limits
# COPY ./30-nofilelimit.conf /etc/security/limits.d/30-nofilelimit.conf

# # Set working directory
# WORKDIR ${UNDERLAY_WS}

# RUN echo "workdir is: " && $UNDERLAY_WS

# # Install dependencies
# RUN apt update && apt install -y \
#     git \
#     ros-${ROS_DISTRO}-rplidar-ros \
#     ros-${ROS_DISTRO}-tf

# # # Clone repository and build workspace
# RUN mkdir -p src && \
#     cd src && \
#     git clone https://github.com/bjoernellens1/catkin_ws_for_RMP.git && \
#     mv catkin_ws_for_RMP/RMP1.0_catkin_ws . && \
#     rm -rf catkin_ws_for_RMP && \
#     cd .. && \
#     catkin_make -DCATKIN_WHITELIST_PACKAGES='segway_msgs' && \
#     catkin_make -DCATKIN_WHITELIST_PACKAGES='segwayrmp'

# # Set up the entrypoint
# COPY ./docker/entrypoint.sh /
# RUN chmod +x /entrypoint.sh

# ENTRYPOINT [ "/entrypoint.sh" ]

# # Stage for navigation
# FROM ros:${ROS_DISTRO} AS navigation

# SHELL ["/bin/bash", "-c"]

# WORKDIR ${UNDERLAY_WS}

# # Create a ROS package
# RUN source /opt/ros/noetic/setup.bash && \
#     catkin_create_pkg segway_rmp_2dnav move_base segway_rmp_tf_configuration_dep segway_rmp_odom_configuration_dep segway_rmp_sensor_configuration_dep

# # Use the same entrypoint
# ENTRYPOINT [ "/entrypoint.sh" ]

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
RUN apt update && apt install -y \
    git \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-teleop-twist-joy && \
    echo "Dependencies installed successfully."

# Clone repository and build workspace
RUN mkdir -p src && \
    cd src && \
    echo "Cloning repository into $(pwd)" && \
    git clone https://github.com/bjoernellens1/catkin_ws_for_RMP.git && \
    mv catkin_ws_for_RMP/RMP1.0_catkin_ws . && \
    rm -rf catkin_ws_for_RMP && \
    cd .. && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    echo "Building workspace in ${UNDERLAY_WS}" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES='segway_msgs' && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES='segwayrmp' && \
    echo "Workspace built successfully."

# Copy rmp220_middleware package
COPY ./src/packages/rmp220_middleware src/

# Build package
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make

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

# Create a ROS package
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    echo "Creating ROS package in ${UNDERLAY_WS}" && \
    catkin_create_pkg segway_rmp_2dnav move_base segway_rmp_tf_configuration_dep segway_rmp_odom_configuration_dep segway_rmp_sensor_configuration_dep && \
    echo "ROS package created successfully."

# Use the same entrypoint
ENTRYPOINT [ "/entrypoint.sh" ]
