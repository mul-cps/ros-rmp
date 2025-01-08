#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

UNDERLAY_WS=/rmp_ws

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS ${ROS_DISTRO}"

# Source the base workspace, if built
if [ -f ${UNDERLAY_WS}/install/setup.bash ]
then
  source ${UNDERLAY_WS}/install/setup.bash
  echo "Sourced CPS RMP 220 base workspace"
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
  source /overlay_ws/install/setup.bash
  echo "Sourced CPS RMP 220 Overlay workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"