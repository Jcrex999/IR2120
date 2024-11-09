#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Ejecutar NiryoStudio sin sandboxing
/opt/NiryoStudio/niryo-studio --no-sandbox &

CATKIN_WS_PATH=/home/user/niryo/catkin_ws

mkdir -p ${CATKIN_WS_PATH};
cd ${CATKIN_WS_PATH};
exec "$@"
