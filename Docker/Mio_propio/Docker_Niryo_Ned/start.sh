#!/bin/bash
# Configurar ROS
source /opt/ros/melodic/setup.bash

pip3 install -r /home/user/catkin_ws/src/requeriments_ned2.txt
cd /home/user/catkin_ws
catkin_make
source devel/setup.bash

# Ejecutar NiryoStudio en segundo plano
./opt/NiryoStudio/NiryoStudio --no-sandbox &

# roslaunch niryo_robot_bringup niryo_ned_simulation.launch

# Dejar un shell abierto
exec "$@"


# Para correr el contenedor
#  docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros_melodic_niryo_ned1 bash