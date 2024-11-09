#!/bin/bash
# Configurar ROS
source /opt/ros/melodic/setup.bash

# Ejecutar NiryoStudio en segundo plano
./opt/NiryoStudio/NiryoStudio --no-sandbox &

# Dejar un shell abierto
exec "$@"


# Para correr el contenedor
#  docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros_melodic_niryo_ned1 bash