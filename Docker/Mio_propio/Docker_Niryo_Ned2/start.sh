#!/bin/bash
# Configurar ROS
source /opt/ros/melodic/setup.bash

# Ejecutar NiryoStudio sin sandboxing
/opt/NiryoStudio/niryo-studio --no-sandbox &

# Dejar un shell abierto
exec "$@"

# Para correr el contenedor
# docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros_melodic_niryo bash