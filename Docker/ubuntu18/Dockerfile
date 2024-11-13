# Utilizar Ubuntu 18.04 como imagen base
FROM ubuntu:18.04

# Configurar el entorno no interactivo y variables del entorno gráfico
ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color

# Actualizar el sistema e instalar herramientas básicas
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    curl wget git vim build-essential lsb-release gnupg2 apt-utils unzip sudo

RUN apt-get install -y python3 python3-pip
RUN apt-get install -y software-properties-common dirmngr

# Instalar ROS Melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install -y ros-melodic-desktop-full && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Instalar catkin, rosdep y herramientas adicionales
RUN apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential && \
    apt-get install -y python-catkin-tools

# Instalar snap y Visual Studio Code
RUN apt-get install -y snapd && \
    snap install --classic code

# Instalar Gazebo
RUN apt-get install -y gazebo9 libgazebo9-dev

# Crear un directorio de trabajo
WORKDIR /workspace

# Copiar el script de instalación al contenedor
COPY install_niryo.sh /workspace/install_niryo.sh

# Otorgar permisos de ejecución al script
RUN chmod +x /workspace/install_niryo.sh

# Ejecutar el script al inicio
#CMD ["/bin/bash", "-c", "source /workspace/install_niryo.sh"]
