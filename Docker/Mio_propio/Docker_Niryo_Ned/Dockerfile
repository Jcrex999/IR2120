# Usa Ubuntu 18.04 como base, compatible con ROS Melodic
FROM ubuntu:18.04

# Evitar interacciones durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# Actualizar e instalar dependencias básicas como root
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    python3 \
    python3-pip \
    python2.7 \
    iputils-ping \
    build-essential \
    sqlite3 \
    ffmpeg \
    git \
    unzip && rm -rf /var/lib/apt/lists/*

# Agregar repositorio de ROS y claves como root
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Instalar ROS Melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Configurar el entorno de ROS
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Instalación de paquetes necesarios de Ubuntu
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python-pymodbus \
    python-rosdistro \
    python-rospkg \
    python-rosdep-modules \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    && rm -rf /var/lib/apt/lists/*

# Instalación de paquetes específicos de ROS Melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-moveit \
    ros-melodic-controller-manager \
    ros-melodic-joint-state-controller \
    ros-melodic-effort-controllers \
    ros-melodic-tf2-web-republisher \
    ros-melodic-rosbridge-server \
    ros-melodic-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/melodic/setup.bash

RUN mkdir -p /home/user/catkin_ws/src \
    && cd /home/user/catkin_ws \
    && git clone https://github.com/NiryoRobotics/ned_ros src

# Instalar dependencias de Python y pyniryo
RUN python3 -m pip install --upgrade pip \
    && pip3 install pyniryo

# RUN pip3 install -r ned_ros/src/requeriments_ned2.txt

# Crear un directorio temporal para descargar NiryoStudio
#WORKDIR /home/user

# Descargar y descomprimir NiryoStudio
RUN curl -L -o NiryoStudio.zip "https://s3-niryo-public.s3.eu-west-3.amazonaws.com/niryo_studio/v4.1.2/NiryoStudio-linux-x64_v4.1.2.zip" \
    && unzip NiryoStudio.zip \
    && rm NiryoStudio.zip \
    && mv dist-app/NiryoStudio-linux-x64 /opt/NiryoStudio \
    && chown root:root /opt/NiryoStudio/chrome-sandbox \
    && chmod 4755 /opt/NiryoStudio/chrome-sandbox

RUN apt-get update && apt-get install -y libpthread-stubs0-dev

RUN apt-get update && apt-get install -y software-properties-common
RUN apt-add-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get update && apt-get install -y cmake
RUN apt-get update && apt-get install -y ros-melodic-rosdoc-lite


# Crear un script de arranque para facilitar el uso del contenedor
COPY start.sh /start.sh
RUN chmod +x /start.sh

# Definir el punto de entrada
ENTRYPOINT ["/start.sh"]

# Crear un nuevo usuario con nombre 'user' y contraseña 'root'
RUN useradd -ms /bin/bash user && echo "user:root" | chpasswd

# Otorgar permisos de sudo al nuevo usuario
RUN usermod -aG sudo user

# Permitir que el usuario 'user' use sudo sin contraseña
RUN echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
