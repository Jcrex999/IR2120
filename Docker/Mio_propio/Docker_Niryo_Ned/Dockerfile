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
    iputils-ping \
    unzip && rm -rf /var/lib/apt/lists/*

# Agregar repositorio de ROS y claves como root
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Instalar ROS Melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Crear un nuevo usuario
RUN useradd -ms /bin/bash user

# Cambiar a este nuevo usuario
#USER user

# Configurar el entorno de ROS
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Instalar dependencias de Python y pyniryo
RUN python3 -m pip install --upgrade pip \
    && pip3 install pyniryo

# Crear un directorio temporal para descargar NiryoStudio
#WORKDIR /home/user

# Descargar y descomprimir NiryoStudio
RUN curl -L -o NiryoStudio.zip "https://s3-niryo-public.s3.eu-west-3.amazonaws.com/niryo_studio/v4.1.2/NiryoStudio-linux-x64_v4.1.2.zip" \
    && unzip NiryoStudio.zip \
    && rm NiryoStudio.zip \
    && mv dist-app/NiryoStudio-linux-x64 /opt/NiryoStudio \
    && chown root:root /opt/NiryoStudio/chrome-sandbox \
    && chmod 4755 /opt/NiryoStudio/chrome-sandbox



# Crear un script de arranque para facilitar el uso del contenedor
COPY start.sh /start.sh
RUN chmod +x /start.sh

# Definir el punto de entrada
ENTRYPOINT ["/start.sh"]

# Cambiar a este nuevo usuario
USER user
