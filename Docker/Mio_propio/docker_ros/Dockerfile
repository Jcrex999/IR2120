# Definir la distribución de ROS
ARG ROS_DISTRO=melodic

# Usar la imagen base de ROS con la distribución especificada
FROM ros:${ROS_DISTRO}-perception
ARG ROS_DISTRO

# Copiar el contenido del directorio actual al contenedor
COPY . /stack

# Instalar dependencias necesarias
RUN apt update && apt install -y --no-install-recommends \
    curl \
    g++ \
    gcc \
    git \
    python3-catkin-tools \
    python3-pip \
    ros-${ROS_DISTRO}-ros-babel-fish \
    ros-${ROS_DISTRO}-rosdoc-lite \
    libxtst6 \
    libx11-xcb1 \
    libnss3 \
    libxcomposite1 \
    libasound2 \
    libxrender1 \
    libxrandr2 \
    && rm -rf /var/lib/apt/lists/*

# Instalar dependencias de rosdep
RUN rosdep update && \
    rosdep install --rosdistro ${ROS_DISTRO} --from-paths /stack --ignore-src --default-yes \
    && rm -rf /root/.ros/rosdep/sources.cache

# Instalar paquetes de Python
RUN python3 -m pip install --upgrade pip && \
    pip install sentry-sdk && \
    pip install opencv-python

# Crear un workspace de ROS
RUN mkdir -p /home/niryo/catkin_ws

RUN apt update && apt install -y unzip && rm -rf /var/lib/apt/lists/*

# Descargar y descomprimir NiryoStudio
RUN curl -L -o NiryoStudio.zip "https://s3-niryo-public.s3.eu-west-3.amazonaws.com/niryo_studio/v4.1.2/NiryoStudio-linux-x64_v4.1.2.zip" \
    && unzip NiryoStudio.zip \
    && rm NiryoStudio.zip \
    && mv dist-app/NiryoStudio-linux-x64 /opt/NiryoStudio \
    && chown root:root /opt/NiryoStudio/chrome-sandbox \
    && chmod 4755 /opt/NiryoStudio/chrome-sandbox

# Exponer puertos necesarios
EXPOSE 9090 11311

# Copiar el script de entrada
COPY docker_ros_entrypoint.sh /docker_ros_entrypoint.sh
RUN chmod +x /docker_ros_entrypoint.sh

# Definir el punto de entrada
ENTRYPOINT [ "/docker_ros_entrypoint.sh" ]

# Crear un nuevo usuario con nombre 'user' y contraseña 'root'
RUN useradd -ms /bin/bash user && echo "user:root" | chpasswd

# Otorgar permisos de sudo al nuevo usuario
RUN usermod -aG sudo user

# Permitir que el usuario 'user' use sudo sin contraseña
RUN echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
