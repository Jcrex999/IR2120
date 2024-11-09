ARG ROS_DISTRO=noetic

FROM ros:${ROS_DISTRO}-perception
ARG ROS_DISTRO

COPY . /stack

RUN apt update && apt install -y --no-install-recommends \
    curl \
    g++-aarch64-linux-gnu\
    gcc-aarch64-linux-gnu  \
    git  \
    python3-catkin-tools  \
    python3-pip  \
    unzip  \
    ros-${ROS_DISTRO}-ros-babel-fish \
    ros-${ROS_DISTRO}-rosdoc-lite  \
    && rosdep update  \
    && rosdep install --rosdistro ${ROS_DISTRO} --from-paths /stack --ignore-src --default-yes --skip-keys="python3-rpi.gpio python-rpi.gpio" \
    && rm -rf /var/lib/apt/lists/* /root/.ros/rosdep/sources.cache

RUN python3 -m pip install --upgrade pip \
    && pip install sentry-sdk \
    && pip install opencv-python

RUN mkdir -p /home/niryo/catkin_ws;

# Descargar y descomprimir NiryoStudio
RUN curl -L -o NiryoStudio.zip "https://s3-niryo-public.s3.eu-west-3.amazonaws.com/niryo_studio/v4.1.2/NiryoStudio-linux-x64_v4.1.2.zip" \
    && unzip NiryoStudio.zip \
    && rm NiryoStudio.zip \
    && mv dist-app/NiryoStudio-linux-x64 /opt/NiryoStudio \
    && chown root:root /opt/NiryoStudio/chrome-sandbox \
    && chmod 4755 /opt/NiryoStudio/chrome-sandbox

# Rosbridge suite
EXPOSE 9090

# Expose ros master port
EXPOSE 11311

COPY .docker/docker_ros_entrypoint.sh /opt/docker_ros_entrypoint.sh
ENTRYPOINT [ "/opt/docker_ros_entrypoint.sh" ]
