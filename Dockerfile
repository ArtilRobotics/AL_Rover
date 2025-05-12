# Base ROS 2 Humble image
FROM ros:humble

# Variables de entorno necesarias para ROS 2
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV ROS_NAMESPACE=""
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Instalar herramientas y dependencias del sistema
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    libserialport-dev \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-launch \
    ros-humble-launch-ros \
    git \
    && rm -rf /var/lib/apt/lists/*

# Crear directorio de trabajo
WORKDIR /root/ws_motor_controller

# Copiar tu código fuente
COPY . ./src/motor_controller

# Compilar el workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select motor_controller

# Fuente del entorno de ROS 2 al iniciar el contenedor
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws_motor_controller/install/setup.bash" >> ~/.bashrc

# Comando por defecto
CMD ["bash"]
