
# 🤖 AL_Rover - Control de Motores para Rover en ROS 2 Humble

![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)  
![C++](https://img.shields.io/badge/language-C++-informational.svg)  

Este paquete está diseñado y **verificado para funcionar en ROS 2 Humble**. Permite el **control completo de los motores de un rover de 4 extremidades**, cada una con dos motores: **superior** y **inferior**, sumando un total de **8 motores**. 

El controlador principal está implementado en el script `motor_node.cpp`.

---

## 🚀 Lanzamiento del Nodo

Puedes iniciar el nodo principal ejecutando:

```bash
ros2 launch motor_controller motor_launch.py
```

## ⚙️ Configuración de Interfaces

Las interfaces conectadas corresponden a los dispositivos **U2D2** de Robotis. Los motores no se detectan individualmente, sino a través de estas interfaces.

### 🔌 Mapeo de Interfaces y Motores

En el archivo `motor_node.cpp`, puedes modificar las siguientes estructuras si cambian los **números de serie** o **IDs**:

```cpp
std::map<std::string, std::string> serial_to_label = {
    {"FTA7NOUS", "FL"},
    {"FT8ISETK", "FR"},
    {"FT94W6WF", "RL"},
    {"FT89FBGO", "RR"}
};

std::map<std::string, std::pair<int, int>> motor_ids = {
    {"FL", {0, 2}},
    {"FR", {1, 0}},
    {"RL", {1, 0}},
    {"RR", {1, 0}}
};
````

> ⚠️ Si una extremidad no está conectada, **los tópicos relacionados no aparecerán**.

---

## 📡 Tópicos Disponibles

### 🛞 Publicadores de Comandos

```python
# Motores inferiores (ruedas)
motors/FL_lower/cmd
motors/FR_lower/cmd
motors/RL_lower/cmd
motors/RR_lower/cmd

# Motores superiores (hombros)
motors/FL_upper/cmd
motors/FR_upper/cmd
motors/RL_upper/cmd
motors/RR_upper/cmd
```

### 📨 Tipo de Mensaje para Comando (`MotorCommand.msg`)

```plaintext
float32 torque_desired
float32 speed_desired
float32 position_desired
float32 k_pos
float32 k_spd
```

### 🛞 Suscriptores de Mensajes

```python
# Motores inferiores (ruedas)
motors/FL_lower/state
motors/FR_lower/state
motors/RL_lower/state
motors/RR_lower/state

# Motores superiores (hombros)
motors/FL_upper/state
motors/FR_upper/state
motors/RL_upper/state
motors/RR_upper/state
```

### 📥 Tópicos de Lectura del Estado del Motor

```plaintext
float32 torque
float32 speed
float32 position
float32 temperature
uint16 error
```

---

## 🧭 Control y Ejemplos

El paquete `omnidirectional_controller` contiene ejemplos prácticos de:

* Lectura de estado de los motores
* Envío de comandos de control
* Control total del rover en el archivo `all_motors.py`

---

## 🎥 Cámara Intel RealSense D435i

Para integrar la cámara con IMU, sigue estos pasos:

### 1. 📦 Instalar SDK de RealSense y drivers ROS 2

Sigue la guía oficial de instalación de `realsense2_camera`.

### 2. 🚀 Lanzar el nodo de la cámara con IMU

```bash
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1
```

### 3. 🧭 Usar filtro Madgwick para IMU

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
  -p use_mag:=false \
  -p world_frame:=enu \
  -p publish_tf:=false \
  -r imu/data_raw:=/camera/camera/imu \
  -p publish_tf:=true
```

> El parámetro `publish_tf:=true` habilita la publicación del `tf` de la IMU en RViz. Actualmente solo publica la **rotación** (no traslación).

---
