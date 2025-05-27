# AL_Rover

## Descripción del Proyecto - AL-Rover

**AL-Rover** es un rover asistente diseñado para actuar como mayordomo robótico y soporte de operaciones para drones. Su propósito principal es asistir, transportar y levantar drones cuando sea necesario.
![image](https://github.com/user-attachments/assets/f0d1615d-d2d4-46b4-a057-082c8bfcfbcc)

El sistema mecánico del AL-Rover utiliza motores de **Unitree Go2** en cada una de sus extremidades. Cada articulación del cuadrupedo posee un ID:

- **ID 0**: Motor superior de cada articulación (usado también en los hombros)
- **ID 1**: Motor intermedio, que conecta la estructura principal con la pata
- **ID 2**: Motor más cercano a la pata
![Motor_Parts_Go2](https://github.com/user-attachments/assets/7cc6fb0c-ce3d-47c2-a9f6-676e61ccebe9)

___
Para la contruccion del Rover se recomienda usar los motores de ID 0 como hombros y los de ID 1 como ruedas, aunque se recomienda seguir esta estandarización para facilitar el mantenimiento y la implementación de algoritmos de control, no es estrictamente necesario.

Para el control de los motores es obligatorio usar una velocidad de comunicación de **4 Mbps (4 000 000 baudios)**, lo cual asegura una respuesta eficiente y sin latencia.

El diseño mecánico y ensamblaje completo del AL-Rover está disponible en Onshape (privado), en el siguiente enlace:

🔗 [Diseño CAD en Onshape](https://cad.onshape.com/documents/8ab630367136c8f38a4ebdb2/w/5252c226c42ef4611ad4758d/e/cbe931ffc4bd2b8360cfe31e?renderMode=0&uiState=6835dd82368d7b3fe241a823)

## Control de los Motores

El control de los motores del AL-Rover se realiza a través del dispositivo **U2D2**, el cual actúa como interfaz entre la PC y los motores. La conexión se establece de la siguiente manera:

1. **PC ↔ U2D2**: Mediante un cable USB.
2. **U2D2 ↔ Motores**: Utilizando comunicación serial **RS485**.
   
![Board](https://github.com/user-attachments/assets/7c69b7c1-f569-48c0-8df2-16507a17263e)


Este esquema permite enviar comandos de control directamente desde la PC hacia cada motor del sistema de locomoción y articulación.

Para una guía detallada sobre la configuración, control y protocolos de comunicación de los motores de Unitree, se recomienda consultar la documentación oficial en el siguiente enlace:

🔗 [Unitree Motor SDK - Developer Guide](https://support.unitree.com/home/en/Motor_SDK_Dev_Guide/overview)

___
## Control del Rover

En esta versión del proyecto, **AL-Rover** está listo para ser controlado mediante un joystick de **Xbox**, utilizando el script `control.cpp` escrito en **C++**. Este script permite:

- **Movimiento omnidireccional** del rover.
- **Levantamiento y descenso** del cuerpo en incrementos definidos.
- Control mediante **modos de posición y velocidad**.
- Utilización del **modo Damping** para resistencia pasiva y estabilidad.

### 🌀 Modo Damping

> *Damping mode is a special velocity mode. When we set W = 0.0, the motor will maintain a shaft speed of 0. When rotated by external forces, an impedance torque is generated in the opposite direction, proportional to the rotational speed. Once the force is removed, the motor holds its current position. This resembles a linear damper—hence the name.*

Este modo es especialmente útil para **bloquear articulaciones** pasivamente, aportando seguridad y estabilidad al sistema.

---

### Identificación de Interfaces Seriales

Cada U2D2 conectado a una articulación se identifica mediante su **número de serie**, lo cual debe actualizarse en el código en caso de cambios de hardware. En `control.cpp`, se configuran así:

```cpp
std::map<std::string, std::string> logical_serials = {
    {"FR", "FT8ISETK"},
    {"FL", "FT7WBEJZ"},
    {"RR", "FT89FBGO"},
    {"RL", "FT94W6WF"}
};
````

También deben ajustarse los **IDs de los motores** correspondientes a los hombros:

```cpp
std::map<std::string, int> label_to_motor_id = {
    {"FL", 0}, {"FR", 1}, {"RL", 1}, {"RR", 1}
};
```

Y finalmente, los comandos se envían según esta configuración a las ruedas, que se deben modificar igual en caso de cambio:

```cpp
send_motor_cmd("FL", ports[logical_serials["FL"]], 1, w1);
send_motor_cmd("FR", ports[logical_serials["FR"]], 0, -w2);
send_motor_cmd("RL", ports[logical_serials["RL"]], 0, w3);
send_motor_cmd("RR", ports[logical_serials["RR"]], 0, -w4);
```

---

### Scripts adicionales

Para pruebas y configuraciones previas, se incluyen los siguientes scripts:

* `read_joystick.cpp`: Lee el joystick y publica el estado de botones y ejes.
* `menu_motor_move.cpp`: Permite mover motores individualmente desde un menú interactivo.
* `read_motors.cpp`: Lee y muestra el estado de todos los motores.
* `control_mecanum.cpp`: Control exclusivo de ruedas con movimiento mecanum.
* `arms_control.cpp`: Control exclusivo de los brazos del rover.
* `listar_puertos.cpp`: Lista todas las interfaces seriales disponibles, útil para identificar el número de serie de nuevos U2D2 conectados.

Estos archivos son fundamentales para validar el sistema antes de usar el control completo.


