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

