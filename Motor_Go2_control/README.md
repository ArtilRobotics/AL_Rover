# Control de Motores - Proyecto de Movimiento

Este repositorio contiene una colección de programas en C++ para controlar motores y sistemas móviles en un entorno robótico. Cada archivo cumple una función específica, desde pruebas iniciales de comunicación hasta menús interactivos de control de movimiento.

---

## Archivos Principales

### `arms_control.cpp`
- Programa principal para subir motores en pasos definidos.
- Al ejecutarlo, despliega un **menú interactivo** que permite controlar el movimiento paso a paso.
- Se utilizó un incremento de **0.2 radianes por paso** durante las pruebas.
- Ideal para calibración precisa de los motores del brazo.

### `block_comm_demo.cpp`
- Primer archivo utilizado para **comunicación básica con el motor**.
- Sirve como referencia para establecer la conexión inicial y enviar comandos simples.
- Útil para validar conexiones y protocolos.

### `control_mecanum.cpp`
- Control del **carro con ruedas mecanum** mediante **entrada por teclado**.
- Permite movimientos omnidireccionales: adelante, atrás, laterales y rotaciones.
- Excelente para pruebas de movilidad o teleoperación básica.

### `menu_motor_move.cpp`
- Ofrece un **menú completo** para mover motores individualmente en todos sus modos de operación: posición, velocidad, corriente, etc.
- Permite una selección detallada del motor y del tipo de control a aplicar.
- Diseñado para pruebas exhaustivas o ajustes individualizados.

### `identifica_u2d2.cpp`
- Herramienta de utilidad para identificar el **puerto serie** de cada adaptador **U2D2** conectado.
- Muy útil en entornos con múltiples interfaces o cuando existen problemas de conexión.
- Ayuda a depurar y configurar correctamente los puertos de comunicación.

---

## Compilación con CMake

Este proyecto está estructurado para ser compilado con **CMake**, lo cual permite una compilación organizada y portátil.

### 1. Crear el directorio de compilación:
```bash
mkdir build
cd build

