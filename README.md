# Delta-CIRC2026 Core Repository

![Status](https://img.shields.io/badge/Status-In%20Development-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-orange)
![Firmware](https://img.shields.io/badge/Firmware-STM32-brightgreen)

Contenedor principal para el software, firmware y control del rover/robot para la competencia CIRC 2026. Este repositorio contiene el ecosistema de ROS 2, código embebido para STM32 y las ramificaciones modulares del proyecto.

## 📁 Arquitectura del Proyecto

Este repositorio está altamente modularizado. El código se divide no solo lógicamente en carpetas, sino estratégicamente en **Git Branches** para un flujo de trabajo continuo (CI/CD) aislando cada subsistema y unificándolo de forma segura.

### Estructura de Directorios (Branch main)

""	ext
📦 Delta-CIRC2026
 ┣ 📂 src              # Espacio de trabajo (Workspace) principal de ROS 2
 ┃ ┣ 📂 arm           # Paquetes ROS 2 para el control del brazo robótico (Cinemática, MoveIt)
 ┃ ┗ 📂 chassis       # Paquetes ROS 2 para navegación, odometría, teleoperación de la base
 ┣ 📂 firmware         # Firmware embebido de bajo nivel
 ┃ ┗ 📂 stm32         # Código C/C++ STM32Cube, RTOS, micro-ROS para tarjetas de control
 ┣ 📂 hardware         # Archivos de electrónica y diseño mecánico
 ┃ ┣ 📂 cad           # Archivos CAD (.step, Fusion360, SolidWorks)
 ┃ ┗ 📂 pcb           # Esquemáticos y PCBs en KiCad/Altium
 ┣ 📂 docs             # Documentación técnica, manuales, pinouts
 ┗ 📂 scripts          # Scripts bash/python para instalación y utilidades (Docker, sh)
""

## 🌿 Estrategia de Ramas (Git Branching Model)

Este repositorio emplea un esquema multi-branch enfocado a sistemas robóticos complejos:

* **main**: Rama de integración final. Contiene código funcional y probado para simulaciones y hardware real. Toda actualización requiere un Pull Request (PR) y Code Review.
* **rm**: Rama orientada al desarrollo exclusivo de la física, cinemática inversa, MoveIt, sensórica y el firmware local del brazo robótico.
* **chassis**: Rama para el desarrollo del ecosistema de navegación autónoma, control de tracción, SLAM, y percepcion visual de la base motriz.
* **stm32**: Rama de desarrollo dedicada únicamente a los firmwares, periféricos (I2C, SPI, CAN, UART) e integración con micro-ROS.

## 🚀 Guía de Inicio Rápido (Quick Start)

### 1. Clonar el repositorio
""ash
git clone https://github.com/Organization/Delta-CIRC2026.git
cd Delta-CIRC2026
""

### 2. Seleccionar el módulo a trabajar
Dependiendo del equipo donde aportes, cambia de branch:
""ash
git checkout arm      # Para desarrollo de manipulador
git checkout chassis  # Para desarrollo de la base/navegación
git checkout stm32    # Para desarrollo embebido
""

## 🛠 Entorno de Desarrollo y Dependencias
- **ROS 2:** Humble Hawksbill o superior (Ubuntu 22.04).
- **Firmware:** STM32CubeIDE o PlatformIO.
- **Middleware:** micro-ROS para integración directa nodo-microcontrolador.

## 🤝 Contribuciones y Reglas
1. **Nunca** hacer push directo a main.
2. Siempre crear features desde la rama correspondiente de tu subsistema (git checkout -b feature/mi-nueva-funcionalidad arm).
3. Crear un **Pull Request** detallando el impacto del cambio, hardware utilizado y pruebas realizadas.

