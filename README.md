# ⚡ Módulo STM32 (Firmware Embebido)

Esta rama está dedicada al desarrollo de firmwares, gestión de periféricos (I2C, SPI, CAN, UART) y la integración directa con micro-ROS para la tarjeta de control base de CIRC 2026.

## 📋 Plan de Actividades y Roadmap

- [ ] Setup e inicialización de STM32CubeMX, FreeRTOS y librerías estáticas de micro-ROS. Configuración UART/USB.
- [ ] Estructuración y prioridades de FreeRTOS. Integrar interrupciones por E-stop.
- [ ] Desarrollo de cálculo PID por interrupción de timer. Asignación total de Publishers / Subscribers.
- [ ] Gestión de envío de IMU / Encoders al Agent, evitando cuellos de botella en FreeRTOS.
- [ ] Servicio automatizado Systemd del Agent en PC. Volcados de fallos de STM32 hacia la UI (batería).
- [ ] Pruebas de latencia bajo alta carga de operaciones simultáneas motor/brazo/camaras.
- [ ] Protección de hardware en Utah (Temperatura/Polvo) y blindaje de código firmware.
