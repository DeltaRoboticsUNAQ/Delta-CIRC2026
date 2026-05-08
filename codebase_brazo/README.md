# Control Base STM32 + RoboClaw

Sistema de control desarrollado para el rover de Delta Robotics (UNAQ) utilizando una STM32F411RE y controladores RoboClaw.

El proyecto integra:

- Control de actuadores lineales mediante PWM + retroalimentación analógica
- Control de motores DC mediante RoboClaw
- Differential wrist control usando mixed mode
- Scheduler basado en timers
- Interfaz UART para control manual y depuración
- Sistema de seguridad con auto-stop

---

# Hardware utilizado

## Microcontrolador
- STM32F411RE

## Drivers
- RoboClaw (control base y muñeca diferencial)

## Sensores
- Potenciómetros analógicos en actuadores lineales
- ADC + DMA para lectura continua

## Comunicación
- UART1 → RoboClaw
- UART2 → Terminal serial / control manual

---

# Arquitectura del sistema

## Actuadores lineales

Los actuadores lineales son controlados mediante:

- PWM (TIM3)
- señales de dirección GPIO
- retroalimentación por ADC

Cada actuador utiliza control proporcional simple:


duty = kp * error

donde:

error = posicion_objetivo - posicion_actual

La posición actual se obtiene mediante ADC de 12 bits y se convierte a porcentaje.

---

## RoboClaw

El sistema utiliza dos RoboClaw:

| RoboClaw | Dirección | Función |
|---       |---        |---      |
| Base     | 0x81      | Rotación base |
| Wrist    | 0x80      | Differential wrist |

---

# Differential Wrist

La muñeca utiliza dos motores conectados a un diferencial mecánico.

Se utiliza el modo mixed de RoboClaw:

## Movimiento vertical (pitch)

```c
RoboClaw_ForwardBackwardMixed()
```

Internamente:

```text
M1 = +V
M2 = +V
```

---

## Rotación

```c
RoboClaw_LeftRightMixed()
```

Internamente:

```text
M1 = +V
M2 = -V
```

Esto permite desacoplar:

- pitch
- rotación

utilizando únicamente dos motores.

---

# Scheduler

El sistema utiliza TIM4 como scheduler principal.

La ISR únicamente activa una bandera:

```c
flag_control = 1;
```

y toda la lógica se ejecuta en el loop principal.

Esto evita:

- bloqueos dentro de interrupciones
- problemas con UART
- ejecución pesada en ISR

---

# Seguridad

## Auto-stop RoboClaw

Si no se reciben comandos durante 7 segundos:

```c
TIMEOUT_MS = 7000
```

el sistema detiene automáticamente todos los motores controlados por RoboClaw.

---

## Emergency Stop

### ESPACIO
Detiene únicamente RoboClaw.

### X
Detiene:
- RoboClaw
- actuadores lineales
- control de posición

---

# Controles UART

## Actuadores lineales

| Tecla | Acción |
|---|---|
| 1 | Actuador 1 → 25% |
| 2 | Actuador 1 → 50% |
| 3 | Actuador 1 → 75% |
| 4 | Actuador 2 → 25% |
| 5 | Actuador 2 → 50% |
| 6 | Actuador 2 → 75% |
| 0 | Ambos → 0% |
| 9 | Ambos → 100% |

---

## Base

| Tecla | Acción |
|---|---|
| A | Base izquierda |
| D | Base derecha |

---

## Muñeca diferencial

| Tecla | Acción |
|---|---|
| W | Subir muñeca |
| S | Bajar muñeca |
| Q | Rotar izquierda |
| E | Rotar derecha |

---

## Seguridad

| Tecla | Acción |
|---|---|
| ESPACIO | Stop RoboClaw |
| X | Stop total |

---

# Librería RoboClaw

El proyecto utiliza una librería RoboClaw porteada de Arduino a STM32 HAL.

Características:

- CRC16
- UART HAL
- control de velocidad
- control por duty
- lectura de encoders
- PID
- mixed mode
- control de posición
- lectura de corriente y voltaje

---

# Configuración principal

## UART

| UART | Función |
|---|---|
| USART1 | RoboClaw |
| USART2 | Terminal serial |

---

## Timers

| Timer | Función |
|---|---|
| TIM3 | PWM actuadores |
| TIM4 | Scheduler |

---

# Flujo principal

```text
TIM4 interrupt
    ↓
flag_control = 1
    ↓
while(1)
    ↓
Control actuadores
    ↓
Leer UART
    ↓
Procesar comandos
    ↓
Verificar timeout
```

---

# Autor

Aquiles Montenegro  
Delta Robotics — Universidad Aeronáutica en Querétaro (UNAQ)

Proyecto desarrollado para rover de competencia CIRC.