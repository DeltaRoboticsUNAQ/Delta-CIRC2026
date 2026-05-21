# STM32 Firmware — Referencia de Implementación

Guía de referencia para implementar el firmware del STM32 que se comunica con
el nodo `stm32_hardware_bridge` de ROS 2.

---

## Decisión de arquitectura: micro-ROS vs UART serial

### ¿Por qué UART serie simple (esta arquitectura)?

| Criterio | UART serie simple | micro-ROS |
|---|---|---|
| Complejidad firmware | Baja | Alta |
| Requisitos de memoria | ~4 KB RAM | ~32+ KB RAM |
| Tiempo de integración | Horas | Días |
| Soporte STM32F1/F3 | ✅ Todos | ⚠️ Solo modelos con FreeRTOS y suficiente RAM |
| Depuración | Fácil (monitor serie) | Compleja |
| Latencia | ~2 ms (UART) | ~5-10 ms (DDS middleware) |
| Recomendado para | **Este proyecto (inicio)** | Proyectos con múltiples MCUs |

**Recomendación**: Comenzar con UART serie simple. Una vez que el sistema
funcione de extremo a extremo, migrar a micro-ROS si se necesitan múltiples
nodos MCU o comunicación directa con topics ROS 2 desde el microcontrolador.

---

## Protocolo de comunicación

### ROS 2 → STM32 (comando de posición)

```
CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>\r\n
```

| Campo | Descripción | Rango |
|---|---|---|
| `B` | Ángulo deseado base (grados) | −86.0 a +86.0 |
| `W1` | Ángulo motor muñeca 1 (grados) | −270.0 a +270.0 |
| `W2` | Ángulo motor muñeca 2 (grados) | −270.0 a +270.0 |
| `A1` | Extensión actuador 1 (%) | 0.0 a 100.0 |
| `A2` | Extensión actuador 2 (%) | 0.0 a 100.0 |

**Ejemplo:**
```
CMD:B:45.00;W1:30.00;W2:-10.00;A1:60.0;A2:40.0\r\n
```

### STM32 → ROS 2 (feedback de sensores)

```
FB:BE:<ticks>;W1:<ticks>;W2:<ticks>;P1:<adc>;P2:<adc>\r\n
```

| Campo | Descripción | Ejemplo |
|---|---|---|
| `BE` | Encoder base (ticks con signo) | `BE:1250` |
| `W1` | Encoder motor muñeca 1 (ticks con signo) | `W1:834` |
| `W2` | Encoder motor muñeca 2 (ticks con signo) | `W2:-278` |
| `P1` | ADC potenciómetro actuador 1 (0-4095) | `P1:2100` |
| `P2` | ADC potenciómetro actuador 2 (0-4095) | `P2:1875` |

**Ejemplo:**
```
FB:BE:1250;W1:834;W2:-278;P1:2100;P2:1875\r\n
```

---

## Arquitectura de periféricos STM32 sugerida

### Pines (STM32F446RE Nucleo como referencia)

```
Periférico          Pin STM32        Descripción
──────────────────────────────────────────────────────────────
UART2 TX            PA2              Comunicación ROS 2 (TX)
UART2 RX            PA3              Comunicación ROS 2 (RX)

TIM1 CH1/CH2        PA8/PA9          Encoder base (modo cuadratura)
TIM3 CH1/CH2        PA6/PA7          Encoder muñeca motor 1
TIM4 CH1/CH2        PB6/PB7          Encoder muñeca motor 2

ADC1 CH0            PA0              Potenciómetro actuador 1
ADC1 CH1            PA1              Potenciómetro actuador 2

TIM2 CH1 (PWM)      PA15             PWM motor base
TIM2 CH2 (PWM)      PB3              PWM motor muñeca 1
TIM2 CH3 (PWM)      PB10             PWM motor muñeca 2
TIM5 CH1 (PWM)      PA0* (o PC0)     PWM actuador 1 (si es PWM)
TIM5 CH2 (PWM)      PA1* (o PC1)     PWM actuador 2 (si es PWM)

GPIO OUT            PC0              Dirección motor base (IN1)
GPIO OUT            PC1              Dirección motor base (IN2)
GPIO OUT            PC2              Dirección muñeca M1 (IN1)
GPIO OUT            PC3              Dirección muñeca M1 (IN2)
GPIO OUT            PC4              Dirección muñeca M2 (IN1)
GPIO OUT            PC5              Dirección muñeca M2 (IN2)
GPIO OUT            PC6              Enable actuador 1
GPIO OUT            PC7              Enable actuador 2
```

> Ajustar pines según el driver de motor disponible (L298N, BTS7960, etc.)

---

## Código de referencia C (STM32 HAL)

### 1. Lectura de encoders en cuadratura

```c
// En CubeMX: configurar TIM1, TIM3, TIM4 en modo "Encoder Mode TI1 and TI2"
// Resolución: 4x PPR del encoder (ej: 500 PPR encoder → 2000 ticks/rev)

// Variables globales de posición
volatile int32_t encoder_base   = 0;
volatile int32_t encoder_wrist1 = 0;
volatile int32_t encoder_wrist2 = 0;

// Valores anteriores del contador (para detectar overflow de 16 bits)
static uint16_t prev_base   = 32768;
static uint16_t prev_wrist1 = 32768;
static uint16_t prev_wrist2 = 32768;

void update_encoders(void) {
    uint16_t curr_base   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    uint16_t curr_wrist1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t curr_wrist2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    // Diferencia con manejo de overflow (aritmética de 16 bits sin signo)
    encoder_base   += (int16_t)(curr_base   - prev_base);
    encoder_wrist1 += (int16_t)(curr_wrist1 - prev_wrist1);
    encoder_wrist2 += (int16_t)(curr_wrist2 - prev_wrist2);

    prev_base   = curr_base;
    prev_wrist1 = curr_wrist1;
    prev_wrist2 = curr_wrist2;
}
```

### 2. Lectura de potenciómetros (ADC DMA)

```c
// En CubeMX: ADC1, canales 0 y 1, DMA circular, 12 bits, 1 kHz aprox.
uint16_t adc_values[2];  // [0] = A1, [1] = A2

// En main():
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 2);

// Uso:
uint16_t pot_a1 = adc_values[0];  // 0 – 4095
uint16_t pot_a2 = adc_values[1];  // 0 – 4095
```

### 3. Control de motores DC (PWM + dirección)

```c
// En CubeMX: TIM2 en modo PWM, frecuencia 20 kHz, ARR = 1000 (resolución 0.1%)

// Función: establecer velocidad y dirección de un motor
// speed: -100 a +100 (porcentaje). Negativo = reversa.
void motor_set(TIM_HandleTypeDef *htim, uint32_t channel,
               GPIO_TypeDef *in1_port, uint16_t in1_pin,
               GPIO_TypeDef *in2_port, uint16_t in2_pin,
               int8_t speed) {
    if (speed > 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        speed = -speed;
    } else {
        // Freno activo
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    }
    uint32_t pulse = (uint32_t)(speed * 10);  // 0-1000 para ARR=1000
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}
```

### 4. Control PID simple para posición

```c
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_limit;
} PID_t;

float pid_compute(PID_t *pid, float setpoint, float measurement, float dt) {
    float error    = setpoint - measurement;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    // Anti-windup y saturación
    if (output > pid->output_limit) {
        output = pid->output_limit;
        pid->integral -= error * dt;  // no acumular cuando saturado
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
        pid->integral -= error * dt;
    }
    return output;
}

// Instancias PID (ajustar Kp, Ki, Kd con el hardware real)
PID_t pid_base   = {2.0f, 0.05f, 0.1f, 0.0f, 0.0f, 90.0f};
PID_t pid_wrist1 = {2.5f, 0.05f, 0.1f, 0.0f, 0.0f, 90.0f};
PID_t pid_wrist2 = {2.5f, 0.05f, 0.1f, 0.0f, 0.0f, 90.0f};
PID_t pid_act1   = {3.0f, 0.1f,  0.05f, 0.0f, 0.0f, 90.0f};
PID_t pid_act2   = {3.0f, 0.1f,  0.05f, 0.0f, 0.0f, 90.0f};
```

### 5. Parser del protocolo ROS 2 → STM32

```c
// Setpoints recibidos de ROS 2
volatile float cmd_base_deg   = 0.0f;
volatile float cmd_wrist1_deg = 0.0f;
volatile float cmd_wrist2_deg = 0.0f;
volatile float cmd_act1_pct   = 50.0f;
volatile float cmd_act2_pct   = 50.0f;

// Buffer de recepción UART
#define RX_BUF_SIZE 128
char rx_buf[RX_BUF_SIZE];
uint8_t rx_index = 0;

// Llamar desde callback UART (HAL_UART_RxCpltCallback)
void parse_command(const char *line) {
    // Formato: CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>
    if (strncmp(line, "CMD:", 4) != 0) return;

    float b, w1, w2, a1, a2;
    if (sscanf(line + 4,
               "B:%f;W1:%f;W2:%f;A1:%f;A2:%f",
               &b, &w1, &w2, &a1, &a2) == 5) {
        // Validar límites antes de asignar
        cmd_base_deg   = fmaxf(-86.0f, fminf(86.0f, b));
        cmd_wrist1_deg = fmaxf(-270.0f, fminf(270.0f, w1));
        cmd_wrist2_deg = fmaxf(-270.0f, fminf(270.0f, w2));
        cmd_act1_pct   = fmaxf(0.0f, fminf(100.0f, a1));
        cmd_act2_pct   = fmaxf(0.0f, fminf(100.0f, a2));
    }
}
```

### 6. Envío de feedback a ROS 2

```c
// Llamar desde el bucle principal a ~50 Hz
void send_feedback(void) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "FB:BE:%ld;W1:%ld;W2:%ld;P1:%u;P2:%u\r\n",
        (long)encoder_base,
        (long)encoder_wrist1,
        (long)encoder_wrist2,
        (unsigned)adc_values[0],
        (unsigned)adc_values[1]);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 10);
}
```

### 7. Bucle principal (main loop)

```c
// En main() después de inicialización:
const float DT = 0.01f;  // 100 Hz loop, ajustar con un timer

while (1) {
    update_encoders();

    // Convertir ticks a grados para usar con PID
    float TICKS_TO_DEG = 360.0f / TICKS_PER_REV;  // ej: 360/2000 = 0.18°/tick
    float base_deg   = encoder_base   * TICKS_TO_DEG;
    float wrist1_deg = encoder_wrist1 * TICKS_TO_DEG;
    float wrist2_deg = encoder_wrist2 * TICKS_TO_DEG;

    // Convertir ADC a porcentaje para usar con PID
    float act1_pct = (float)(adc_values[0] - POT_MIN) / (POT_MAX - POT_MIN) * 100.0f;
    float act2_pct = (float)(adc_values[1] - POT_MIN) / (POT_MAX - POT_MIN) * 100.0f;

    // Calcular PID
    float out_base   = pid_compute(&pid_base,   cmd_base_deg,   base_deg,   DT);
    float out_wrist1 = pid_compute(&pid_wrist1, cmd_wrist1_deg, wrist1_deg, DT);
    float out_wrist2 = pid_compute(&pid_wrist2, cmd_wrist2_deg, wrist2_deg, DT);
    float out_act1   = pid_compute(&pid_act1,   cmd_act1_pct,   act1_pct,   DT);
    float out_act2   = pid_compute(&pid_act2,   cmd_act2_pct,   act2_pct,   DT);

    // Comandar motores
    motor_set(&htim2, TIM_CHANNEL_1, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1, (int8_t)out_base);
    motor_set(&htim2, TIM_CHANNEL_2, GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3, (int8_t)out_wrist1);
    motor_set(&htim2, TIM_CHANNEL_3, GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_5, (int8_t)out_wrist2);
    // Para actuadores lineales: control PWM similar o señal de dirección según el driver

    // Enviar feedback a ROS 2 cada 20 ms (~50 Hz)
    static uint32_t last_fb = 0;
    if (HAL_GetTick() - last_fb >= 20) {
        send_feedback();
        last_fb = HAL_GetTick();
    }
}
```

---

## Herramientas sugeridas

| Herramienta | Uso |
|---|---|
| STM32CubeIDE o CubeMX | Generación de código HAL, configuración de periféricos |
| ST-LINK / SWD | Programación y depuración del STM32 |
| Serial monitor (Arduino IDE, minicom, screen) | Verificar protocolo serie antes de conectar a ROS 2 |
| Logic analyzer | Verificar señales PWM y UART |

---

## Calibración del sistema

Antes de conectar al brazo real:

1. **Encoder base**: Girar manualmente la base 360° y verificar que `encoder_base`
   cambia en `TICKS_PER_REV` ticks. Ajustar `base_ticks_per_rev` en el launch file.

2. **Encoders muñeca**: Verificar que W1 y W2 se incrementan/decrementan
   correctamente con la rotación de cada motor.

3. **Potenciómetros**: Con el actuador completamente retraído, registrar el valor
   ADC → `pot_min`. Extender completamente → `pot_max`. Ajustar en el launch file.

4. **PID**: Comenzar con Kp muy pequeño (0.5-1.0), Ki=0, Kd=0. Aumentar Kp
   gradualmente hasta que el sistema responda sin oscilar, luego añadir Kd.

---

## Migración futura a micro-ROS

Si en el futuro se quiere usar micro-ROS en lugar del protocolo UART:

1. Instalar micro-ROS para STM32 (requiere FreeRTOS):
   https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/

2. El STM32 publicará directamente en topics ROS 2:
   - Publica: `/hardware/joint_states` (sensor_msgs/JointState)
   - Suscribe: `/joint_states_cmd` (sensor_msgs/JointState)

3. Eliminar el nodo `stm32_hardware_bridge` del launch file ya que el STM32
   actuará como un nodo ROS 2 directamente.

**Requisito mínimo de hardware para micro-ROS**: STM32F4 o superior con
≥64 KB RAM libre después del sistema operativo (FreeRTOS).
