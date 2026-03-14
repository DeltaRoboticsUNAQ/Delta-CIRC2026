/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Control brazo robótico + protocolo ROS2 serial
  *
  * Protocolo ROS2 → STM32 (USART2):
  *   CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>\r\n
  *
  * Protocolo STM32 → ROS2 (USART2):
  *   FB:BE:<ticks>;W1:<ticks>;W2:<ticks>;P1:<adc>;P2:<adc>\n
  *
  * NOTA: Las líneas de debug (printf) son ignoradas por ROS2 ya que no
  *       empiezan con "FB:", por lo que coexisten sin problema.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "RoboClaw.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define RC_ADDR_BASE   0x81
#define RC_ADDR_WRIST  0x80

#define TIMEOUT_MS     7000   // Auto-stop RoboClaw si no llega comando ROS

// Ticks de encoder por grado (ajustar según hardware real)
// Ejemplo: encoder 500 PPR en cuadratura = 2000 ticks/rev → 2000/360 = 5.56 ticks/°
#define TICKS_PER_DEG_BASE   5.56f
#define TICKS_PER_DEG_WRIST  5.56f

// Ganancia P para control de posición base y muñeca (0-127 escala RoboClaw)
// Aumentar si el motor responde lento, reducir si oscila
#define BASE_KP    0.8f
#define WRIST_KP   0.8f

// Zona muerta (ticks) — no mover si el error es menor a esto
#define BASE_DEADZONE    10
#define WRIST_DEADZONE   10

// Velocidad mínima para vencer fricción estática (0-127)
#define MIN_SPEED    15

// Buffer de recepción UART
#define UART_LINE_BUF_SIZE  128
/* USER CODE END PD */

/* USER CODE BEGIN PV */
// ── Actuadores lineales ───────────────────────────────────────────────────────
uint16_t adc_val[2];   // DMA: adc_val[0]=act1, adc_val[1]=act2
int pos1 = 0;          // Target actuador 1 (0-100%)
int pos2 = 0;          // Target actuador 2 (0-100%)
float kp = 0.8f;
int e1, e2, duty1, duty2;

// ── RoboClaw ──────────────────────────────────────────────────────────────────
RoboClaw_HandleTypeDef rcBase;
RoboClaw_HandleTypeDef rcWrist;
uint32_t lastCommandTime = 0;
bool motorsRunning = false;

// ── Encoders (leídos desde RoboClaw) ─────────────────────────────────────────
int32_t enc_base = 0;
int32_t enc_w1   = 0;
int32_t enc_w2   = 0;

// ── Targets ROS (actualizados por CMD) ───────────────────────────────────────
float target_b_deg  = 0.0f;   // Base en grados
float target_w1_deg = 0.0f;   // Motor muñeca 1 en grados
float target_w2_deg = 0.0f;   // Motor muñeca 2 en grados

// ── Recepción UART en modo interrupción ──────────────────────────────────────
uint8_t uart_rx_byte;                        // byte actual (IT)
char    uart_line_buf[UART_LINE_BUF_SIZE];   // buffer de línea
uint8_t uart_line_idx = 0;
volatile uint8_t flag_cmd_ready = 0;         // línea completa recibida

// ── Flags de scheduler ────────────────────────────────────────────────────────
volatile uint8_t flag_control  = 0;   // TIM4: loop de control
volatile uint8_t flag_feedback = 0;   // TIM2: envío de feedback ROS
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
int  uart2_write(int ch);
int  __io_putchar(int ch);
void forwardAct(int actuator, int duty);
void backwardAct(int actuator, int duty);
void actStop(void);
void stopAllRoboClaw(void);
void processRosCommand(const char *line);
void sendFeedback(void);
void controlBaseWrist(void);
int  clampSpeed(float speed);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// ── UART2 printf ─────────────────────────────────────────────────────────────
int uart2_write(int ch) {
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = (ch & 0xFF);
    return ch;
}
int __io_putchar(int ch) { return uart2_write(ch); }

// ── Callback IT UART: acumula bytes hasta \n ──────────────────────────────────
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        char c = (char)uart_rx_byte;
        if (c == '\n' || c == '\r') {
            if (uart_line_idx > 0) {
                uart_line_buf[uart_line_idx] = '\0';
                flag_cmd_ready = 1;
                uart_line_idx  = 0;
            }
        } else {
            if (uart_line_idx < UART_LINE_BUF_SIZE - 1) {
                uart_line_buf[uart_line_idx++] = c;
            }
        }
        // Rearmar la recepción
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

// ── Timer scheduler ──────────────────────────────────────────────────────────
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if      (htim->Instance == TIM4) flag_control  = 1;
    else if (htim->Instance == TIM2) flag_feedback = 1;
}
/* USER CODE END 0 */

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, 2);

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim2);

    RoboClaw_Init(&rcBase,  &huart1, 100);
    RoboClaw_Init(&rcWrist, &huart1, 100);

    actStop();
    stopAllRoboClaw();

    // Arrancar recepción UART2 por interrupciones
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    printf("\r\n=== Brazo ROS2 + Control manual ===\r\n");
    printf("ROS: esperando CMD:B:...;W1:...;W2:...;A1:...;A2:...\r\n");
    printf("Manual: A/D=base  W/S=pitch  Q/E=roll  1-6=actuadores  X=stop\r\n\r\n");
    /* USER CODE END 2 */

    while (1) {

        // ── Procesar línea ROS recibida ───────────────────────────────────────
        if (flag_cmd_ready) {
            flag_cmd_ready = 0;
            char line_copy[UART_LINE_BUF_SIZE];
            strncpy(line_copy, uart_line_buf, UART_LINE_BUF_SIZE);

            if (strncmp(line_copy, "CMD:", 4) == 0) {
                processRosCommand(line_copy);
                lastCommandTime = HAL_GetTick();
                motorsRunning   = true;
            } else {
                // Comando manual de teclado (letra suelta)
                if (strlen(line_copy) == 1) {
                    uint8_t ch = (uint8_t)line_copy[0];
                    // Actuadores lineales
                    if      (ch == '1') { pos1 = 25;  printf(">> Act1 25%%\r\n"); }
                    else if (ch == '2') { pos1 = 50;  printf(">> Act1 50%%\r\n"); }
                    else if (ch == '3') { pos1 = 75;  printf(">> Act1 75%%\r\n"); }
                    else if (ch == '4') { pos2 = 25;  printf(">> Act2 25%%\r\n"); }
                    else if (ch == '5') { pos2 = 50;  printf(">> Act2 50%%\r\n"); }
                    else if (ch == '6') { pos2 = 75;  printf(">> Act2 75%%\r\n"); }
                    else if (ch == '0') { pos1 = 0; pos2 = 0; printf(">> Ambos 0%%\r\n"); }
                    else if (ch == '9') { pos1 = 100; pos2 = 100; printf(">> Ambos 100%%\r\n"); }
                    // RoboClaw manual
                    else if (ch=='A'||ch=='a') { RoboClaw_ForwardM1(&rcBase, RC_ADDR_BASE, 60);      lastCommandTime=HAL_GetTick(); motorsRunning=true; printf(">> Base IZQ\r\n"); }
                    else if (ch=='D'||ch=='d') { RoboClaw_BackwardM1(&rcBase, RC_ADDR_BASE, 60);     lastCommandTime=HAL_GetTick(); motorsRunning=true; printf(">> Base DER\r\n"); }
                    else if (ch=='W'||ch=='w') { RoboClaw_ForwardBackwardMixed(&rcWrist, RC_ADDR_WRIST,  30); lastCommandTime=HAL_GetTick(); motorsRunning=true; printf(">> Pitch +\r\n"); }
                    else if (ch=='S'||ch=='s') { RoboClaw_ForwardBackwardMixed(&rcWrist, RC_ADDR_WRIST, -30); lastCommandTime=HAL_GetTick(); motorsRunning=true; printf(">> Pitch -\r\n"); }
                    else if (ch=='Q'||ch=='q') { RoboClaw_LeftRightMixed(&rcWrist, RC_ADDR_WRIST, -30);       lastCommandTime=HAL_GetTick(); motorsRunning=true; printf(">> Roll -\r\n"); }
                    else if (ch=='E'||ch=='e') { RoboClaw_LeftRightMixed(&rcWrist, RC_ADDR_WRIST,  30);       lastCommandTime=HAL_GetTick(); motorsRunning=true; printf(">> Roll +\r\n"); }
                    else if (ch==' ')          { stopAllRoboClaw(); motorsRunning=false; printf(">> STOP RoboClaw\r\n"); }
                    else if (ch=='X'||ch=='x') { stopAllRoboClaw(); actStop(); pos1=0; pos2=0; motorsRunning=false; printf(">> TODO STOP\r\n"); }
                }
            }
        }

        // ── Loop de control (TIM4) ────────────────────────────────────────────
        if (flag_control) {
            flag_control = 0;

            // Control actuadores lineales
            int current1 = (adc_val[0] * 100) / 4095;
            int current2 = (adc_val[1] * 100) / 4095;
            e1 = pos1 - current1;
            e2 = pos2 - current2;

            if (abs(e1) < 2) {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
            } else {
                duty1 = (int)(kp * abs(e1));
                if (duty1 > 49) duty1 = 49;
                (e1 > 0) ? forwardAct(1, duty1) : backwardAct(1, duty1);
            }

            if (abs(e2) < 2) {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            } else {
                duty2 = (int)(kp * abs(e2));
                if (duty2 > 49) duty2 = 49;
                (e2 > 0) ? forwardAct(2, duty2) : backwardAct(2, duty2);
            }

            // Control posición base y muñeca con RoboClaw
            controlBaseWrist();

            // Auto-stop si no llegan comandos ROS
            if (motorsRunning && (HAL_GetTick() - lastCommandTime > TIMEOUT_MS)) {
                stopAllRoboClaw();
                motorsRunning = false;
                printf(">> AUTO-STOP (timeout)\r\n");
            }
        }

        // ── Enviar feedback ROS (TIM2) ────────────────────────────────────────
        if (flag_feedback) {
            flag_feedback = 0;
            sendFeedback();
        }
    }
}

// ── Parsear comando ROS ───────────────────────────────────────────────────────
// Formato: CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>
void processRosCommand(const char *line) {
    float b=0, w1=0, w2=0, a1=50, a2=50;

    // Parsear cada campo
    char *p;
    char buf[UART_LINE_BUF_SIZE];
    strncpy(buf, line + 4, sizeof(buf));   // saltar "CMD:"

    char *token = strtok(buf, ";");
    while (token != NULL) {
        if      (strncmp(token, "B:",  2) == 0) b  = atof(token + 2);
        else if (strncmp(token, "W1:", 3) == 0) w1 = atof(token + 3);
        else if (strncmp(token, "W2:", 3) == 0) w2 = atof(token + 3);
        else if (strncmp(token, "A1:", 3) == 0) a1 = atof(token + 3);
        else if (strncmp(token, "A2:", 3) == 0) a2 = atof(token + 3);
        token = strtok(NULL, ";");
    }

    // Actualizar targets
    target_b_deg  = b;
    target_w1_deg = w1;
    target_w2_deg = w2;
    pos1 = (int)a1;   // 0-100%
    pos2 = (int)a2;   // 0-100%

    if (pos1 > 100) pos1 = 100;
    if (pos2 > 100) pos2 = 100;
}

// ── Control proporcional base y muñeca ───────────────────────────────────────
void controlBaseWrist(void) {
    uint32_t status;

    // Leer encoders del RoboClaw
    enc_base = RoboClaw_ReadEncM1(&rcBase,  RC_ADDR_BASE,  &status);
    enc_w1   = RoboClaw_ReadEncM1(&rcWrist, RC_ADDR_WRIST, &status);
    enc_w2   = RoboClaw_ReadEncM2(&rcWrist, RC_ADDR_WRIST, &status);

    // Control BASE
    int32_t tgt_base_ticks = (int32_t)(target_b_deg * TICKS_PER_DEG_BASE);
    int32_t err_base = tgt_base_ticks - enc_base;

    if (abs(err_base) < BASE_DEADZONE) {
        RoboClaw_ForwardM1(&rcBase, RC_ADDR_BASE, 0);
    } else {
        int spd = clampSpeed(BASE_KP * abs(err_base));
        if (err_base > 0) RoboClaw_ForwardM1 (&rcBase, RC_ADDR_BASE, spd);
        else              RoboClaw_BackwardM1(&rcBase, RC_ADDR_BASE, spd);
    }

    // Control MUÑECA — W1 y W2 son motores individuales del diferencial
    int32_t tgt_w1_ticks = (int32_t)(target_w1_deg * TICKS_PER_DEG_WRIST);
    int32_t tgt_w2_ticks = (int32_t)(target_w2_deg * TICKS_PER_DEG_WRIST);
    int32_t err_w1 = tgt_w1_ticks - enc_w1;
    int32_t err_w2 = tgt_w2_ticks - enc_w2;

    // W1 → M1 del RoboClaw muñeca
    if (abs(err_w1) < WRIST_DEADZONE) {
        RoboClaw_ForwardM1(&rcWrist, RC_ADDR_WRIST, 0);
    } else {
        int spd = clampSpeed(WRIST_KP * abs(err_w1));
        if (err_w1 > 0) RoboClaw_ForwardM1 (&rcWrist, RC_ADDR_WRIST, spd);
        else            RoboClaw_BackwardM1(&rcWrist, RC_ADDR_WRIST, spd);
    }

    // W2 → M2 del RoboClaw muñeca
    if (abs(err_w2) < WRIST_DEADZONE) {
        RoboClaw_ForwardM2(&rcWrist, RC_ADDR_WRIST, 0);
    } else {
        int spd = clampSpeed(WRIST_KP * abs(err_w2));
        if (err_w2 > 0) RoboClaw_ForwardM2 (&rcWrist, RC_ADDR_WRIST, spd);
        else            RoboClaw_BackwardM2(&rcWrist, RC_ADDR_WRIST, spd);
    }
}

// Limita velocidad al rango válido de RoboClaw con velocidad mínima
int clampSpeed(float raw) {
    int spd = (int)raw + MIN_SPEED;
    if (spd > 127) spd = 127;
    return spd;
}

// ── Enviar feedback ROS ───────────────────────────────────────────────────────
// Formato: FB:BE:<ticks>;W1:<ticks>;W2:<ticks>;P1:<adc>;P2:<adc>\n
void sendFeedback(void) {
    char fb[96];
    int len = snprintf(fb, sizeof(fb),
        "FB:BE:%ld;W1:%ld;W2:%ld;P1:%u;P2:%u\n",
        (long)enc_base, (long)enc_w1, (long)enc_w2,
        (unsigned)adc_val[0], (unsigned)adc_val[1]);

    // Enviar directo por USART2 (no usar printf para no mezclar buffers)
    for (int i = 0; i < len; i++) uart2_write(fb[i]);
}

// ── Actuadores lineales ───────────────────────────────────────────────────────
void forwardAct(int actuator, int duty) {
    if (actuator == 1) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
}

void backwardAct(int actuator, int duty) {
    if (actuator == 1) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
}

void actStop(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

void stopAllRoboClaw(void) {
    RoboClaw_ForwardM1(&rcBase,  RC_ADDR_BASE,  0);
    RoboClaw_ForwardM1(&rcWrist, RC_ADDR_WRIST, 0);
    RoboClaw_ForwardM2(&rcWrist, RC_ADDR_WRIST, 0);
}

/* SystemClock_Config y Error_Handler igual que en tu código original */
