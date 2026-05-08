/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Código combinado actuadores + RoboClaw
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RoboClaw.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Direcciones RoboClaw
#define RC_ADDR_BASE   0x81
#define RC_ADDR_WRIST  0x80

// Timeout de seguridad para RoboClaw (7 segundos)
#define TIMEOUT_MS 7000

#define RX_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

// Variables para actuadores lineales (ADC/PWM)
uint16_t adc_val[2];      // Buffer ADC (DMA)

int pos1 = 0;             // Posición objetivo actuador 1
int pos2 = 0;             // Posición objetivo actuador 2

float kp = 0.8f;          // Ganancia proporcional

int e1 = 0;               // Error actuador 1
int e2 = 0;               // Error actuador 2

int duty1 = 0;            // Duty cycle actuador 1
int duty2 = 0;            // Duty cycle actuador 2

// Variables para RoboClaw
RoboClaw_HandleTypeDef rcBase;
RoboClaw_HandleTypeDef rcWrist;

uint32_t lastCommandTime = 0;
bool motorsRunning = false;

// Scheduler por timers
volatile uint8_t flag_control = 0;
volatile uint8_t flag_print = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// Funciones UART/printf
int uart2_write(int ch);
int __io_putchar(int ch);
int _read(int file, char *ptr, int len);
//funcion UART Ros
void parseROSCommand(char *cmd);
//feedback to ros
void sendROSFeedback(void);

// Funciones actuadores lineales
void forwardAct(int actuator, int duty);
void backwardAct(int actuator, int duty);
void actStop(void);

// Funciones RoboClaw
void stopAllRoboClaw(void);
void processCommand(uint8_t ch);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //Inicializacion actuadores lineales
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 2);

  // Iniciar timers scheduler
  HAL_TIM_Base_Start_IT(&htim4);   // control loop
  HAL_TIM_Base_Start_IT(&htim2);   // impresión debug

  // Inicializacion RoboClaw
  RoboClaw_Init(&rcBase, &huart1, 100);
  RoboClaw_Init(&rcWrist, &huart1, 100);

  // Detener todos los motores al inicio
  actStop();
  stopAllRoboClaw();

  // Mensaje de bienvenida
  printf("\r\n========================================\r\n");
  printf("		Control Base STM32\r\n");
  printf("========================================\r\n");
  printf("Controles Actuadores Lineales:\r\n");
  printf("  1/2/3 - Actuador 1: 25%%/50%%/75%%\r\n");
  printf("  4/5/6 - Actuador 2: 25%%/50%%/75%%\r\n");
  printf("  0 - Ambos a 0%%\r\n");
  printf("  9 - Ambos a 100%%\r\n");
  printf("----------------------------------------\r\n");
  printf("Controles RoboClaw:\r\n");
  printf("  A/D - Base izquierda/derecha\r\n");
  printf("  W/S - Muneca subir/bajar\r\n");
  printf("  Q/E - Muneca rotar izq/der\r\n");
  printf("  ESPACIO - Detener RoboClaw\r\n");
  printf("  X - Detener TODO\r\n");
  printf("----------------------------------------\r\n");
  printf("Auto-stop RoboClaw: 7 segundos\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      // Control de actuadores
      if(flag_control==1)
      {
          flag_control = 0;

          int current1 = (adc_val[0] * 100) / 4095;
          int current2 = (adc_val[1] * 100) / 4095;

          // Calcular error
          e1 = pos1 - current1;
          e2 = pos2 - current2;

          // ACTUADOR 1
          if(abs(e1) < 2){
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
          }else{
              duty1 = (int)(kp * abs(e1));
              if(duty1 > 49) duty1 = 49;

              if(e1 > 0){
                  forwardAct(1, duty1);
              }else{
                  backwardAct(1, duty1);
              }
          }

          // ACTUADOR 2
          if(abs(e2) < 2){
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
          }else{
              duty2 = (int)(kp * abs(e2));
              if(duty2 > 49) duty2 = 49;

              if(e2 > 0){
                  forwardAct(2, duty2);
              }else{
                  backwardAct(2, duty2);
              }
          }

          //Revisar comandos UART ROS
          uint8_t ch;

          if(HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
          {
        	  if(ch == '\n')
        	  {
        	      if(rx_index > 0)
        	      {
        	          rx_buffer[rx_index] = 0;
        	          parseROSCommand(rx_buffer);
        	          rx_index = 0;
        	      }
        	  }
        	  else if(ch != '\r')
        	  {
        	      if(rx_index < RX_BUFFER_SIZE-1)
        	      {
        	          rx_buffer[rx_index++] = ch;
        	      }
        	      else
        	      {
        	          rx_index = 0;
        	      }
        	  }
          }

          //Timeout RoboClaw
          if (motorsRunning && (HAL_GetTick() - lastCommandTime > TIMEOUT_MS))
          {
              stopAllRoboClaw();
              motorsRunning = false;
              printf(">> AUTO-STOP RoboClaw (timeout 7s)\r\n");
          }
      }

      //  Impresion debug
      if(flag_print==1)
      {
          flag_print = 0;

          sendROSFeedback();
      }

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Funciones UART/printf
int uart2_write(int ch){
    while(!(USART2->SR & USART_SR_TXE)){}
    USART2->DR = (ch & 0xFF);
    return ch;
}

int __io_putchar(int ch){
    uart2_write(ch);
    return ch;
}

int _read(int file, char *ptr, int len){
    if (file == STDIN_FILENO){
        int count = 0;
        while(count < len){
            uint8_t ch;
            if(HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY) != HAL_OK)
                break;
            ptr[count++] = (char)ch;
            if(ch == '\n' || ch == '\r') break;
        }
        return count;
    }
    return -1;
}

// Funciones Actuadores Lineales
void forwardAct(int actuator, int duty){
    if(actuator == 1){
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    else if(actuator == 2){
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
}

void backwardAct(int actuator, int duty){
    if(actuator == 1){
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else if(actuator == 2){
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
}

void actStop(void){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

// Funciones RoboClaw
void stopAllRoboClaw(void)
{
    RoboClaw_ForwardM1(&rcBase, RC_ADDR_BASE, 0);
    RoboClaw_ForwardM1(&rcWrist, RC_ADDR_WRIST, 0);
    RoboClaw_ForwardM2(&rcWrist, RC_ADDR_WRIST, 0);
}

//Procesador de comandos unificado
void processCommand(uint8_t ch)
{
    char key = (char)ch;

    //  Comandos Actuadores Lineales
    if(key == '1') { pos1 = 25; printf(">> Act1 -> 25%%\r\n"); }
    else if(key == '2') { pos1 = 50; printf(">> Act1 -> 50%%\r\n"); }
    else if(key == '3') { pos1 = 75; printf(">> Act1 -> 75%%\r\n"); }
    else if(key == '4') { pos2 = 25; printf(">> Act2 -> 25%%\r\n"); }
    else if(key == '5') { pos2 = 50; printf(">> Act2 -> 50%%\r\n"); }
    else if(key == '6') { pos2 = 75; printf(">> Act2 -> 75%%\r\n"); }
    else if(key == '0') { pos1 = 0; pos2 = 0; printf(">> Ambos -> 0%%\r\n"); }
    else if(key == '9') { pos1 = 100; pos2 = 100; printf(">> Ambos -> 100%%\r\n"); }

    // Comandos RoboClaw
    // DETENER RoboClaw CON ESPACIO
    else if (key == ' ')
    {
        stopAllRoboClaw();
        motorsRunning = false;
        printf(">> RoboClaw DETENIDO\r\n");
    }

    // DETENER TODO CON X
    else if (key == 'X' || key == 'x')
    {
        stopAllRoboClaw();
        actStop();
        pos1 = 0;
        pos2 = 0;
        motorsRunning = false;
        printf(">> TODO DETENIDO\r\n");
    }

    // BASE - Izquierda/Derecha
    else if (key == 'A' || key == 'a')
    {
        RoboClaw_ForwardM1(&rcBase, RC_ADDR_BASE, 60);
        lastCommandTime = HAL_GetTick();
        motorsRunning = true;
        printf(">> Base IZQ\r\n");
    }
    else if (key == 'D' || key == 'd')
    {
        RoboClaw_BackwardM1(&rcBase, RC_ADDR_BASE, 60);
        lastCommandTime = HAL_GetTick();
        motorsRunning = true;
        printf(">> Base DER\r\n");
    }

    // MUÑECA - Subir/Bajar
    else if (key == 'W' || key == 'w')
    {
    	RoboClaw_ForwardBackwardMixed(&rcWrist, RC_ADDR_WRIST, 30);
        lastCommandTime = HAL_GetTick();
        motorsRunning = true;
        printf(">> Muneca SUBIR\r\n");
    }
    else if (key == 'S' || key == 's')
    {
    	RoboClaw_ForwardBackwardMixed(&rcWrist, RC_ADDR_WRIST, -30);
        lastCommandTime = HAL_GetTick();
        motorsRunning = true;
        printf(">> Muneca BAJAR\r\n");
    }

    // MUÑECA - Rotar
    else if (key == 'Q' || key == 'q')
    {
    	RoboClaw_LeftRightMixed(&rcWrist, RC_ADDR_WRIST, -30);
        lastCommandTime = HAL_GetTick();
        motorsRunning = true;
        printf(">> Muneca ROT IZQ\r\n");
    }
    else if (key == 'E' || key == 'e')
    {
    	RoboClaw_LeftRightMixed(&rcWrist, RC_ADDR_WRIST, 30);
        lastCommandTime = HAL_GetTick();
        motorsRunning = true;
        printf(">> Muneca ROT DER\r\n");
    }
}

// Timer Scheduler
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		flag_control = 1;
	}

	else if(htim->Instance == TIM2)
	{
		flag_print = 1;
	}
}

void parseROSCommand(char *cmd)
{

	if(strncmp(cmd,"CMD:",4)!=0)
		return;
    float base = 0;
    float w1 = 0;
    float w2 = 0;
    float a1 = 0;
    float a2 = 0;

    int parsed = sscanf(cmd,
        "CMD:B:%f;W1:%f;W2:%f;A1:%f;A2:%f",
        &base, &w1, &w2, &a1, &a2);

    if(parsed == 5)
    {
    	 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // LED blink
        // BASE
        if(base > 0)
            RoboClaw_ForwardM1(&rcBase, RC_ADDR_BASE, base);
        else
            RoboClaw_BackwardM1(&rcBase, RC_ADDR_BASE, base);

        // WRIST M1
        if(w1 > 0)
            RoboClaw_ForwardM1(&rcWrist, RC_ADDR_WRIST, w1);
        else
            RoboClaw_BackwardM1(&rcWrist, RC_ADDR_WRIST, w1);

        // WRIST M2
        if(w2 > 0)
            RoboClaw_ForwardM2(&rcWrist, RC_ADDR_WRIST, w2);
        else
            RoboClaw_BackwardM2(&rcWrist, RC_ADDR_WRIST, w2);

        // ACTUADORES
        pos1 = a1;
        pos2 = a2;

        lastCommandTime = HAL_GetTick();
        motorsRunning = true;
    }
}

void sendROSFeedback(void)
{
    int baseTicks = 0;
    int w1Ticks = 0;
    int w2Ticks = 0;

    int pot1 = adc_val[0];
    int pot2 = adc_val[1];

    printf("FB:BE:%d;W1:%d;W2:%d;P1:%d;P2:%d\n",
            baseTicks,
            w1Ticks,
            w2Ticks,
            pot1,
            pot2);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
