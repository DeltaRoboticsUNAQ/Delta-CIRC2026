/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Variables para conteo de pulsos, tiempo, revoluciones, velocidad y posicion
// Estas variables son actualizadas en las interrupciones y usadas para calcular la velocidad y posicion del encoder
// el tipo volatile se usa para indicar al compilador que estas variables pueden ser modificadas por interrupciones y no deben ser optimizadas
volatile int cont=0;
volatile int cont1=0;
volatile float tim=0.0;
volatile float rev=0.0;
volatile float vel=0.0;
volatile float pos=0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int uart2_write(int ch);
int __io_putchar(int ch);
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
int uart2_write(int ch){
	while(!(USART2 -> SR & USART_SR_TXE)){}

	USART2->DR = (ch & 0xFF);
	return ch;
}

int __io_putchar(int ch){
	uart2_write(ch);
	return ch;
}

/*
 * Aqui podras ver los 4 modos de lecturas de encoder en codigo comentado
 *
 * Este proyecto esta configurado para modo 4x, este modo usa 2 interrupciones externas que detectan
 * flancos de subida y bajada
 *
 * Para modo 2x configuras 2 interrupciones externas pero solo en modo de deteccion de flancos de bajada
 * GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; desde CUBEMX
 *
 * Para modo 1x configura 1 interrupcion externa que solo detecte flancos de bajada (canal A del encoder)
 * y un input (canal B del encoder)
 *
 * NOTA: las interrupciones externas y los inputs deben detener tambien un Pull-up
 *
 */



// MODO 4X

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//lectura en modo 4x; revisar configuracion en gpio
	// se dectectan flancos de subida y bajada
	if(GPIO_Pin == GPIO_PIN_6){
        // Leer el estado del Canal B para determinar direccion
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET){
            cont++;
        } else {
            cont--;
        }
	}
	if(GPIO_Pin == GPIO_PIN_7){
	        // Leer el estado del Canal B para determinar direccion
	        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET){
	            cont--;
	        } else {
	            cont++;
	        }
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM3){ // interrupcion por timer cada 15ms
		tim+=0.015f; //sampling time
		rev=(float)cont/(4.0*495.0); //4(modo 4x) 495(PPR depende del motor)
		vel=(((float)cont-(float)cont1)*60.0)/(4.0*495.0*0.015);
		cont1=cont;
		pos=(((float)cont)/(4.0*495.0))*360.0;
		printf("%.2f, %.2f, %.2f\n\r", tim, pos, vel);
	}

}

//MODO 2X

/*
 * Para el modo 2x el codigo es exactamente igual al de 4x
 * la diferencia es como declaramos los pines en gpio
 * para 4x detectan flancos de subidas y bajas mientras que en 2x solo flancos de bajada.
 *
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_6){
        //* Leer el estado del Canal B para determinar direccion
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET){
            cont++;
        } else {
            cont--;
        }
	}
	if(GPIO_Pin == GPIO_PIN_7){
	        //* Leer el estado del Canal B para determinar direccion
	        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET){
	            cont--;
	        } else {
	            cont++;
	        }
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM3){ // interrupcion por timer cada 15ms
		tim+=0.015f;//sampling time
		rev=(float)cont/(2.0*495.0); //2(modo 2x) 495(PPR depende del motor)
		vel=(((float)cont-(float)cont1)*60.0)/(2.0*495.0*0.015);
		cont1=cont;
		printf("%.2f, %.2f, %.2f\n\r", tim, rev, vel);
	}

} */


//MODO 1x

/*
 * Este es el modo mas sencillo es util muchas veces como en control de velocidad
 * declaramos solo un pin con interrupcion externa y el otro como input
 * normalmente canal A es EXTI(interrupcion externa) y canal b input(determina direccion de giro)
 *
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_6){
		//* Leer el estado del Canal B para determinar direccion
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET){
			cont++;
		} else {
			cont--;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM3){ // interrupcion por timer cada 15ms
		tim+=0.015f;//sampling time
		rev=(float)cont/(1.0*495.0); //1(modo 1x) 495(PPR depende del motor)
		vel=(((float)cont-(float)cont1)*60.0)/(1.0*495.0*0.015);
		cont1=cont;
		printf("%.2f, %.2f, %.2f\n\r", tim, rev, vel);
	}
 */



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
