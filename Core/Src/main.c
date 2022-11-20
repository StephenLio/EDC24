/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  back = 0,
  forward
}Direction_Typedef; //the direction of motor

typedef struct
{
  Direction_Typedef direction;
  int velocity;
}Motor_Typedef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Wheel_L 10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Motor_Typedef motor[5];

uint8_t RxBuffer[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void MOTOR_Direction(int16_t motor_number, Direction_Typedef direction);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MOTOR_Direction(1, forward);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 300);
    // u5_printf("HELLO WORLD!\n");
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
/**
  * @brief 电机正反�??
  * @param motor_number:电机序号，范�?? 1~4
  * @param direction:正反转，forward为正转，back为反�??
  */
void MOTOR_Direction(int16_t motor_number, Direction_Typedef direction)
{       
  motor[motor_number].direction = direction;
  if (direction == back)
  {
      switch (motor_number)
      {
      case 1:
        HAL_GPIO_WritePin(Motor1_in1_GPIO_Port, Motor1_in1_Pin, 1);
        HAL_GPIO_WritePin(Motor1_in2_GPIO_Port, Motor1_in2_Pin, 0);
        break;
      case 2:
        HAL_GPIO_WritePin(Motor2_in1_GPIO_Port, Motor2_in1_Pin, 1);
        HAL_GPIO_WritePin(Motor2_in2_GPIO_Port, Motor2_in2_Pin, 0);
        break;
      case 3:
        HAL_GPIO_WritePin(Motor3_in1_GPIO_Port, Motor3_in1_Pin, 1);
        HAL_GPIO_WritePin(Motor3_in2_GPIO_Port, Motor3_in2_Pin, 0);
        break;
      case 4:
        HAL_GPIO_WritePin(Motor4_in1_GPIO_Port, Motor4_in1_Pin, 1);
        HAL_GPIO_WritePin(Motor4_in2_GPIO_Port, Motor4_in2_Pin, 0);
        break;

      default:
        break;
      }
  }
  if (direction == forward)
  {
      switch (motor_number)
      {
      case 1:
        HAL_GPIO_WritePin(Motor1_in1_GPIO_Port, Motor1_in1_Pin, 0);
        HAL_GPIO_WritePin(Motor1_in2_GPIO_Port, Motor1_in2_Pin, 1);
        break;
      case 2:
        HAL_GPIO_WritePin(Motor2_in1_GPIO_Port, Motor2_in1_Pin, 0);
        HAL_GPIO_WritePin(Motor2_in2_GPIO_Port, Motor2_in2_Pin, 1);
        break;
      case 3:
        HAL_GPIO_WritePin(Motor3_in1_GPIO_Port, Motor3_in1_Pin, 0);
        HAL_GPIO_WritePin(Motor3_in2_GPIO_Port, Motor3_in2_Pin, 1);
        break;
      case 4:
        HAL_GPIO_WritePin(Motor4_in1_GPIO_Port, Motor4_in1_Pin, 0);
        HAL_GPIO_WritePin(Motor4_in2_GPIO_Port, Motor4_in2_Pin, 1);
        break;
      }
  }
}


/**
  * @brief TIM中断回调函数
  * @param htim：�?�择是哪个定时器
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    __IO uint32_t count1 = __HAL_TIM_GET_COUNTER(&htim2);//电机1的编码器计数
    __IO uint32_t count2 = __HAL_TIM_GET_COUNTER(&htim3);//电机2的编码器计数
    __IO uint32_t count3 = __HAL_TIM_GET_COUNTER(&htim4);//电机3的编码器计数
    __IO uint32_t count4 = __HAL_TIM_GET_COUNTER(&htim5);//电机4的编码器计数
    
    __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SetCounter(&htim3, 0);
    __HAL_TIM_SetCounter(&htim4, 0);
    __HAL_TIM_SetCounter(&htim5, 0);

    int speed1 = (float)count1 / 4 / 0.001 / 260 * Wheel_L;
    int speed2 = (float)count2 / 4 / 0.001 / 260 * Wheel_L;
    int speed3 = (float)count3 / 4 / 0.001 / 260 * Wheel_L;
    int speed4 = (float)count4 / 4 / 0.001 / 260 * Wheel_L;

    u5_printf("count1: %d ; speed1: %d\r\n", count1, speed1);
  }
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

#ifdef  USE_FULL_ASSERT
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
