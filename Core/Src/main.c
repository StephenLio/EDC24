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

typedef struct
{
	float Kp, Ki, Kd;
	float P, I, D;
	float Error_Last;	
}PID_Typedef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PID_MAX 1000//改成自己设定的PWM波的单波�?长输出时�?
#define PID_MIN -1000//改成自己设定的PWM波的单波�?长输出时间的相反�?

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Wheel_L 20
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Motor_Typedef motor[5];

PID_Typedef *pid_speed;
PID_Typedef *pid_rotate;
PID_Typedef *pid_ordinate;

uint8_t set_speed;
uint8_t set_angle;
uint8_t set_position_x, set_position_y;


uint8_t u4_RxBuffer[10];
uint8_t u5_Rxbuffer[10];//[0] for pid status:1 for speed, 2 for ordinate, 3 for rotate, [5][6] for set
uint8_t Rx_u4_len = 10;
uint8_t Rx_u5_len = 6;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void MOTOR_Direction(int16_t motor_number, Direction_Typedef direction);
void MOTOR_SpeedRead();
void MOTOR_Speedcontrol(int16_t motor_number, Direction_Typedef direction, float target_speed);

void PID_Init(PID_Typedef *pid, float p_set, float i_set, float d_set);
void PID_Clear(PID_Typedef *pid);
int PID_Calculate(PID_Typedef *pid, float set_value, float now_value );

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

  HAL_UART_Receive_DMA(&huart4, u4_RxBuffer, Rx_u4_len);
  HAL_UART_Receive_IT(&huart5, u5_Rxbuffer, Rx_u5_len);

  PID_Init(pid_speed, 20, 0.5, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);    
      // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
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
  * @brief the direction of motor
  * @param motor_number: the number of motor 1~4
  * @param direction: forward or back
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
 * @brief read the speed of motors, stored in motor[].velocity
*/
void MOTOR_SpeedRead()
{
    __IO uint32_t count1 = __HAL_TIM_GET_COUNTER(&htim2);//电机1的编码器计数
    __IO uint32_t count2 = __HAL_TIM_GET_COUNTER(&htim3);//电机2的编码器计数
    __IO uint32_t count3 = __HAL_TIM_GET_COUNTER(&htim4);//电机3的编码器计数
    __IO uint32_t count4 = __HAL_TIM_GET_COUNTER(&htim5);//电机4的编码器计数
    
    __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SetCounter(&htim3, 0);
    __HAL_TIM_SetCounter(&htim4, 0);
    __HAL_TIM_SetCounter(&htim5, 0);

    motor[1].velocity = (float)count1 / 4 / 0.001 / 260 * Wheel_L;
    motor[2].velocity = (float)count2 / 4 / 0.001 / 260 * Wheel_L;
    motor[3].velocity = (float)count3 / 4 / 0.001 / 260 * Wheel_L;
    motor[4].velocity = (float)count4 / 4 / 0.001 / 260 * Wheel_L;
    u5_printf("%d\n", motor[1].velocity);
}

/**
  * @brief Initialize PID
  * @param pid: the pid controller
  * @param p_set: Kp of a specific pid controller
  * @param i_set: Ki of a specific pid controller
  * @param d_set: Kd of a specific pid controller
*/
void PID_Init(PID_Typedef *pid, float p_set, float i_set, float d_set)
{
  pid->Kp = p_set;
  pid->Ki = i_set;
  pid->Kd = d_set;
}

/**
 * @brief clear the former status of a pid controller
 * @param pid: the pid controller
*/
void PID_Clear(PID_Typedef *pid)
{
  pid->P = 0;
  pid->I = 0;
  pid->D = 0;
  pid->Error_Last = 0;
}


/**
 * @brief Calculate PID
 * @param pid: which pid controller
 * @param set_value: the target value
 * @param now_value: the current value
 * @return the pwm to be applied
*/
int PID_Calculate(PID_Typedef *pid, float set_value, float now_value )
{
	pid->P = set_value - now_value;
	pid->I += pid->P;
	pid->D = pid->P - pid->Error_Last;
	pid->Error_Last = pid->P;
	pid->I=pid->I>1000?1000:(pid->I<(-1000)?(-1000):pid->I);
	if( set_value == 0 )			pid->I = 0;
	return(pid->Kp*pid->P  +  pid->Ki*pid->I  +  pid->Kd*pid->D);
}



/**
 * @brief control the speed of motors
 * @param motor_number: number of the motor, range 1~4
 * @param direction: forward or back
 * @param target_speed: target speed to reach
*/
void MOTOR_Speedcontrol(int16_t motor_number, Direction_Typedef direction, float target_speed)
{
  MOTOR_Direction(motor_number, direction);
  MOTOR_SpeedRead();
  switch (motor_number)
  {
    case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PID_Calculate(pid_speed, target_speed, motor[1].velocity)); break;   
    case 2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PID_Calculate(pid_speed, target_speed, motor[1].velocity)); break;  
    case 3: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PID_Calculate(pid_speed, target_speed, motor[1].velocity)); break;  
    case 4: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PID_Calculate(pid_speed, target_speed, motor[1].velocity)); break;
  }
}




/**
  * @brief TIM interrupt function
  * @param htim：choose the timer that cause the interrupt
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    MOTOR_Direction(1, forward);
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if(huart->Instance == UART5)
  {
    // PID_Typedef *a;
    // switch ((int)u5_Rxbuffer[0])
    // {
    //   case 1: a = pid_speed; set_speed = u5_Rxbuffer[4];  break;
    //   case 2: a = pid_ordinate;  break;
    //   case 3: a = pid_rotate; break;
    // }
    // u5_printf("Kp: Ki: Kd: ");
    // HAL_UART_Transmit(&huart5, (uint8_t*)u5_Rxbuffer, Rx_u5_len, HAL_MAX_DELAY);
    // PID_Init(a, u5_Rxbuffer[1], u5_Rxbuffer[2], u5_Rxbuffer[3]);
    // HAL_UART_Receive_IT(&huart5, (uint8_t*)u5_Rxbuffer, Rx_u5_len);
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
