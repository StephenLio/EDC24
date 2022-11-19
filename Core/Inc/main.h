/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder4_A_Pin GPIO_PIN_0
#define Encoder4_A_GPIO_Port GPIOA
#define Encoder4_B_Pin GPIO_PIN_1
#define Encoder4_B_GPIO_Port GPIOA
#define Encoder2_A_Pin GPIO_PIN_6
#define Encoder2_A_GPIO_Port GPIOA
#define Encoder2_B_Pin GPIO_PIN_7
#define Encoder2_B_GPIO_Port GPIOA
#define Motor2_in1_Pin GPIO_PIN_1
#define Motor2_in1_GPIO_Port GPIOB
#define Motor2_in2_Pin GPIO_PIN_2
#define Motor2_in2_GPIO_Port GPIOB
#define Motor1_in1_Pin GPIO_PIN_10
#define Motor1_in1_GPIO_Port GPIOB
#define Motor1_in2_Pin GPIO_PIN_11
#define Motor1_in2_GPIO_Port GPIOB
#define Motor3_in1_Pin GPIO_PIN_14
#define Motor3_in1_GPIO_Port GPIOB
#define Motor3_in2_Pin GPIO_PIN_15
#define Motor3_in2_GPIO_Port GPIOB
#define Motor4_in1_Pin GPIO_PIN_6
#define Motor4_in1_GPIO_Port GPIOC
#define Motor4_in2_Pin GPIO_PIN_7
#define Motor4_in2_GPIO_Port GPIOC
#define Motor1_pwm_Pin GPIO_PIN_8
#define Motor1_pwm_GPIO_Port GPIOA
#define Motor2_pwm_Pin GPIO_PIN_9
#define Motor2_pwm_GPIO_Port GPIOA
#define Motor3_pwm_Pin GPIO_PIN_10
#define Motor3_pwm_GPIO_Port GPIOA
#define Motor4_pwm_Pin GPIO_PIN_11
#define Motor4_pwm_GPIO_Port GPIOA
#define Encoder1_A_Pin GPIO_PIN_15
#define Encoder1_A_GPIO_Port GPIOA
#define Encoder1_B_Pin GPIO_PIN_3
#define Encoder1_B_GPIO_Port GPIOB
#define Encoder3_A_Pin GPIO_PIN_6
#define Encoder3_A_GPIO_Port GPIOB
#define Encoder3_B_Pin GPIO_PIN_7
#define Encoder3_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
