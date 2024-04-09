/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_INA_Pin GPIO_PIN_0
#define M1_INA_GPIO_Port GPIOC
#define M2_INA_Pin GPIO_PIN_1
#define M2_INA_GPIO_Port GPIOC
#define M3_INA_Pin GPIO_PIN_2
#define M3_INA_GPIO_Port GPIOC
#define M4_INA_Pin GPIO_PIN_3
#define M4_INA_GPIO_Port GPIOC
#define Encoder_M3_A_Pin GPIO_PIN_0
#define Encoder_M3_A_GPIO_Port GPIOA
#define Encoder_M3_B_Pin GPIO_PIN_1
#define Encoder_M3_B_GPIO_Port GPIOA
#define PWM_M3_Pin GPIO_PIN_2
#define PWM_M3_GPIO_Port GPIOA
#define PWM_M4_Pin GPIO_PIN_3
#define PWM_M4_GPIO_Port GPIOA
#define PWM_M1_Pin GPIO_PIN_5
#define PWM_M1_GPIO_Port GPIOA
#define Encoder_M2_A_Pin GPIO_PIN_6
#define Encoder_M2_A_GPIO_Port GPIOA
#define Encoder_M2_B_Pin GPIO_PIN_7
#define Encoder_M2_B_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define Encoder_M4_A_Pin GPIO_PIN_6
#define Encoder_M4_A_GPIO_Port GPIOC
#define Encoder_M4_B_Pin GPIO_PIN_7
#define Encoder_M4_B_GPIO_Port GPIOC
#define Encoder_M1_A_Pin GPIO_PIN_8
#define Encoder_M1_A_GPIO_Port GPIOA
#define Encoder_M1_B_Pin GPIO_PIN_9
#define Encoder_M1_B_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define PWM_M2_Pin GPIO_PIN_9
#define PWM_M2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
