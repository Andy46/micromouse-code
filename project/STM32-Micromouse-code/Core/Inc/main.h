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
#define PMW_CS_Pin GPIO_PIN_13
#define PMW_CS_GPIO_Port GPIOC
#define PMW_MOTION_Pin GPIO_PIN_14
#define PMW_MOTION_GPIO_Port GPIOC
#define PMW_NRESET_Pin GPIO_PIN_15
#define PMW_NRESET_GPIO_Port GPIOC
#define ENCODER_A_C1_Pin GPIO_PIN_0
#define ENCODER_A_C1_GPIO_Port GPIOA
#define ENCODER_A_C2_Pin GPIO_PIN_1
#define ENCODER_A_C2_GPIO_Port GPIOA
#define MOTOR_A_1_Pin GPIO_PIN_2
#define MOTOR_A_1_GPIO_Port GPIOA
#define MOTOR_A_2_Pin GPIO_PIN_3
#define MOTOR_A_2_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define MOTOR_B_2_Pin GPIO_PIN_0
#define MOTOR_B_2_GPIO_Port GPIOB
#define MOTOR_B_1_Pin GPIO_PIN_1
#define MOTOR_B_1_GPIO_Port GPIOB
#define MOTOR_FAULT_Pin GPIO_PIN_2
#define MOTOR_FAULT_GPIO_Port GPIOB
#define BMI_CS_Pin GPIO_PIN_12
#define BMI_CS_GPIO_Port GPIOB
#define SD_DETECT_Pin GPIO_PIN_8
#define SD_DETECT_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_9
#define SD_CS_GPIO_Port GPIOA
#define BT_STATE_Pin GPIO_PIN_10
#define BT_STATE_GPIO_Port GPIOA
#define BT_ENABLE_Pin GPIO_PIN_11
#define BT_ENABLE_GPIO_Port GPIOA
#define TOF_INT_Pin GPIO_PIN_12
#define TOF_INT_GPIO_Port GPIOA
#define MOTOR_SLEEP_Pin GPIO_PIN_15
#define MOTOR_SLEEP_GPIO_Port GPIOA
#define ENCODER_B_C1_Pin GPIO_PIN_4
#define ENCODER_B_C1_GPIO_Port GPIOB
#define ENCODER_B_C2_Pin GPIO_PIN_5
#define ENCODER_B_C2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
