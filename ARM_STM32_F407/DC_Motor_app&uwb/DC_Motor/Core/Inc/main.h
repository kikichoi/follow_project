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
#define CHK_MC_EN_Pin GPIO_PIN_2
#define CHK_MC_EN_GPIO_Port GPIOE
#define CHK_MC_DIS_Pin GPIO_PIN_3
#define CHK_MC_DIS_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_14
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOC
#define SWITCH1_Pin GPIO_PIN_1
#define SWITCH1_GPIO_Port GPIOA
#define SWITCH2_Pin GPIO_PIN_2
#define SWITCH2_GPIO_Port GPIOA
#define A2_5_M_SO_M2_Pin GPIO_PIN_5
#define A2_5_M_SO_M2_GPIO_Port GPIOA
#define T3_1_ENC1_A_Pin GPIO_PIN_6
#define T3_1_ENC1_A_GPIO_Port GPIOA
#define T3_2_ENC1_B_Pin GPIO_PIN_7
#define T3_2_ENC1_B_GPIO_Port GPIOA
#define T1_1_ENC2_A_Pin GPIO_PIN_9
#define T1_1_ENC2_A_GPIO_Port GPIOE
#define T1_2_ENC2_B_Pin GPIO_PIN_11
#define T1_2_ENC2_B_GPIO_Port GPIOE
#define nSLEEP_M2_Pin GPIO_PIN_14
#define nSLEEP_M2_GPIO_Port GPIOB
#define T12_2_PWM_DC_M2_Pin GPIO_PIN_15
#define T12_2_PWM_DC_M2_GPIO_Port GPIOB
#define DIR_DC_M2_Pin GPIO_PIN_8
#define DIR_DC_M2_GPIO_Port GPIOD
#define FAULTn_M2_Pin GPIO_PIN_9
#define FAULTn_M2_GPIO_Port GPIOD
#define SNSOUT_M2_Pin GPIO_PIN_11
#define SNSOUT_M2_GPIO_Port GPIOD
#define FAULTn_M1_Pin GPIO_PIN_12
#define FAULTn_M1_GPIO_Port GPIOD
#define nSLEEP_M1_Pin GPIO_PIN_13
#define nSLEEP_M1_GPIO_Port GPIOD
#define T4_3_PWM_DC_M1_Pin GPIO_PIN_14
#define T4_3_PWM_DC_M1_GPIO_Port GPIOD
#define DIR_DC_M1_Pin GPIO_PIN_15
#define DIR_DC_M1_GPIO_Port GPIOD
#define SNSOUT__M1_Pin GPIO_PIN_6
#define SNSOUT__M1_GPIO_Port GPIOC
#define BRK_CONTROL_2_Pin GPIO_PIN_7
#define BRK_CONTROL_2_GPIO_Port GPIOC
#define BRK_CONTORL_1_Pin GPIO_PIN_8
#define BRK_CONTORL_1_GPIO_Port GPIOC
#define MC_EN_Pin GPIO_PIN_11
#define MC_EN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
