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
#include "stm32wbxx_hal.h"

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
#define L_Trigger_Pin GPIO_PIN_13
#define L_Trigger_GPIO_Port GPIOC
#define L_JOY_X_Pin GPIO_PIN_0
#define L_JOY_X_GPIO_Port GPIOC
#define L_JOY_Y_Pin GPIO_PIN_1
#define L_JOY_Y_GPIO_Port GPIOC
#define L_shoulder_Pin GPIO_PIN_0
#define L_shoulder_GPIO_Port GPIOA
#define L3_BTN_Pin GPIO_PIN_1
#define L3_BTN_GPIO_Port GPIOA
#define RUMBLE_L_Pin GPIO_PIN_2
#define RUMBLE_L_GPIO_Port GPIOA
#define D_LEFT_Pin GPIO_PIN_3
#define D_LEFT_GPIO_Port GPIOA
#define D_UP_Pin GPIO_PIN_4
#define D_UP_GPIO_Port GPIOA
#define D_RIGHT_Pin GPIO_PIN_5
#define D_RIGHT_GPIO_Port GPIOA
#define D_DOWN_Pin GPIO_PIN_6
#define D_DOWN_GPIO_Port GPIOA
#define RUMBLE_R_Pin GPIO_PIN_7
#define RUMBLE_R_GPIO_Port GPIOA
#define R_JOY_X_Pin GPIO_PIN_8
#define R_JOY_X_GPIO_Port GPIOA
#define R_JOY_Y_Pin GPIO_PIN_9
#define R_JOY_Y_GPIO_Port GPIOA
#define R3_BTN_Pin GPIO_PIN_4
#define R3_BTN_GPIO_Port GPIOC
#define B_BTN_Pin GPIO_PIN_0
#define B_BTN_GPIO_Port GPIOB
#define A_BTN_Pin GPIO_PIN_1
#define A_BTN_GPIO_Port GPIOB
#define Y_BTN_Pin GPIO_PIN_4
#define Y_BTN_GPIO_Port GPIOE
#define LED_IND_Pin GPIO_PIN_12
#define LED_IND_GPIO_Port GPIOB
#define X_BTN_Pin GPIO_PIN_13
#define X_BTN_GPIO_Port GPIOB
#define R_TRIGGER_Pin GPIO_PIN_14
#define R_TRIGGER_GPIO_Port GPIOB
#define R_SHOULDER_Pin GPIO_PIN_15
#define R_SHOULDER_GPIO_Port GPIOB
#define PLUS_BTN_Pin GPIO_PIN_6
#define PLUS_BTN_GPIO_Port GPIOC
#define BTH_SYNC_Pin GPIO_PIN_10
#define BTH_SYNC_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOD
#define QON_Pin GPIO_PIN_1
#define QON_GPIO_Port GPIOD
#define HOME_BTN_Pin GPIO_PIN_6
#define HOME_BTN_GPIO_Port GPIOB
#define MINUS_BTN_Pin GPIO_PIN_7
#define MINUS_BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
