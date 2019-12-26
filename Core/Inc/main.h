/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define GPS_IGT_Pin GPIO_PIN_0
#define GPS_IGT_GPIO_Port GPIOC
#define LED_PIN_9_Pin GPIO_PIN_13
#define LED_PIN_9_GPIO_Port GPIOB
#define LED_PIN_1_Pin GPIO_PIN_14
#define LED_PIN_1_GPIO_Port GPIOB
#define LED_PIN_0_Pin GPIO_PIN_15
#define LED_PIN_0_GPIO_Port GPIOB
#define LED_PIN_A_Pin GPIO_PIN_6
#define LED_PIN_A_GPIO_Port GPIOC
#define LED_PIN_B_Pin GPIO_PIN_7
#define LED_PIN_B_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define LED_PIN_2_Pin GPIO_PIN_8
#define LED_PIN_2_GPIO_Port GPIOA
#define LED_PIN_3_Pin GPIO_PIN_11
#define LED_PIN_3_GPIO_Port GPIOA
#define LED_PIN_4_Pin GPIO_PIN_12
#define LED_PIN_4_GPIO_Port GPIOA
#define TMS_SWDIO_Pin GPIO_PIN_13
#define TMS_SWDIO_GPIO_Port GPIOA
#define TCK_SWCLK_Pin GPIO_PIN_14
#define TCK_SWCLK_GPIO_Port GPIOA
#define LED_PIN_5_Pin GPIO_PIN_10
#define LED_PIN_5_GPIO_Port GPIOC
#define LED_PIN_6_Pin GPIO_PIN_11
#define LED_PIN_6_GPIO_Port GPIOC
#define LED_PIN_7_Pin GPIO_PIN_12
#define LED_PIN_7_GPIO_Port GPIOC
#define LED_PIN_8_Pin GPIO_PIN_2
#define LED_PIN_8_GPIO_Port GPIOD
#define LED_PIN_10_Pin GPIO_PIN_6
#define LED_PIN_10_GPIO_Port GPIOB
#define LED_PIN_11_Pin GPIO_PIN_7
#define LED_PIN_11_GPIO_Port GPIOB
#define LED_PIN_12_Pin GPIO_PIN_8
#define LED_PIN_12_GPIO_Port GPIOB
#define LED_PIN_13_Pin GPIO_PIN_9
#define LED_PIN_13_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
