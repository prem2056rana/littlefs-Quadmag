/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#define SS1_Pin GPIO_PIN_0
#define SS1_GPIO_Port GPIOB
#define SS2_Pin GPIO_PIN_1
#define SS2_GPIO_Port GPIOB
#define SS3_Pin GPIO_PIN_2
#define SS3_GPIO_Port GPIOB
#define SS4_Pin GPIO_PIN_10
#define SS4_GPIO_Port GPIOB
#define FM_CS_Pin GPIO_PIN_12
#define FM_CS_GPIO_Port GPIOB
#define MSN_EN4_Pin GPIO_PIN_8
#define MSN_EN4_GPIO_Port GPIOA
#define MSN_EN2_Pin GPIO_PIN_15
#define MSN_EN2_GPIO_Port GPIOA
#define DRDY4_Pin GPIO_PIN_3
#define DRDY4_GPIO_Port GPIOB
#define DRDY3_Pin GPIO_PIN_4
#define DRDY3_GPIO_Port GPIOB
#define DRDY2_Pin GPIO_PIN_5
#define DRDY2_GPIO_Port GPIOB
#define DRDY1_Pin GPIO_PIN_6
#define DRDY1_GPIO_Port GPIOB
#define MSN_EN3_Pin GPIO_PIN_8
#define MSN_EN3_GPIO_Port GPIOB
#define MSN_EN1_Pin GPIO_PIN_9
#define MSN_EN1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
