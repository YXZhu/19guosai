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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define C13_Pin GPIO_PIN_13
#define C13_GPIO_Port GPIOC
#define DIR_1_Pin GPIO_PIN_0
#define DIR_1_GPIO_Port GPIOA
#define ENA_1_Pin GPIO_PIN_1
#define ENA_1_GPIO_Port GPIOA
#define F_CS_Pin GPIO_PIN_4
#define F_CS_GPIO_Port GPIOA
#define ENA_2_Pin GPIO_PIN_2
#define ENA_2_GPIO_Port GPIOB
#define DIR_2_Pin GPIO_PIN_10
#define DIR_2_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_3
#define INT_GPIO_Port GPIOB
#define INT_EXTI_IRQn EXTI3_IRQn
/* USER CODE BEGIN Private defines */

#define debug_printf 1
#if debug_printf == 0
	#define printf(fmt, ...) 
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
