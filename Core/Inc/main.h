/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define I2C_SLAVE_ADDRESS_BASE 0x26
#define TOR_3_Pin GPIO_PIN_13
#define TOR_3_GPIO_Port GPIOC
#define TOR_4_Pin GPIO_PIN_14
#define TOR_4_GPIO_Port GPIOC
#define VAC_1_Pin GPIO_PIN_0
#define VAC_1_GPIO_Port GPIOA
#define VAC_2_Pin GPIO_PIN_1
#define VAC_2_GPIO_Port GPIOA
#define VAC_3_Pin GPIO_PIN_2
#define VAC_3_GPIO_Port GPIOA
#define VAC_4_Pin GPIO_PIN_3
#define VAC_4_GPIO_Port GPIOA
#define EV_1_Pin GPIO_PIN_4
#define EV_1_GPIO_Port GPIOA
#define STBY_1_Pin GPIO_PIN_5
#define STBY_1_GPIO_Port GPIOA
#define PUMP_1_Pin GPIO_PIN_6
#define PUMP_1_GPIO_Port GPIOA
#define EV_2_Pin GPIO_PIN_7
#define EV_2_GPIO_Port GPIOA
#define STBY_2_Pin GPIO_PIN_0
#define STBY_2_GPIO_Port GPIOB
#define PUMP_2_Pin GPIO_PIN_1
#define PUMP_2_GPIO_Port GPIOB
#define B1_TEST_Pin GPIO_PIN_2
#define B1_TEST_GPIO_Port GPIOB
#define EV_3_Pin GPIO_PIN_10
#define EV_3_GPIO_Port GPIOB
#define STBY_3_Pin GPIO_PIN_11
#define STBY_3_GPIO_Port GPIOB
#define PUMP_3_Pin GPIO_PIN_12
#define PUMP_3_GPIO_Port GPIOB
#define EV_4_Pin GPIO_PIN_13
#define EV_4_GPIO_Port GPIOB
#define STBY_4_Pin GPIO_PIN_14
#define STBY_4_GPIO_Port GPIOB
#define PUMP_4_Pin GPIO_PIN_15
#define PUMP_4_GPIO_Port GPIOB
#define PRES_1_Pin GPIO_PIN_8
#define PRES_1_GPIO_Port GPIOA
#define PRES_2_Pin GPIO_PIN_9
#define PRES_2_GPIO_Port GPIOA
#define PRES_3_Pin GPIO_PIN_10
#define PRES_3_GPIO_Port GPIOA
#define PRES_4_Pin GPIO_PIN_11
#define PRES_4_GPIO_Port GPIOA
#define HEART_BEAT_Pin GPIO_PIN_12
#define HEART_BEAT_GPIO_Port GPIOA
#define ADD_2_Pin GPIO_PIN_4
#define ADD_2_GPIO_Port GPIOB
#define ADD_1_Pin GPIO_PIN_5
#define ADD_1_GPIO_Port GPIOB
#define TOR_1_Pin GPIO_PIN_8
#define TOR_1_GPIO_Port GPIOB
#define TOR_2_Pin GPIO_PIN_9
#define TOR_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
