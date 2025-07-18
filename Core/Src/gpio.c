/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EV_1_Pin|STBY_1_Pin|PUMP_1_Pin|EV_2_Pin
                          |PRES_4_Pin|PRES_3_Pin|PRES_2_Pin|PRES_1_Pin
                          |HEART_BEAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STBY_2_Pin|PUMP_2_Pin|EV_3_Pin|STBY_3_Pin
                          |PUMP_3_Pin|EV_4_Pin|STBY_4_Pin|PUMP_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TOR_3_Pin TOR_4_Pin */
  GPIO_InitStruct.Pin = TOR_3_Pin|TOR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EV_1_Pin STBY_1_Pin PUMP_1_Pin EV_2_Pin
                           PRES_4_Pin PRES_3_Pin PRES_2_Pin PRES_1_Pin
                           HEART_BEAT_Pin */
  GPIO_InitStruct.Pin = EV_1_Pin|STBY_1_Pin|PUMP_1_Pin|EV_2_Pin
                          |PRES_4_Pin|PRES_3_Pin|PRES_2_Pin|PRES_1_Pin
                          |HEART_BEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STBY_2_Pin PUMP_2_Pin EV_3_Pin STBY_3_Pin
                           PUMP_3_Pin EV_4_Pin STBY_4_Pin PUMP_4_Pin */
  GPIO_InitStruct.Pin = STBY_2_Pin|PUMP_2_Pin|EV_3_Pin|STBY_3_Pin
                          |PUMP_3_Pin|EV_4_Pin|STBY_4_Pin|PUMP_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_TEST_Pin I2C_ADD_2_Pin I2C_ADD_1_Pin TOR_1_Pin
                           TOR_2_Pin */
  GPIO_InitStruct.Pin = B1_TEST_Pin|I2C_ADD_2_Pin|I2C_ADD_1_Pin|TOR_1_Pin
                          |TOR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
