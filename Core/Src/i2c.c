/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "itm_log.h"
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  // Lecture des flags pour l'adresse I2C
  int i2cSlaveAddress = I2C_SLAVE_ADDRESS_BASE << 1;
  // Uncomment to use pin to change I2C address
  //i2cSlaveAddress += HAL_GPIO_ReadPin(I2C_ADD_1_GPIO_Port, I2C_ADD_1_Pin) == GPIO_PIN_SET ? 2 : 0;
  //i2cSlaveAddress += HAL_GPIO_ReadPin(I2C_ADD_2_GPIO_Port, I2C_ADD_2_Pin) == GPIO_PIN_SET ? 4 : 0;

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = i2cSlaveAddress;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static uint8_t rxBuffer[2]; 	// index of current rxBuffer

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  char buf[100];
  sprintf(buf, "i2c: Listen Complete Callback -> Command 0x%02X", rxBuffer[0]);
  LOG_INFO(buf);
  HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
  char buf[100];
  sprintf(buf, "i2c: Address Callback (Dir : %s)", TransferDirection == I2C_DIRECTION_RECEIVE ? "RX" : "TX");
  LOG_INFO(buf);

  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, sizeof(rxBuffer), I2C_FIRST_FRAME);

  } else {
    if (rxBuffer[0] == I2C_CMD_GET_VERSION) {
      sprintf(buf, "i2c: Address Callback send version %s", FIRMWARE_VERSION);
      LOG_INFO(buf);
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, FIRMWARE_VERSION, sizeof(FIRMWARE_VERSION), I2C_NEXT_FRAME);

    } else if (rxBuffer[0] == I2C_CMD_GET_ALL_PUMP_VALUES) {
      LOG_INFO("i2c: Address Callback send all pump status");
      uint8_t txBuffer[8];

      // Pompe 1
      txBuffer[0] = ((pompe1.vacuum >> 8) & 0xFF) + (pompe1.presence << 7) + (pompe1.tor << 6);
      txBuffer[1] = (pompe1.vacuum & 0xFF);

      // Pompe 2
      txBuffer[2] = ((pompe2.vacuum >> 8) & 0xFF) + (pompe2.presence << 7) + (pompe2.tor << 6);
      txBuffer[3] = (pompe2.vacuum & 0xFF);

      // Pompe 3
      txBuffer[4] = ((pompe3.vacuum >> 8) & 0xFF) + (pompe3.presence << 7) + (pompe3.tor << 6);
      txBuffer[5] = (pompe3.vacuum & 0xFF);

      // Pompe 4
      txBuffer[6] = ((pompe4.vacuum >> 8) & 0xFF) + (pompe4.presence << 7) + (pompe4.tor << 6);
      txBuffer[7] = (pompe4.vacuum & 0xFF);

      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, sizeof(txBuffer), I2C_NEXT_FRAME);

    } else if (rxBuffer[0] == I2C_CMD_GET_PUMP1_VALUES) {
      LOG_INFO("i2c: Address Callback send pump 1 status");
      uint8_t txBuffer[2];

      // Pompe 1
      txBuffer[0] = ((pompe1.vacuum >> 8) & 0xFF) + (pompe1.presence << 7) + (pompe1.tor << 6);
      txBuffer[1] = (pompe1.vacuum & 0xFF);

      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, sizeof(txBuffer), I2C_NEXT_FRAME);

    } else if (rxBuffer[0] == I2C_CMD_GET_PUMP2_VALUES) {
      LOG_INFO("i2c: Address Callback send pump 2 status");
      uint8_t txBuffer[2];

      // Pompe 2
      txBuffer[0] = ((pompe2.vacuum >> 8) & 0xFF) + (pompe2.presence << 7) + (pompe2.tor << 6);
      txBuffer[1] = (pompe2.vacuum & 0xFF);

      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, sizeof(txBuffer), I2C_NEXT_FRAME);

    } else if (rxBuffer[0] == I2C_CMD_GET_PUMP3_VALUES) {
      LOG_INFO("i2c: Address Callback send pump 3 status");
      uint8_t txBuffer[2];

      // Pompe 3
      txBuffer[0] = ((pompe3.vacuum >> 8) & 0xFF) + (pompe3.presence << 7) + (pompe3.tor << 6);
      txBuffer[1] = (pompe3.vacuum & 0xFF);

      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, sizeof(txBuffer), I2C_NEXT_FRAME);

    } else if (rxBuffer[0] == I2C_CMD_GET_PUMP4_VALUES) {
      LOG_INFO("i2c: Address Callback send pump 4 status");
      uint8_t txBuffer[2];

      // Pompe 4
      txBuffer[0] = ((pompe4.vacuum >> 8) & 0xFF) + (pompe4.presence << 7) + (pompe4.tor << 6);
      txBuffer[1] = (pompe4.vacuum & 0xFF);

      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, sizeof(txBuffer), I2C_NEXT_FRAME);

    } else {
      LOG_WARN("i2c: Address Callback, unknown command");
    }
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  char buf[100];
  sprintf(buf, "i2c: RX complete Callback -> Command = 0x%02X", rxBuffer[0]);
  LOG_INFO(buf);

  if (rxBuffer[0] == I2C_CMD_SET_PUMP_MODE) {
    sprintf(buf, "i2c: RX complete Callback -> All pumps = 0x%02X", rxBuffer[1]);
    LOG_INFO(buf);
    pompe1.mode = rxBuffer[1] & 0x03;
    pompe2.mode = (rxBuffer[1] >> 2) & 0x03;
    pompe3.mode = (rxBuffer[1] >> 4) & 0x03;
    pompe4.mode = (rxBuffer[1] >> 6) & 0x03;
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  char buf[100];
  sprintf(buf, "i2c: TX complete Callback -> Command = 0x%02X", rxBuffer[0]);
  LOG_INFO(buf);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  uint32_t errorCode = HAL_I2C_GetError(hi2c);
  if (errorCode == HAL_I2C_ERROR_AF) {
    // transaction terminated by master
    LOG_WARN("i2c: Error Callback -> transaction terminated by master");
  } else {
    char buf[100];
    sprintf(buf, "i2c: Error Callback -> err=0x%02lX", errorCode);
    LOG_ERROR(buf);
  }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
  LOG_INFO("i2c: Abort complete callback");  // never seen...
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
