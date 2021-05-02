/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "itm_log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for heartBeatTask */
osThreadId_t heartBeatTaskHandle;
const osThreadAttr_t heartBeatTask_attributes = {
  .name = "heartBeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readPumpTask */
osThreadId_t readPumpTaskHandle;
const osThreadAttr_t readPumpTask_attributes = {
  .name = "readPumpTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for writePumpTask */
osThreadId_t writePumpTaskHandle;
const osThreadAttr_t writePumpTask_attributes = {
  .name = "writePumpTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartHeartBeatTask(void *argument);
void StartReadPumpTask(void *argument);
void StartWritePumpTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  LOG_INFO("freertos: Init FreeRTOS");
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of heartBeatTask */
  heartBeatTaskHandle = osThreadNew(StartHeartBeatTask, NULL, &heartBeatTask_attributes);

  /* creation of readPumpTask */
  readPumpTaskHandle = osThreadNew(StartReadPumpTask, NULL, &readPumpTask_attributes);

  /* creation of writePumpTask */
  writePumpTaskHandle = osThreadNew(StartWritePumpTask, NULL, &writePumpTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartBeatTask */
/**
  * @brief  Function implementing the heartBeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartBeatTask */
void StartHeartBeatTask(void *argument)
{
  /* USER CODE BEGIN StartHeartBeatTask */
  LOG_INFO("heartBeatTask: Start");

  HAL_GPIO_WritePin(PRES_1_GPIO_Port, PRES_1_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_2_GPIO_Port, PRES_2_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_3_GPIO_Port, PRES_3_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_4_GPIO_Port, PRES_4_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_1_GPIO_Port, PRES_1_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_2_GPIO_Port, PRES_2_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_3_GPIO_Port, PRES_3_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_4_GPIO_Port, PRES_4_Pin, GPIO_PIN_RESET);
  osDelay(100);

  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while(true) {
    HAL_GPIO_TogglePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin);
    osDelay(1000);
  }
#pragma clang diagnostic pop
  /* USER CODE END StartHeartBeatTask */
}

/* USER CODE BEGIN Header_StartReadPumpTask */
/**
* @brief Function implementing the readPumpTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadPumpTask */
void StartReadPumpTask(void *argument)
{
  /* USER CODE BEGIN StartReadPumpTask */
  LOG_INFO("ioTask: Start");

  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while(true) {
    if (pompe1.mode != POMPE_DISABLED) {
      LOG_INFO("ioTask: Read TOR 1");
      pompe1.tor = HAL_GPIO_ReadPin(TOR_1_GPIO_Port, TOR_1_Pin) == GPIO_PIN_SET;

      LOG_INFO("ioTask: Read Vacuostat 1");
      adcSelectVacuostat1();
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 1000);
      pompe1.vacuum = HAL_ADC_GetValue(&hadc1) / ADC_RESOLUTION;
      HAL_ADC_Stop(&hadc1);
    }

    if (pompe2.mode != POMPE_DISABLED) {
      LOG_INFO("ioTask: Read TOR 2");
      pompe2.tor = HAL_GPIO_ReadPin(TOR_2_GPIO_Port, TOR_2_Pin) == GPIO_PIN_SET;

      LOG_INFO("ioTask: Read Vacuostat 2");
      adcSelectVacuostat2();
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 1000);
      pompe2.vacuum = HAL_ADC_GetValue(&hadc1) / ADC_RESOLUTION;
      HAL_ADC_Stop(&hadc1);
    }

    if (pompe3.mode != POMPE_DISABLED) {
      LOG_INFO("ioTask: Read TOR3");
      pompe3.tor = HAL_GPIO_ReadPin(TOR_3_GPIO_Port, TOR_3_Pin) == GPIO_PIN_SET;

      LOG_INFO("ioTask: Read Vacuostat 3");
      adcSelectVacuostat3();
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 1000);
      pompe3.vacuum = HAL_ADC_GetValue(&hadc1) / ADC_RESOLUTION;
      HAL_ADC_Stop(&hadc1);
    }

    if (pompe4.mode != POMPE_DISABLED) {
      LOG_INFO("ioTask: Read TOR4");
      pompe4.tor = HAL_GPIO_ReadPin(TOR_4_GPIO_Port, TOR_4_Pin) == GPIO_PIN_SET;

      LOG_INFO("ioTask: Read Vacuostat 4");
      adcSelectVacuostat4();
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 1000);
      pompe4.vacuum = HAL_ADC_GetValue(&hadc1) / ADC_RESOLUTION;
      HAL_ADC_Stop(&hadc1);
    }

    osDelay(500);
  }
#pragma clang diagnostic pop
  /* USER CODE END StartReadPumpTask */
}

/* USER CODE BEGIN Header_StartWritePumpTask */
/**
* @brief Function implementing the writePumpTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWritePumpTask */
void StartWritePumpTask(void *argument)
{
  /* USER CODE BEGIN StartWritePumpTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartWritePumpTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
