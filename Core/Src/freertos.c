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
bool ihmLedState = false;
/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for heartBeatTimer */
osTimerId_t heartBeatTimerHandle;
const osTimerAttr_t heartBeatTimer_attributes = {
  .name = "heartBeatTimer"
};
/* Definitions for ledIhmTimer */
osTimerId_t ledIhmTimerHandle;
const osTimerAttr_t ledIhmTimer_attributes = {
  .name = "ledIhmTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ihmLed(Pompe *pompe, GPIO_TypeDef *presGpioPort, uint16_t presGpioPin);

void refreshPumpState(Pompe *pompe, GPIO_TypeDef *torGpioPort, uint16_t torGpioPin, void (*adcSelect)(void));

void managePump(Pompe *pompe,
                GPIO_TypeDef *stdbyGpioPort, uint16_t stdbyGpioPin,
                GPIO_TypeDef *pumpGpioPort, uint16_t pumpGpioPin,
                GPIO_TypeDef *evGpioPort, uint16_t evGpioPin);

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void heartBeatCallback(void *argument);
void ledIhmCallback(void *argument);

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

  /* Create the timer(s) */
  /* creation of heartBeatTimer */
  heartBeatTimerHandle = osTimerNew(heartBeatCallback, osTimerPeriodic, NULL, &heartBeatTimer_attributes);

  /* creation of ledIhmTimer */
  ledIhmTimerHandle = osTimerNew(ledIhmCallback, osTimerPeriodic, NULL, &ledIhmTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartMainTask, NULL, &mainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  LOG_INFO("mainTask: Start");

  // Boot animation
  HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_1_GPIO_Port, PRES_1_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_2_GPIO_Port, PRES_2_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_3_GPIO_Port, PRES_3_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_4_GPIO_Port, PRES_4_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_1_GPIO_Port, PRES_1_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_2_GPIO_Port, PRES_2_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_3_GPIO_Port, PRES_3_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(PRES_4_GPIO_Port, PRES_4_Pin, GPIO_PIN_RESET);
  osDelay(100);

  // Start timers after boot init
  osTimerStart(heartBeatTimerHandle, 1000);
  osTimerStart(ledIhmTimerHandle, 250);

  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (true) {
    refreshPumpState(&pompe1, TOR_1_GPIO_Port, TOR_1_Pin, adcSelectVacuostat1);
    managePump(&pompe1, STBY_1_GPIO_Port, STBY_1_Pin, PUMP_1_GPIO_Port, PUMP_1_Pin, EV_1_GPIO_Port, EV_1_Pin);

    refreshPumpState(&pompe2, TOR_2_GPIO_Port, TOR_2_Pin, adcSelectVacuostat2);
    managePump(&pompe2, STBY_2_GPIO_Port, STBY_2_Pin, PUMP_2_GPIO_Port, PUMP_2_Pin, EV_2_GPIO_Port, EV_2_Pin);

    refreshPumpState(&pompe3, TOR_3_GPIO_Port, TOR_3_Pin, adcSelectVacuostat3);
    managePump(&pompe3, STBY_3_GPIO_Port, STBY_3_Pin, PUMP_3_GPIO_Port, PUMP_3_Pin, EV_3_GPIO_Port, EV_3_Pin);

    refreshPumpState(&pompe4, TOR_4_GPIO_Port, TOR_4_Pin, adcSelectVacuostat4);
    managePump(&pompe4, STBY_4_GPIO_Port, STBY_4_Pin, PUMP_4_GPIO_Port, PUMP_4_Pin, EV_4_GPIO_Port, EV_4_Pin);

    osDelay(100);
  }
#pragma clang diagnostic pop
  /* USER CODE END StartMainTask */
}

/* heartBeatCallback function */
void heartBeatCallback(void *argument)
{
  /* USER CODE BEGIN heartBeatCallback */
  if (i2cErrorCode == HAL_I2C_ERROR_NONE) { // Pas d'erreur
    HAL_GPIO_TogglePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin);

  } else { // Erreur I2C persistante
    HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, GPIO_PIN_SET);
  }
  /* USER CODE END heartBeatCallback */
}

/* ledIhmCallback function */
void ledIhmCallback(void *argument)
{
  /* USER CODE BEGIN ledIhmCallback */
  ihmLed(&pompe1, PRES_1_GPIO_Port, PRES_1_Pin);
  ihmLed(&pompe2, PRES_2_GPIO_Port, PRES_2_Pin);
  ihmLed(&pompe3, PRES_3_GPIO_Port, PRES_3_Pin);
  ihmLed(&pompe4, PRES_4_GPIO_Port, PRES_4_Pin);

  ihmLedState = !ihmLedState;
  /* USER CODE END ledIhmCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ihmLed(Pompe *pompe, GPIO_TypeDef *presGpioPort, uint16_t presGpioPin) {
  if (pompe->mode == POMPE_ON || pompe->mode == POMPE_ON_FORCE) {
    if (pompe->presence) {
      HAL_GPIO_WritePin(presGpioPort, presGpioPin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(presGpioPort, presGpioPin, ihmLedState ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
  } else {
    HAL_GPIO_WritePin(presGpioPort, presGpioPin, GPIO_PIN_RESET);
  }
}

void refreshPumpState(Pompe *pompe, GPIO_TypeDef *torGpioPort, uint16_t torGpioPin, void (*adcSelect)(void)) {
  // Refresh du capteur TOR tous le temps
  pompe->tor = HAL_GPIO_ReadPin(torGpioPort, torGpioPin) == GPIO_PIN_RESET;

  // Refresh des infos lié au vide
  if (pompe->mode != POMPE_DISABLED) {

    adcSelect();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    pompe->vacuum = HAL_ADC_GetValue(&hadc1); // (HAL_ADC_GetValue(&hadc1) / ADC_RESOLUTION) * V_REF;
    HAL_ADC_Stop(&hadc1);

    pompe->presence = pompe->vacuum >= pompe->vacuumPresence;

  } else {
    pompe->presence = false;
    pompe->vacuum = 0;
  }
}

void managePump(Pompe *pompe,
                GPIO_TypeDef *stdbyGpioPort, uint16_t stdbyGpioPin,
                GPIO_TypeDef *pumpGpioPort, uint16_t pumpGpioPin,
                GPIO_TypeDef *evGpioPort, uint16_t evGpioPin) {

  // Detection du changement de mode de la pompe
  bool changeModePump = false;
  if (pompe->mode != pompe->modePrev) {
    pompe->modePrev = pompe->mode;
    changeModePump = true;
  }

  // Si le mode est disabled, on met le composant de gestion en Stand Bye
  if (pompe->mode == POMPE_DISABLED && changeModePump) {
    HAL_GPIO_WritePin(stdbyGpioPort, stdbyGpioPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pumpGpioPort, pumpGpioPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(evGpioPort, evGpioPin, GPIO_PIN_RESET);

    // Si on est sur un changement de mode, du coup autre que DISABLED, on active le composant
  } else if (changeModePump) {
    HAL_GPIO_WritePin(stdbyGpioPort, stdbyGpioPin, GPIO_PIN_SET);
  }

  // Si on a une presence détectée et la pompe ON
  if (pompe->mode == POMPE_ON && pompe->tor) {
    HAL_GPIO_WritePin(evGpioPort, evGpioPin, GPIO_PIN_RESET);
    if ((pompe->vacuum < pompe->vacuumOK && HAL_GPIO_ReadPin(pumpGpioPort, pumpGpioPin) == GPIO_PIN_SET) ||
        (pompe->vacuum < pompe->vacuumNOK)) {
      HAL_GPIO_WritePin(pumpGpioPort, pumpGpioPin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(pumpGpioPort, pumpGpioPin, GPIO_PIN_RESET);
    }
  } else if (pompe->mode == POMPE_ON && !pompe->tor) {
    HAL_GPIO_WritePin(evGpioPort, evGpioPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pumpGpioPort, pumpGpioPin, GPIO_PIN_RESET);
  } else if (pompe->mode == POMPE_ON_FORCE) {
    HAL_GPIO_WritePin(evGpioPort, evGpioPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pumpGpioPort, pumpGpioPin, GPIO_PIN_SET);
  }

  // Si on est sur un changement de mode et en mode OFF, procéssus de release de l'objet en différer pour tous les
  // déclencher quasiment en même temps.
  if (pompe->mode == POMPE_OFF && changeModePump) {
    pompe->cycleDepose = 0;
  } else if (pompe->mode == POMPE_OFF && !changeModePump && pompe->cycleDepose < 20) {
    pompe->cycleDepose++;
  }

#if ASPIRATION_ON_RELEASE == 1
  // 1 incrément de cycle égale le délai d'attente entre deux exec (x 50ms)
  if (pompe->mode == POMPE_OFF && pompe->cycleDepose == 0) {
    // Cycle dépose 1 (0ms) -> Allumage pompe à vide
    HAL_GPIO_WritePin(pumpGpioPort, pumpGpioPin, GPIO_PIN_SET);
  } else if (pompe->mode == POMPE_OFF && pompe->cycleDepose == 2) {
    // Cycle dépose 2 (200ms) -> Ouverture electrovanne
    HAL_GPIO_WritePin(evGpioPort, evGpioPin, GPIO_PIN_SET);
  } else if (pompe->mode == POMPE_OFF && pompe->cycleDepose == 3) {
    // Cycle dépose 3 (300ms) -> Arret Pompe à vide
    HAL_GPIO_WritePin(pumpGpioPort, pumpGpioPin, GPIO_PIN_RESET);
  } else if (pompe->mode == POMPE_OFF && pompe->cycleDepose == 5) {
    // Cycle dépose 4 (500ms) -> Fermeture electrovanne
    HAL_GPIO_WritePin(evGpioPort, evGpioPin, GPIO_PIN_RESET);
  }
#else
  if (pompe->mode == POMPE_OFF && pompe->cycleDepose == 0) {
    // Cycle dépose 1 (0ms) -> Ouverture electrovanne + arret pompe
    HAL_GPIO_WritePin(pumpGpioPort, pumpGpioPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(evGpioPort, evGpioPin, GPIO_PIN_SET);
  } else if (pompe->mode == POMPE_OFF && pompe->cycleDepose == 19) {
    // Cycle dépose 19 (1900ms) -> Fermeture electrovanne
    HAL_GPIO_WritePin(evGpioPort, evGpioPin, GPIO_PIN_RESET);
  }
#endif
}
/* USER CODE END Application */

