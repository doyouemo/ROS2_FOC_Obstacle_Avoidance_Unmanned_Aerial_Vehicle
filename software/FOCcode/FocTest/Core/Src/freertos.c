/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "INA240.h"
#include "MotorPWM.h"
#include "FOC.h"
#include "AS5600.h"
#include "gpio.h"
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
osThreadId motorTaskHandle;
osThreadId logTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMotorTask(void const * argument);
void StartLogTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(motorTask, StartMotorTask, osPriorityAboveNormal, 0, 512);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  osThreadDef(logTask, StartLogTask, osPriorityLow, 0, 512);
  logTaskHandle = osThreadCreate(osThread(logTask), NULL);
  /* USER CODE END RTOS_THREADS */
}

void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  for (;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

void StartMotorTask(void const * argument)
{
  uint8_t key_stable[3] = {1U, 1U, 1U};
  uint8_t key_cnt[3] = {0};
  (void)argument;

  INA240_Init();
  MotorPWM_Init();
  FOC_Init();
  /* 默认 KEY1：不对齐、不加电压，只跑 Step 采数 + 日志 */
  FOC_SetMode(FOC_MODE_STOP);

  printf("\r\nDefault STOP (KEY1). Sensors logged; motor Uq=0.\r\n");
  printf("KEY1 stop  KEY2 speed  KEY3 position (need good AS5600)\r\n");
  printf("Full FOC: I->Clarke->Park | PI->Uq->InvPark->SVPWM\r\n");
  printf("Bad reads ignored (hold last good / Uq=0).\r\n\r\n");

  for (;;)
  {
    const uint8_t raw[3] = {
      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1U : 0U,
      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) ? 1U : 0U,
      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) ? 1U : 0U
    };
    uint8_t i;

    for (i = 0; i < 3U; i++)
    {
      if (raw[i] == key_stable[i])
      {
        key_cnt[i] = 0;
      }
      else if (++key_cnt[i] >= 20U)
      {
        key_cnt[i] = 0;
        key_stable[i] = raw[i];
        if (raw[i] != 0U)
        {
          if (i == 0U)
          {
            FOC_SetMode(FOC_MODE_STOP);
            printf("MODE stop\r\n");
          }
          else if (i == 1U)
          {
            /* 进速度前尝试对齐；失败则仍可进模式但 Uq 会被置 0 */
            printf("Align...\r\n");
            FOC_BeginAlign();
            osDelay(800);
            FOC_FinishAlign();
            FOC_SetMode(FOC_MODE_SPEED);
            printf("MODE speed\r\n");
          }
          else
          {
            printf("Align...\r\n");
            FOC_BeginAlign();
            osDelay(800);
            FOC_FinishAlign();
            FOC_SetMode(FOC_MODE_POSITION);
            printf("MODE hold position\r\n");
          }
        }
      }
    }

    FOC_Step(0.001f);
    osDelay(1);
  }
}

void StartLogTask(void const * argument)
{
  (void)argument;

  osDelay(200);

  for (;;)
  {
    printf("mode=%u | ASok=%u/%u AS=%u/%u | Iok=%u/%u INA=%u/%u/%u/%u | Iq=%ld/%ld mA | Uq=%ld/%ld mV\r\n",
           (unsigned)FOC_GetMode(),
           (unsigned)FOC_GetAngleOk(0), (unsigned)FOC_GetAngleOk(1),
           (unsigned)FOC_GetAngleRaw(0), (unsigned)FOC_GetAngleRaw(1),
           (unsigned)FOC_GetCurrentOk(0), (unsigned)FOC_GetCurrentOk(1),
           (unsigned)FOC_GetInaRaw(0), (unsigned)FOC_GetInaRaw(1),
           (unsigned)FOC_GetInaRaw(2), (unsigned)FOC_GetInaRaw(3),
           (long)(FOC_GetIq(0) * 1000.0f), (long)(FOC_GetIq(1) * 1000.0f),
           (long)(FOC_GetUq(0) * 1000.0f), (long)(FOC_GetUq(1) * 1000.0f));

    osDelay(300);
  }
}

/* USER CODE END Application */
