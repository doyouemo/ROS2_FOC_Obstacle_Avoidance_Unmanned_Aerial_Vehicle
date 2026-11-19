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
#include <stdlib.h>
#include "INA240.h"
#include "usart.h"
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

static uint8_t s_aligned = 0U;

static void align_once(void)
{
  if (s_aligned != 0U)
  {
    return;
  }
  printf("Align...\r\n");
  FOC_BeginAlign();
  osDelay(800);
  FOC_FinishAlign();
  s_aligned = 1U;
}

static void foc_apply_speed_line(char *buf, uint8_t n)
{
  if (n == 0U)
  {
    return;
  }
  buf[n] = '\0';
  FOC_SetSpeedRef(strtof(buf, NULL));
  printf("VEL %ld mrad/s\r\n", (long)(FOC_GetSpeedRef() * 1000.0f));
}

static void foc_poll_speed_uart(void)
{
  static char buf[16];
  static uint8_t n = 0U;
  static uint8_t idle = 0U;
  uint8_t raw;
  uint8_t got = 0U;

  while (USART1_RxPop(&raw) != 0U)
  {
    char ch = (char)raw;

    got = 1U;
    if ((ch == '\r') || (ch == '\n'))
    {
      foc_apply_speed_line(buf, n);
      n = 0U;
    }
    else if ((n + 1U) < (uint8_t)sizeof(buf) &&
             ((ch == '-') || (ch == '.') || ((ch >= '0') && (ch <= '9'))))
    {
      buf[n] = ch;
      n++;
    }
    else
    {
      n = 0U;
    }
  }

  if (got != 0U)
  {
    idle = 0U;
  }
  else if (n > 0U)
  {
    if (++idle >= 40U)
    {
      foc_apply_speed_line(buf, n);
      n = 0U;
      idle = 0U;
    }
  }
}

static void foc_dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void StartMotorTask(void const * argument)
{
  uint8_t key_stable[3] = {1U, 1U, 1U};
  uint8_t key_cnt[3] = {0};
  uint32_t last_cyc;
  (void)argument;

  INA240_Init();
  MotorPWM_Init();
  FOC_Init();
  foc_dwt_init();
  last_cyc = DWT->CYCCNT;
  /* 默认 KEY1：不对齐、不加电压 */
  FOC_SetMode(FOC_MODE_STOP);

  printf("\r\nDefault STOP (KEY1). Log off while stopped.\r\n");
  printf("KEY1 stop  KEY2 speed  KEY3 position\r\n");
  printf("UART speed rad/s, range %ld..%ld, default %ld\r\n\r\n",
         (long)FOC_VEL_MIN, (long)FOC_VEL_MAX, (long)FOC_VEL_DEFAULT);

  for (;;)
  {
    const uint8_t raw[3] = {
      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1U : 0U,
      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) ? 1U : 0U,
      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) ? 1U : 0U
    };
    uint8_t i;
    uint32_t now;
    float dt;

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
            align_once();
            FOC_SetMode(FOC_MODE_SPEED);
            printf("MODE speed\r\n");
          }
          else
          {
            align_once();
            FOC_SetMode(FOC_MODE_POSITION);
            printf("MODE hold position\r\n");
          }
          last_cyc = DWT->CYCCNT;
        }
      }
    }

    now = DWT->CYCCNT;
    dt = (float)(now - last_cyc) / (float)SystemCoreClock;
    last_cyc = now;
    if ((dt < 0.0002f) || (dt > 0.05f))
    {
      dt = 0.001f;
    }
    FOC_Step(dt);
    osDelay(1);
  }
}

void StartLogTask(void const * argument)
{
  uint8_t wait = 0U;
  (void)argument;

  osDelay(200);

  for (;;)
  {
    foc_poll_speed_uart();
    if (FOC_GetMode() != FOC_MODE_STOP)
    {
      if (wait == 0U)
      {
        uint8_t axis = 0xFFU;

        if (FOC_GetAngleOk(1) != 0U)
        {
          axis = 1U;
        }
        else if (FOC_GetAngleOk(0) != 0U)
        {
          axis = 0U;
        }
        if (axis != 0xFFU)
        {
          /* VOFA+ FireWater。这一拍读成功才打印。 */
          printf("%u,%ld,%u,%ld,%ld\n",
                 (unsigned)FOC_GetMode(),
                 (long)(FOC_GetSpeedRef() * 1000.0f),
                 (unsigned)FOC_GetAngleRaw(axis),
                 (long)(FOC_GetUq(axis) * 1000.0f),
                 (long)(FOC_GetIq(axis) * 1000.0f));
        }
      }
      wait = (uint8_t)((wait + 1U) % 30U);
    }
    else
    {
      wait = 0U;
    }

    osDelay(10);
  }
}

/* USER CODE END Application */
