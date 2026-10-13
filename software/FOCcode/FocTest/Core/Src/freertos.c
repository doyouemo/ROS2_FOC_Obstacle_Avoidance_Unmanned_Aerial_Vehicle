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
#include "INA240.h"
#include "MotorPWM.h"
#include "FOC.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint32_t print_div = 0;
  uint8_t key_stable[3] = {1U, 1U, 1U}; /* 1=未按下（高电平） */
  uint8_t key_cnt[3] = {0};

  INA240_Init();
  MotorPWM_Init();
  FOC_Init();

  /* 对齐时转子会被拉住，轴上不要加负载，编码器必须接上 */
  printf("\r\nAligning electrical angle, keep shafts free ~0.8s\r\n");
  FOC_BeginAlign();
  osDelay(800);
  FOC_FinishAlign();
  FOC_SetMode(FOC_MODE_STOP);

  printf("KEY1 stop  KEY2 speed  KEY3 hold position\r\n");
  printf("Need AS5600.\r\n\r\n");

  for(;;)
  {
    /* PB3/4/5 = KEY1/2/3，外部上拉，按下为低 */
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
      else if (++key_cnt[i] >= 20U) /* 连续 20ms 才认边沿，消抖 */
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
            FOC_SetMode(FOC_MODE_SPEED);
            printf("MODE speed\r\n");
          }
          else
          {
            FOC_SetMode(FOC_MODE_POSITION);
            printf("MODE hold position\r\n");
          }
        }
      }
    }

    FOC_Step(0.001f); /* 与 osDelay(1) 对应，约 1kHz */

    if (++print_div >= 300U)
    {
      print_div = 0;
      printf("mode=%u | M0=%lu/%lu/%lu | M1=%lu/%lu/%lu\r\n",
             (unsigned)FOC_GetMode(),
             (unsigned long)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1),
             (unsigned long)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2),
             (unsigned long)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3),
             (unsigned long)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4),
             (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1),
             (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2));
    }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

