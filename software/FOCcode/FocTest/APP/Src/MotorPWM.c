#include "MotorPWM.h"

static uint16_t MotorPWM_DutyToCcr(float duty)
{
  if (duty < 0.0f)
  {
    duty = 0.0f;
  }
  else if (duty > 1.0f)
  {
    duty = 1.0f;
  }

  return (uint16_t)(duty * (float)MOTOR_PWM_PERIOD + 0.5f);
}

void MotorPWM_Init(void)
{
  /* 保证 TIM2/TIM3 同频同周期 */
  __HAL_TIM_SET_AUTORELOAD(&htim2, MOTOR_PWM_PERIOD);
  __HAL_TIM_SET_AUTORELOAD(&htim3, MOTOR_PWM_PERIOD);
  htim2.Init.Period = MOTOR_PWM_PERIOD;
  htim3.Init.Period = MOTOR_PWM_PERIOD;

  /* 启动前占空比清零 */
  MotorPWM_StopAll();

  /* M1: TIM2 CH1~3；M2: TIM2 CH4 + TIM3 CH1/CH2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* 对齐计数器后同时开启，减小 M2 跨定时器相位差 */
  __HAL_TIM_DISABLE(&htim2);
  __HAL_TIM_DISABLE(&htim3);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_ENABLE(&htim2);
  __HAL_TIM_ENABLE(&htim3);
}

void MotorPWM_SetDuty(MotorId_t motor, float u, float v, float w)
{
  uint16_t ccr_u = MotorPWM_DutyToCcr(u);
  uint16_t ccr_v = MotorPWM_DutyToCcr(v);
  uint16_t ccr_w = MotorPWM_DutyToCcr(w);

  if (motor == MOTOR_1)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_v);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr_w);
  }
  else if (motor == MOTOR_2)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr_v);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr_w);
  }
}

void MotorPWM_StopAll(void)
{
  MotorPWM_SetDuty(MOTOR_1, 0.0f, 0.0f, 0.0f);
  MotorPWM_SetDuty(MOTOR_2, 0.0f, 0.0f, 0.0f);
}
