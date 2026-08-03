#include "MotorPWM.h"

/* 慢速六步：每扇区停留时间(s)，便于对照串口扇区号 */
#define OL_SECTOR_HOLD_S  1.5f
/* 开环幅值硬限制，防堵转大电流 */
#define OL_AMP_MAX        0.12f

static float s_ol_amp;
static float s_ol_hold_t;
static uint8_t s_ol_sector;
static uint8_t s_ol_enable;

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

static void MotorPWM_ApplySixStepSector(uint8_t sector, float amp)
{
  const float hi = 0.5f + amp;
  const float lo = 0.5f - amp;

  sector %= 6U;

  switch (sector)
  {
    case 0: MotorPWM_SetDuty(MOTOR_1, hi, lo, lo); break;
    case 1: MotorPWM_SetDuty(MOTOR_1, hi, hi, lo); break;
    case 2: MotorPWM_SetDuty(MOTOR_1, lo, hi, lo); break;
    case 3: MotorPWM_SetDuty(MOTOR_1, lo, hi, hi); break;
    case 4: MotorPWM_SetDuty(MOTOR_1, lo, lo, hi); break;
    default: MotorPWM_SetDuty(MOTOR_1, hi, lo, hi); break;
  }

  MotorPWM_SetDuty(MOTOR_2, 0.5f, 0.5f, 0.5f);
}

void MotorPWM_Init(void)
{
  __HAL_TIM_SET_AUTORELOAD(&htim2, MOTOR_PWM_PERIOD);
  __HAL_TIM_SET_AUTORELOAD(&htim3, MOTOR_PWM_PERIOD);
  htim2.Init.Period = MOTOR_PWM_PERIOD;
  htim3.Init.Period = MOTOR_PWM_PERIOD;

  MotorPWM_StopAll();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

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
  /* 三相都拉到 50%：线电压约 0，比全 0(下管常开) 更利于“断电感” */
  MotorPWM_SetDuty(MOTOR_1, 0.5f, 0.5f, 0.5f);
  MotorPWM_SetDuty(MOTOR_2, 0.5f, 0.5f, 0.5f);
  s_ol_enable = 0U;
}

void MotorPWM_OpenLoopM0_Start(float amp, float elec_hz)
{
  (void)elec_hz; /* 慢速点动不再用连续电频率，避免失步发热 */

  if (amp < 0.0f)
  {
    amp = 0.0f;
  }
  else if (amp > OL_AMP_MAX)
  {
    amp = OL_AMP_MAX;
  }

  s_ol_amp = amp;
  s_ol_sector = 0U;
  s_ol_hold_t = 0.0f;
  s_ol_enable = 1U;

  MotorPWM_ApplySixStepSector(s_ol_sector, s_ol_amp);
}

void MotorPWM_OpenLoopM0_Step(float dt_s)
{
  if (!s_ol_enable)
  {
    return;
  }

  s_ol_hold_t += dt_s;
  if (s_ol_hold_t < OL_SECTOR_HOLD_S)
  {
    MotorPWM_ApplySixStepSector(s_ol_sector, s_ol_amp);
    return;
  }

  s_ol_hold_t = 0.0f;
  s_ol_sector++;
  if (s_ol_sector >= 6U)
  {
    s_ol_sector = 0U;
  }

  MotorPWM_ApplySixStepSector(s_ol_sector, s_ol_amp);
}

uint8_t MotorPWM_OpenLoopM0_GetSector(void)
{
  return s_ol_sector;
}

void MotorPWM_M0_HoldDuty(float u, float v, float w)
{
  s_ol_enable = 0U; /* 停开环，避免覆盖静态占空比 */
  MotorPWM_SetDuty(MOTOR_1, u, v, w);
  MotorPWM_SetDuty(MOTOR_2, 0.5f, 0.5f, 0.5f);
}
