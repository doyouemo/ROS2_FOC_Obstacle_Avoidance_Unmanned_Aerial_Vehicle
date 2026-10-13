#include "MotorPWM.h"
#include <math.h>

/* 正弦幅值上限：duty = 0.5 ± amp，三相同一频率，相位差 120° */
#define OL_AMP_MAX        0.40f
#define OL_TWO_PI         6.28318530718f
#define OL_DEG120         (OL_TWO_PI / 3.0f)

static float s_ol_amp;
static float s_ol_hz;
static float s_ol_theta;
static float s_ol_dir = 1.0f;
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

/*
 * 开环正弦（FOC 电压部分）：载波都是 20kHz。
 * 三相基波频率相同，只是时序相差 120°。
 *   U = 0.5 + amp·sin(θ)
 *   V = 0.5 + amp·sin(θ - 120°)
 *   W = 0.5 + amp·sin(θ + 120°)
 */
static void MotorPWM_ApplySine(float theta, float amp)
{
  const float u = 0.5f + amp * sinf(theta);
  const float v = 0.5f + amp * sinf(theta - OL_DEG120);
  const float w = 0.5f + amp * sinf(theta + OL_DEG120);

  MotorPWM_SetDuty(MOTOR_1, u, v, w);
  MotorPWM_SetDuty(MOTOR_2, u, v, w);
}

void MotorPWM_Init(void)
{
  /* Cube 里 TIM 先按 65535 初始化，这里强制改成 20kHz，并立刻生效 */
  __HAL_TIM_DISABLE(&htim2);
  __HAL_TIM_DISABLE(&htim3);

  htim2.Init.Period = MOTOR_PWM_PERIOD;
  htim3.Init.Period = MOTOR_PWM_PERIOD;
  __HAL_TIM_SET_AUTORELOAD(&htim2, MOTOR_PWM_PERIOD);
  __HAL_TIM_SET_AUTORELOAD(&htim3, MOTOR_PWM_PERIOD);
  TIM2->EGR = TIM_EGR_UG;
  TIM3->EGR = TIM_EGR_UG;

  MotorPWM_StopAll();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

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
    /* M0：PA0/PA1/PA2 = TIM2 CH1/2/3 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_v);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr_w);
  }
  else if (motor == MOTOR_2)
  {
    /* M1：PA3=TIM2 CH4，PA6/PA7=TIM3 CH1/CH2 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr_v);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr_w);
  }
}

void MotorPWM_StopAll(void)
{
  MotorPWM_SetDuty(MOTOR_1, 0.5f, 0.5f, 0.5f);
  MotorPWM_SetDuty(MOTOR_2, 0.5f, 0.5f, 0.5f);
  s_ol_enable = 0U;
}

void MotorPWM_OpenLoopM0_Start(float amp, float elec_hz)
{
  if (amp < 0.0f)
  {
    amp = 0.0f;
  }
  else if (amp > OL_AMP_MAX)
  {
    amp = OL_AMP_MAX;
  }

  s_ol_hz = (elec_hz > 0.1f) ? elec_hz : 5.0f;
  s_ol_amp = amp;
  s_ol_theta = 0.0f;
  s_ol_dir = 1.0f;
  s_ol_enable = 1U;

  MotorPWM_ApplySine(s_ol_theta, s_ol_amp);
}

void MotorPWM_OpenLoopM0_Step(float dt_s)
{
  if (!s_ol_enable)
  {
    return;
  }

  s_ol_theta += s_ol_dir * OL_TWO_PI * s_ol_hz * dt_s;
  while (s_ol_theta >= OL_TWO_PI)
  {
    s_ol_theta -= OL_TWO_PI;
  }
  while (s_ol_theta < 0.0f)
  {
    s_ol_theta += OL_TWO_PI;
  }

  MotorPWM_ApplySine(s_ol_theta, s_ol_amp);
}

uint8_t MotorPWM_OpenLoopM0_GetSector(void)
{
  return (uint8_t)(s_ol_theta / (OL_TWO_PI / 6.0f));
}

void MotorPWM_OpenLoop_SetDir(int8_t dir)
{
  s_ol_dir = (dir < 0) ? -1.0f : 1.0f;
  if (!s_ol_enable)
  {
    s_ol_enable = 1U;
    MotorPWM_ApplySine(s_ol_theta, s_ol_amp);
  }
}

void MotorPWM_M0_HoldDuty(float u, float v, float w)
{
  s_ol_enable = 0U;
  /* 两路电机同时锁死，避免只测 M1 时其实没驱动 */
  MotorPWM_SetDuty(MOTOR_1, u, v, w);
  MotorPWM_SetDuty(MOTOR_2, u, v, w);
}
