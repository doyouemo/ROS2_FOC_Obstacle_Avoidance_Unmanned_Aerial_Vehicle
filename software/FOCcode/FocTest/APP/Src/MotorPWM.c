#include "MotorPWM.h"
#include <math.h>

#define OL_AMP_MAX        0.50f          /* |Uαβ|/Udc 上限，防过调制 */
#define OL_TWO_PI         6.28318530718f
#define OL_PI_OVER_3      1.04719755120f /* 60°=一扇区 */
#define SQRT3             1.73205080757f

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

  return (uint16_t)(duty * (float)MOTOR_PWM_PERIOD + 0.5f); /* 占空比→定时器比较值 */
}

static void MotorPWM_SetDuty(MotorId_t motor, float u, float v, float w)
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

static void MotorPWM_StopAll(void)
{
  MotorPWM_SetDuty(MOTOR_1, 0.5f, 0.5f, 0.5f); /* 50%≈无有效电压 */
  MotorPWM_SetDuty(MOTOR_2, 0.5f, 0.5f, 0.5f);
}

/* amp=|Uαβ|/Udc，theta=atan2(Uβ,Uα)；输出三相占空比 0~1 */
static void MotorPWM_SVPWM(float amp, float theta,
                           float *duty_u, float *duty_v, float *duty_w)
{
  float gamma;
  float T1, T2, T0_2;
  float Ta, Tb, Tc;
  uint8_t sector;

  if (amp < 1.0e-6f)
  {
    *duty_u = 0.5f;                      /* 零矢量：三相都 50% */
    *duty_v = 0.5f;
    *duty_w = 0.5f;
    return;
  }

  while (theta >= OL_TWO_PI)
  {
    theta -= OL_TWO_PI;
  }
  while (theta < 0.0f)
  {
    theta += OL_TWO_PI;
  }

  sector = (uint8_t)(theta / OL_PI_OVER_3) + 1U; /* 电压矢量落在第几扇区 1~6 */
  if (sector > 6U)
  {
    sector = 6U;
  }
  gamma = theta - (float)(sector - 1U) * OL_PI_OVER_3; /* 扇区内相对角 */

  T1 = SQRT3 * amp * sinf(OL_PI_OVER_3 - gamma); /* 相邻基本矢量1作用时间 */
  T2 = SQRT3 * amp * sinf(gamma);                /* 相邻基本矢量2作用时间 */

  if ((T1 + T2) > 1.0f)
  {
    float inv = 1.0f / (T1 + T2);
    T1 *= inv;                           /* 过调制时按比例缩回 */
    T2 *= inv;
  }

  T0_2 = 0.5f * (1.0f - T1 - T2);        /* 零矢量时间的一半 */

  switch (sector)                        /* 扇区→三相占空比排布 */
  {
    case 1: Ta = T0_2 + T1 + T2; Tb = T0_2 + T2;      Tc = T0_2;           break;
    case 2: Ta = T0_2 + T1;      Tb = T0_2 + T1 + T2; Tc = T0_2;           break;
    case 3: Ta = T0_2;           Tb = T0_2 + T1 + T2; Tc = T0_2 + T2;      break;
    case 4: Ta = T0_2;           Tb = T0_2 + T1;      Tc = T0_2 + T1 + T2; break;
    case 5: Ta = T0_2 + T2;      Tb = T0_2;           Tc = T0_2 + T1 + T2; break;
    case 6: Ta = T0_2 + T1 + T2; Tb = T0_2;           Tc = T0_2 + T1;      break;
    default:
      Ta = 0.5f;
      Tb = 0.5f;
      Tc = 0.5f;
      break;
  }

  *duty_u = Ta;
  *duty_v = Tb;
  *duty_w = Tc;
}

void MotorPWM_SetVoltageAB(MotorId_t motor, float ualpha, float ubeta)
{
  float amp;
  float theta;
  float u, v, w;

  amp = sqrtf(ualpha * ualpha + ubeta * ubeta); /* |Uαβ| */
  if (amp > OL_AMP_MAX)
  {
    float scale = OL_AMP_MAX / amp;
    ualpha *= scale;                     /* 限幅，保持方向 */
    ubeta *= scale;
    amp = OL_AMP_MAX;
  }

  theta = atan2f(ubeta, ualpha);         /* αβ 矢量角 */
  if (theta < 0.0f)
  {
    theta += OL_TWO_PI;
  }

  MotorPWM_SVPWM(amp, theta, &u, &v, &w); /* αβ→三相占空比 */
  MotorPWM_SetDuty(motor, u, v, w);      /* 写入定时器 */
}

void MotorPWM_Init(void)
{
  __HAL_TIM_DISABLE(&htim2);
  __HAL_TIM_DISABLE(&htim3);

  htim2.Init.Period = MOTOR_PWM_PERIOD;
  htim3.Init.Period = MOTOR_PWM_PERIOD;
  __HAL_TIM_SET_AUTORELOAD(&htim2, MOTOR_PWM_PERIOD);
  __HAL_TIM_SET_AUTORELOAD(&htim3, MOTOR_PWM_PERIOD);
  TIM2->EGR = TIM_EGR_UG;
  TIM3->EGR = TIM_EGR_UG;

  MotorPWM_StopAll();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); /* 电机1: U V W */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); /* 电机2: U 在 TIM2_CH4 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); /* 电机2: V W */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_ENABLE(&htim2);
  __HAL_TIM_ENABLE(&htim3);
}
