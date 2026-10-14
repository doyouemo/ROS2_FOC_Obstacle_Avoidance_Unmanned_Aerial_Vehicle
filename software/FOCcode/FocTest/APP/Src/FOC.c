#include "FOC.h"
#include "MotorPWM.h"
#include "AS5600.h"
#include "INA240.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FOC_TWO_PI      6.28318530718f
#define FOC_3PI_2       4.71238898038f
#define FOC_SQRT3_BY_2  0.86602540378f
#define FOC_VBUS        12.0f
#define FOC_PP          7.0f
#define FOC_SHUNT_OHM   0.010f
#define FOC_INA_GAIN    20.0f
#define FOC_ADC_VREF    3.3f
#define FOC_VEL_REF     8.0f
#define FOC_ALIGN_UQ    2.5f

typedef struct {
  float p;
  float i;
  float integral;
  float out_min;
  float out_max;
} FocPi_t;

typedef struct {
  uint16_t raw;
  uint8_t angle_ok;     /* 本拍角度读成功 */
  uint8_t angle_valid;  /* 历史上至少成功过一次 */
  uint8_t current_ok;   /* 本拍两相电流都成功 */
  float mech;
  float mech_unw;
  float vel;
  float vel_lpf;
  float zero_el;
  float iq;
  float ia;
  float ib;
  float uq;
} FocAxis_t;

static FOC_Mode_t s_mode = FOC_MODE_STOP;
static FocAxis_t s_ax[2];
static FocPi_t s_pi_vel[2];
static FocPi_t s_pi_pos[2];
static float s_pos_ref[2];
static uint16_t s_ina_raw[INA240_CH_NUM];
static uint8_t s_ina_ok[INA240_CH_NUM];

static float clampf(float x, float lo, float hi)
{
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

static float wrap_pi2(float a)
{
  while (a >= FOC_TWO_PI) {
    a -= FOC_TWO_PI;
  }
  while (a < 0.0f) {
    a += FOC_TWO_PI;
  }
  return a;
}

static float pi_run(FocPi_t *pi, float err, float dt)
{
  float out;
  pi->integral += pi->i * err * dt;
  pi->integral = clampf(pi->integral, pi->out_min, pi->out_max);
  out = pi->p * err + pi->integral;
  return clampf(out, pi->out_min, pi->out_max);
}

static void pi_reset(FocPi_t *pi)
{
  pi->integral = 0.0f;
}

void FOC_Clarke(float a, float b, float c, float *alpha, float *beta)
{
  *alpha = (2.0f / 3.0f) * (a - 0.5f * b - 0.5f * c);
  *beta  = (2.0f / 3.0f) * (FOC_SQRT3_BY_2 * b - FOC_SQRT3_BY_2 * c);
}

void FOC_InvClarke(float alpha, float beta, float *a, float *b, float *c)
{
  *a = alpha;
  *b = -0.5f * alpha + FOC_SQRT3_BY_2 * beta;
  *c = -0.5f * alpha - FOC_SQRT3_BY_2 * beta;
}

void FOC_Park(float alpha, float beta, float theta, float *d, float *q)
{
  const float ct = cosf(theta);
  const float st = sinf(theta);
  *d =  ct * alpha + st * beta;
  *q = -st * alpha + ct * beta;
}

void FOC_InvPark(float d, float q, float theta, float *alpha, float *beta)
{
  const float ct = cosf(theta);
  const float st = sinf(theta);
  *alpha = ct * d - st * q;
  *beta  = st * d + ct * q;
}

static void foc_set_uq(MotorId_t mot, float uq, float el)
{
  float ualpha;
  float ubeta;

  uq = clampf(uq, -FOC_VBUS * 0.45f, FOC_VBUS * 0.45f);
  el = wrap_pi2(el);
  FOC_InvPark(0.0f, uq, el, &ualpha, &ubeta);
  MotorPWM_SetVoltageAB(mot, ualpha / FOC_VBUS, ubeta / FOC_VBUS);
}

static float adc_to_amp(uint16_t raw)
{
  const float v = ((float)raw) * (FOC_ADC_VREF / 4095.0f);
  return (FOC_ADC_VREF - v) / (FOC_INA_GAIN * FOC_SHUNT_OHM);
}

static float elec_angle(const FocAxis_t *ax)
{
  return wrap_pi2(FOC_PP * ax->mech - ax->zero_el);
}

/*
 * 角度失败：不更新 mech/vel，沿用旧值。
 * 电流失败：不更新 ia/ib/iq。
 * 两者都成功才做 Clarke/Park。
 */
static void update_axis(FocAxis_t *ax, uint8_t angle_ok, uint16_t raw,
                        uint8_t ia_ok, float ia_new,
                        uint8_t ib_ok, float ib_new,
                        float dt)
{
  float mech;
  float dmech;
  float ic;
  float alpha;
  float beta;
  float id;
  float iq;
  float el;

  ax->angle_ok = angle_ok;
  if (angle_ok != 0U)
  {
    mech = ((float)(raw & 0x0FFFU)) * (FOC_TWO_PI / 4096.0f);
    dmech = mech - ax->mech;
    if (ax->angle_valid != 0U)
    {
      if (dmech > M_PI) {
        dmech -= FOC_TWO_PI;
      } else if (dmech < -M_PI) {
        dmech += FOC_TWO_PI;
      }
      ax->mech_unw += dmech;
      ax->vel = (dt > 1.0e-6f) ? (dmech / dt) : 0.0f;
      ax->vel_lpf += 0.05f * (ax->vel - ax->vel_lpf);
    }
    else
    {
      ax->mech_unw = mech;
      ax->vel = 0.0f;
      ax->vel_lpf = 0.0f;
    }
    ax->mech = mech;
    ax->raw = raw;
    ax->angle_valid = 1U;
  }

  ax->current_ok = ((ia_ok != 0U) && (ib_ok != 0U)) ? 1U : 0U;
  if (ax->current_ok != 0U)
  {
    ax->ia = ia_new;
    ax->ib = ib_new;
  }

  if ((ax->angle_valid != 0U) && (ax->current_ok != 0U))
  {
    ic = -ax->ia - ax->ib;
    FOC_Clarke(ax->ia, ax->ib, ic, &alpha, &beta);
    el = elec_angle(ax);
    FOC_Park(alpha, beta, el, &id, &iq);
    ax->iq = ax->iq + 0.2f * (iq - ax->iq);
  }
}

void FOC_BeginAlign(void)
{
  foc_set_uq(MOTOR_1, FOC_ALIGN_UQ, FOC_3PI_2);
  foc_set_uq(MOTOR_2, FOC_ALIGN_UQ, FOC_3PI_2);
}

void FOC_FinishAlign(void)
{
  uint16_t r0 = 0U;
  uint16_t r1 = 0U;
  uint8_t ok0 = AS5600_ReadAngle1(&r0);
  uint8_t ok1 = AS5600_ReadAngle2(&r1);

  if (ok0 != 0U)
  {
    float m0 = ((float)(r0 & 0x0FFFU)) * (FOC_TWO_PI / 4096.0f);
    s_ax[0].mech = m0;
    s_ax[0].mech_unw = m0;
    s_ax[0].zero_el = wrap_pi2(FOC_PP * m0);
    s_ax[0].raw = r0;
    s_ax[0].angle_valid = 1U;
    s_pos_ref[0] = m0;
  }
  if (ok1 != 0U)
  {
    float m1 = ((float)(r1 & 0x0FFFU)) * (FOC_TWO_PI / 4096.0f);
    s_ax[1].mech = m1;
    s_ax[1].mech_unw = m1;
    s_ax[1].zero_el = wrap_pi2(FOC_PP * m1);
    s_ax[1].raw = r1;
    s_ax[1].angle_valid = 1U;
    s_pos_ref[1] = m1;
  }

  foc_set_uq(MOTOR_1, 0.0f, FOC_3PI_2);
  foc_set_uq(MOTOR_2, 0.0f, FOC_3PI_2);
}

void FOC_Init(void)
{
  uint8_t i;
  for (i = 0U; i < 2U; i++)
  {
    s_pi_vel[i].p = 0.12f;
    s_pi_vel[i].i = 0.40f;
    s_pi_vel[i].out_min = -4.0f;
    s_pi_vel[i].out_max = 4.0f;
    s_pi_pos[i].p = 4.0f;
    s_pi_pos[i].i = 0.0f;
    s_pi_pos[i].out_min = -4.0f;
    s_pi_pos[i].out_max = 4.0f;
    s_ax[i].angle_ok = 0U;
    s_ax[i].angle_valid = 0U;
    s_ax[i].current_ok = 0U;
    s_ax[i].uq = 0.0f;
  }
  for (i = 0U; i < INA240_CH_NUM; i++)
  {
    s_ina_raw[i] = 0U;
    s_ina_ok[i] = 0U;
  }
  s_mode = FOC_MODE_STOP;
}

void FOC_SetMode(FOC_Mode_t mode)
{
  s_mode = mode;
  pi_reset(&s_pi_vel[0]);
  pi_reset(&s_pi_vel[1]);
  pi_reset(&s_pi_pos[0]);
  pi_reset(&s_pi_pos[1]);
  s_pos_ref[0] = s_ax[0].mech_unw;
  s_pos_ref[1] = s_ax[1].mech_unw;
}

FOC_Mode_t FOC_GetMode(void)
{
  return s_mode;
}

float FOC_GetIq(uint8_t axis)
{
  if (axis > 1U) {
    return 0.0f;
  }
  return s_ax[axis].iq;
}

uint16_t FOC_GetAngleRaw(uint8_t axis)
{
  if (axis > 1U) {
    return 0U;
  }
  return s_ax[axis].raw;
}

uint16_t FOC_GetInaRaw(uint8_t ch)
{
  if (ch >= INA240_CH_NUM) {
    return 0U;
  }
  return s_ina_raw[ch];
}

uint8_t FOC_GetAngleOk(uint8_t axis)
{
  if (axis > 1U) {
    return 0U;
  }
  return s_ax[axis].angle_ok;
}

uint8_t FOC_GetCurrentOk(uint8_t axis)
{
  if (axis > 1U) {
    return 0U;
  }
  return s_ax[axis].current_ok;
}

float FOC_GetUq(uint8_t axis)
{
  if (axis > 1U) {
    return 0.0f;
  }
  return s_ax[axis].uq;
}

void FOC_Step(float dt_s)
{
  uint16_t ang0 = s_ax[0].raw;
  uint16_t ang1 = s_ax[1].raw;
  uint8_t aok0;
  uint8_t aok1;
  float uq0 = 0.0f;
  float uq1 = 0.0f;

  /* 电流：失败通道不覆盖旧 raw */
  INA240_ReadAllRawOk(s_ina_raw, s_ina_ok);

  /* 角度：失败则 aok=0，ang 保持旧值 */
  aok0 = AS5600_ReadAngle1(&ang0);
  aok1 = AS5600_ReadAngle2(&ang1);

  update_axis(&s_ax[0], aok0, ang0,
              s_ina_ok[INA240_CH1], adc_to_amp(s_ina_raw[INA240_CH1]),
              s_ina_ok[INA240_CH2], adc_to_amp(s_ina_raw[INA240_CH2]),
              dt_s);
  update_axis(&s_ax[1], aok1, ang1,
              s_ina_ok[INA240_CH3], adc_to_amp(s_ina_raw[INA240_CH3]),
              s_ina_ok[INA240_CH4], adc_to_amp(s_ina_raw[INA240_CH4]),
              dt_s);

  /*
   * 完整控制：速度/位置 PI → Uq → 反 Park → SVPWM
   * 默认 STOP：Uq=0
   * 角度无效（从未读成功或本拍失败且无历史）：强制 Uq=0，不用坏数据驱动
   */
  if (s_mode == FOC_MODE_STOP)
  {
    uq0 = 0.0f;
    uq1 = 0.0f;
  }
  else if (s_mode == FOC_MODE_SPEED)
  {
    if ((s_ax[0].angle_valid != 0U) && (s_ax[0].angle_ok != 0U))
    {
      uq0 = pi_run(&s_pi_vel[0], FOC_VEL_REF - s_ax[0].vel_lpf, dt_s);
    }
    if ((s_ax[1].angle_valid != 0U) && (s_ax[1].angle_ok != 0U))
    {
      uq1 = pi_run(&s_pi_vel[1], FOC_VEL_REF - s_ax[1].vel_lpf, dt_s);
    }
  }
  else
  {
    if ((s_ax[0].angle_valid != 0U) && (s_ax[0].angle_ok != 0U))
    {
      uq0 = pi_run(&s_pi_pos[0], s_pos_ref[0] - s_ax[0].mech_unw, dt_s);
    }
    if ((s_ax[1].angle_valid != 0U) && (s_ax[1].angle_ok != 0U))
    {
      uq1 = pi_run(&s_pi_pos[1], s_pos_ref[1] - s_ax[1].mech_unw, dt_s);
    }
  }

  s_ax[0].uq = uq0;
  s_ax[1].uq = uq1;

  if (s_ax[0].angle_valid != 0U)
  {
    foc_set_uq(MOTOR_1, uq0, elec_angle(&s_ax[0]));
  }
  else
  {
    foc_set_uq(MOTOR_1, 0.0f, 0.0f);
  }

  if (s_ax[1].angle_valid != 0U)
  {
    foc_set_uq(MOTOR_2, uq1, elec_angle(&s_ax[1]));
  }
  else
  {
    foc_set_uq(MOTOR_2, 0.0f, 0.0f);
  }
}
