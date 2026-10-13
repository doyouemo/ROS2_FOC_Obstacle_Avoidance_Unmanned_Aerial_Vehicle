#include "FOC.h"
#include "MotorPWM.h"
#include "AS5600.h"
#include "INA240.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FOC_TWO_PI      6.28318530718f
#define FOC_3PI_2       4.71238898038f /* 270°，对齐时用的电角度 */
#define FOC_SQRT3       1.73205080757f
#define FOC_VBUS        12.0f          /* 母线电压，占空比按这个折算 */
#define FOC_PP          7.0f           /* 极对数，和电机不一致要改 */
#define FOC_SHUNT_OHM   0.010f         /* 分流电阻，须是毫欧档 */
#define FOC_INA_GAIN    20.0f
#define FOC_ADC_VREF    3.3f

typedef struct {
  float p;
  float i;
  float integral;
  float out_min;
  float out_max;
} FocPi_t;

typedef struct {
  uint16_t raw;     /* AS5600 12bit */
  float mech;       /* 机械角 0~2π */
  float mech_unw;   /* 解包后的连续机械角，位置环用 */
  float vel;        /* 机械角速度 rad/s */
  float vel_lpf;
  float zero_el;    /* 对齐得到的电角度零偏 */
  float iq;
} FocAxis_t;

static FOC_Mode_t s_mode = FOC_MODE_STOP;
static FocAxis_t s_ax[2];     /* 0=M0, 1=M1 */
static FocPi_t s_pi_iq[2];
static FocPi_t s_pi_vel[2];
static FocPi_t s_pi_pos[2];
static float s_iq_ref = 0.15f; /* 电流环 Iq 目标，单位 A */
static float s_vel_ref = 8.0f; /* 速度环目标，机械 rad/s */
static float s_pos_ref[2];     /* 位置环目标：切入该模式时的当前角 */

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

/* 电角度归一化到 [0, 2π) */
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

/* Clarke：三相电流 -> αβ */
void _clack(float a, float b, float c, float *alpha, float *beta)
{
  *alpha = (2.0f / 3.0f) * (a - 0.5f * b - 0.5f * c);
  *beta = (FOC_SQRT3 / 3.0f) * (b - c);
}

/* Park：αβ -> dq，theta 为电角度 */
void _park(float alpha, float beta, float theta, float *d, float *q)
{
  const float ct = cosf(theta);
  const float st = sinf(theta);
  *d = ct * alpha + st * beta;
  *q = -st * alpha + ct * beta;
}

/* 反 Park：dq -> αβ。电压型 FOC 里 Ud 通常为 0 */
void _inv_park(float d, float q, float theta, float *alpha, float *beta)
{
  const float ct = cosf(theta);
  const float st = sinf(theta);
  *alpha = ct * d - st * q;
  *beta = st * d + ct * q;
}

/* Ud=0，Uq 反变换成三相电压，再写成 0~1 占空比 */
static void foc_set_uq(MotorId_t mot, float uq, float el)
{
  float ualpha;
  float ubeta;
  float ua;
  float ub;
  float uc;
  float mid = FOC_VBUS * 0.5f;

  uq = clampf(uq, -FOC_VBUS * 0.45f, FOC_VBUS * 0.45f);
  el = wrap_pi2(el);
  _inv_park(0.0f, uq, el, &ualpha, &ubeta);
  ua = ualpha + mid;
  ub = -0.5f * ualpha + 0.5f * FOC_SQRT3 * ubeta + mid;
  uc = -0.5f * ualpha - 0.5f * FOC_SQRT3 * ubeta + mid;
  MotorPWM_SetDuty(mot, clampf(ua / FOC_VBUS, 0.0f, 1.0f),
                   clampf(ub / FOC_VBUS, 0.0f, 1.0f),
                   clampf(uc / FOC_VBUS, 0.0f, 1.0f));
}

/* INA REF 接 3.3V：零电流 ADC≈满量程，只能看把输出往下拉的那一个电流方向 */
static float adc_to_amp(uint16_t raw)
{
  const float v = ((float)raw) * (FOC_ADC_VREF / 4095.0f);
  return (FOC_ADC_VREF - v) / (FOC_INA_GAIN * FOC_SHUNT_OHM);
}

static float elec_angle(const FocAxis_t *ax)
{
  return wrap_pi2(FOC_PP * ax->mech - ax->zero_el);
}

/* 更新机械角、速度、Iq。第三相电流用 ia+ib+ic=0 补 */
static void update_axis(FocAxis_t *ax, uint16_t raw, float ia, float ib, float dt)
{
  float mech;
  float dmech;
  float ic;
  float alpha;
  float beta;
  float id;
  float iq;
  float el;

  mech = ((float)(raw & 0x0FFFU)) * (FOC_TWO_PI / 4096.0f);
  dmech = mech - ax->mech;
  if (dmech > M_PI) {
    dmech -= FOC_TWO_PI;
  } else if (dmech < -M_PI) {
    dmech += FOC_TWO_PI;
  }
  ax->mech = mech;
  ax->mech_unw += dmech;
  ax->vel = (dt > 1.0e-6f) ? (dmech / dt) : 0.0f;
  ax->vel_lpf += 0.05f * (ax->vel - ax->vel_lpf);

  ic = -ia - ib;
  _clack(ia, ib, ic, &alpha, &beta);
  el = elec_angle(ax);
  _park(alpha, beta, el, &id, &iq);
  ax->iq = ax->iq + 0.2f * (iq - ax->iq);
  ax->raw = raw;
}

/* 对齐第一步：固定电角度 270° 加电压，转子被拉到该方向 */
void FOC_BeginAlign(void)
{
  foc_set_uq(MOTOR_1, 2.5f, FOC_3PI_2);
  foc_set_uq(MOTOR_2, 2.5f, FOC_3PI_2);
}

/* 对齐第二步：把此时编码器角记成电角度零点，然后卸电压 */
void FOC_FinishAlign(void)
{
  uint16_t r0 = AS5600_GetAngle1();
  uint16_t r1 = AS5600_GetAngle2();
  float m0 = ((float)(r0 & 0x0FFFU)) * (FOC_TWO_PI / 4096.0f);
  float m1 = ((float)(r1 & 0x0FFFU)) * (FOC_TWO_PI / 4096.0f);

  s_ax[0].mech = m0;
  s_ax[0].mech_unw = m0;
  s_ax[0].zero_el = wrap_pi2(FOC_PP * m0);
  s_ax[1].mech = m1;
  s_ax[1].mech_unw = m1;
  s_ax[1].zero_el = wrap_pi2(FOC_PP * m1);
  s_pos_ref[0] = m0;
  s_pos_ref[1] = m1;
  foc_set_uq(MOTOR_1, 0.0f, FOC_3PI_2);
  foc_set_uq(MOTOR_2, 0.0f, FOC_3PI_2);
}

void FOC_Init(void)
{
  uint8_t i;
  for (i = 0U; i < 2U; i++)
  {
    /* 电流环：I=0，避免 INA 单向采样时积分顶死 */
    s_pi_iq[i].p = 4.0f;
    s_pi_iq[i].i = 0.0f;
    s_pi_iq[i].out_min = -3.0f;
    s_pi_iq[i].out_max = 3.0f;
    s_pi_vel[i].p = 0.12f;
    s_pi_vel[i].i = 0.40f;
    s_pi_vel[i].out_min = -4.0f;
    s_pi_vel[i].out_max = 4.0f;
    s_pi_pos[i].p = 4.0f;
    s_pi_pos[i].i = 0.0f;
    s_pi_pos[i].out_min = -4.0f;
    s_pi_pos[i].out_max = 4.0f;
  }
  s_mode = FOC_MODE_STOP;
}

void FOC_SetMode(FOC_Mode_t mode)
{
  s_mode = mode;
  pi_reset(&s_pi_iq[0]);
  pi_reset(&s_pi_iq[1]);
  pi_reset(&s_pi_vel[0]);
  pi_reset(&s_pi_vel[1]);
  pi_reset(&s_pi_pos[0]);
  pi_reset(&s_pi_pos[1]);
  /* 位置环切入时锁当前角，避免目标跳到 0 把轴猛拧过去 */
  s_pos_ref[0] = s_ax[0].mech_unw;
  s_pos_ref[1] = s_ax[1].mech_unw;
}

FOC_Mode_t FOC_GetMode(void)
{
  return s_mode;
}

void FOC_Step(float dt_s)
{
  uint16_t ina[INA240_CH_NUM];
  float uq0;
  float uq1;
  float ia0;
  float ib0;
  float ia1;
  float ib1;

  INA240_ReadAllRaw(ina);
  ia0 = adc_to_amp(ina[INA240_CH1]); /* M0 两相采样，第三相由 Kirchhoff 补 */
  ib0 = adc_to_amp(ina[INA240_CH2]);
  ia1 = adc_to_amp(ina[INA240_CH3]);
  ib1 = adc_to_amp(ina[INA240_CH4]);

  update_axis(&s_ax[0], AS5600_GetAngle1(), ia0, ib0, dt_s);
  update_axis(&s_ax[1], AS5600_GetAngle2(), ia1, ib1, dt_s);

  if (s_mode == FOC_MODE_STOP)
  {
    uq0 = 0.0f;
    uq1 = 0.0f;
  }
  else if (s_mode == FOC_MODE_SPEED)
  {
    uq0 = pi_run(&s_pi_vel[0], s_vel_ref - s_ax[0].vel_lpf, dt_s);
    uq1 = pi_run(&s_pi_vel[1], s_vel_ref - s_ax[1].vel_lpf, dt_s);
  }
  else
  {
    /* 目标角在切入 POSITION 时锁死，误差越大 Uq 越大，拧开后弹回 */
    uq0 = pi_run(&s_pi_pos[0], s_pos_ref[0] - s_ax[0].mech_unw, dt_s);
    uq1 = pi_run(&s_pi_pos[1], s_pos_ref[1] - s_ax[1].mech_unw, dt_s);
  }

  foc_set_uq(MOTOR_1, uq0, elec_angle(&s_ax[0]));
  foc_set_uq(MOTOR_2, uq1, elec_angle(&s_ax[1]));
}
