#include "FOC.h"
#include "MotorPWM.h"
#include "AS5600.h"
#include "INA240.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FOC_TWO_PI      6.28318530718f   /* 2π */
#define FOC_3PI_2       4.71238898038f   /* 3π/2，对齐用固定电角度 */
#define FOC_SQRT3_BY_2  0.86602540378f
#define FOC_VBUS        12.0f            /* 母线电压假设 [V] */
#define FOC_PP          7.0f             /* 极对数：机械角×PP=电角度 */
#define FOC_SHUNT_OHM   0.010f           /* 分流电阻 */
#define FOC_INA_GAIN    20.0f            /* INA240 增益 */
#define FOC_ADC_VREF    3.3f
#define FOC_ALIGN_UQ    2.5f             /* 对齐时 q 轴电压，拉转子 */

#define FOC_VEL_LPF_TAU 0.008f           /* 速度低通时间常数 [s] */
#define FOC_GLITCH_VEL  40.0f            /* 角度毛刺：允许的最大角速度量级 */
#define FOC_GLITCH_MIN  0.05f            /* 毛刺门限下限 [rad] */

#define FOC_IQ_MAX      2.5f             /* 速度环输出 Iq* 限幅 [A] */
#define FOC_U_MAX       (FOC_VBUS * 0.45f) /* Ud/Uq 电压限幅 */
#define FOC_RQ_FF       0.80f            /* 电流失效时 U≈R·Iq 前馈 */
#define FOC_VEL_ACCEL   40.0f            /* 速度给定斜坡加速度 [rad/s²] */

typedef struct {
  float p;
  float i;
  float integral;
  float out_min;
  float out_max;
} FocPi_t;

typedef struct {
  uint16_t raw;          /* AS5600 原始 0~4095 */
  uint8_t angle_ok;      /* 本拍角度读成功 */
  uint8_t angle_valid;   /* 历史上至少成功过一次 */
  uint8_t current_ok;    /* 本拍两相电流都成功 */
  float mech;            /* 本圈机械角 [0,2π) */
  float mech_unw;        /* 多圈展开机械角，位置环用 */
  float vel;             /* 瞬时角速度 [rad/s] */
  float vel_lpf;         /* 滤波后速度，速度环反馈 */
  float zero_el;         /* 对齐得到的电角度零点偏移 */
  float id;              /* d 轴电流（励磁），目标通常 0 */
  float iq;              /* q 轴电流（转矩），电流环反馈 */
  float iq_ref;          /* 速度环给出的 Iq* */
  float ia;              /* A 相电流 */
  float ib;              /* B 相电流 */
  float ud;              /* d 轴电压指令 */
  float uq;              /* q 轴电压指令 */
  float glitch_mech;     /* 毛刺候选角度 */
  uint8_t glitch_arm;    /* 已看到一次可疑跳变，等下拍确认 */
} FocAxis_t;

static FOC_Mode_t s_mode = FOC_MODE_STOP;
static FocAxis_t s_ax[2];                /* [0]=电机1，[1]=电机2 */
static FocPi_t s_pi_vel[2];              /* 速度环 PI：出 Iq* */
static FocPi_t s_pi_pos[2];              /* 位置环 P：出 ω* */
static FocPi_t s_pi_id[2];               /* Id 电流环：出 Ud */
static FocPi_t s_pi_iq[2];               /* Iq 电流环：出 Uq */
static float s_pos_ref[2];               /* 位置给定（对齐/切模式时锁住） */
static float s_vel_target = FOC_VEL_DEFAULT; /* 串口目标速度 */
static float s_vel_cmd = 0.0f;           /* 斜坡后的速度给定 ω* */
static uint16_t s_ina_raw[INA240_CH_NUM];
static uint8_t s_ina_ok[INA240_CH_NUM];

static float ramp_toward(float cmd, float target, float accel, float dt)
{
  float max_du = accel * dt;             /* 本拍最多变化量 */
  float err = target - cmd;

  if (max_du < 1.0e-6f) {
    return target;
  }
  if (err > max_du) {
    return cmd + max_du;                 /* 向上爬 */
  }
  if (err < -max_du) {
    return cmd - max_du;                 /* 向下爬 */
  }
  return target;                         /* 已到目标 */
}

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
  return a;                              /* 归一化到 [0,2π) */
}

static float pi_run(FocPi_t *pi, float err, float dt)
{
  float out;
  pi->integral += pi->i * err * dt;      /* 积分项累加 */
  pi->integral = clampf(pi->integral, pi->out_min, pi->out_max);
  out = pi->p * err + pi->integral;      /* P + I */
  return clampf(out, pi->out_min, pi->out_max);
}

static void pi_reset(FocPi_t *pi)
{
  pi->integral = 0.0f;                   /* 切模式清积分，防猛冲 */
}

void FOC_Clarke(float a, float b, float c, float *alpha, float *beta)
{
  *alpha = (2.0f / 3.0f) * (a - 0.5f * b - 0.5f * c);              /* 三相→α */
  *beta  = (2.0f / 3.0f) * (FOC_SQRT3_BY_2 * b - FOC_SQRT3_BY_2 * c); /* 三相→β */
}

void FOC_Park(float alpha, float beta, float theta, float *d, float *q)
{
  const float ct = cosf(theta);
  const float st = sinf(theta);
  *d =  ct * alpha + st * beta;          /* 静止系→转子 d 轴 */
  *q = -st * alpha + ct * beta;          /* 静止系→转子 q 轴（转矩） */
}

void FOC_InvPark(float d, float q, float theta, float *alpha, float *beta)
{
  const float ct = cosf(theta);
  const float st = sinf(theta);
  *alpha = ct * d - st * q;              /* dq→α，准备 SVPWM */
  *beta  = st * d + ct * q;              /* dq→β */
}

static void foc_set_udq(MotorId_t mot, float ud, float uq, float el)
{
  float ualpha;
  float ubeta;

  ud = clampf(ud, -FOC_U_MAX, FOC_U_MAX);
  uq = clampf(uq, -FOC_U_MAX, FOC_U_MAX);
  el = wrap_pi2(el);                     /* 电角度 */
  FOC_InvPark(ud, uq, el, &ualpha, &ubeta); /* 控制电压变到 αβ */
  MotorPWM_SetVoltageAB(mot, ualpha / FOC_VBUS, ubeta / FOC_VBUS); /* 归一化后 SVPWM */
}

static float adc_to_amp(uint16_t raw)
{
  const float v = ((float)raw) * (FOC_ADC_VREF / 4095.0f); /* ADC→伏特 */
  return (FOC_ADC_VREF - v) / (FOC_INA_GAIN * FOC_SHUNT_OHM); /* 伏特→安培 */
}

static float elec_angle(const FocAxis_t *ax)
{
  return wrap_pi2(FOC_PP * ax->mech - ax->zero_el); /* 机械角→电角度 */
}

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
    uint8_t accept = 1U;

    mech = ((float)(raw & 0x0FFFU)) * (FOC_TWO_PI / 4096.0f); /* 12bit→弧度 */
    dmech = mech - ax->mech;             /* 本拍角增量 */
    if (ax->angle_valid != 0U)
    {
      float max_step;
      float k;

      if (dmech > M_PI) {
        dmech -= FOC_TWO_PI;             /* 过 0 点正向展开 */
      } else if (dmech < -M_PI) {
        dmech += FOC_TWO_PI;             /* 过 0 点反向展开 */
      }
      max_step = FOC_GLITCH_VEL * dt + FOC_GLITCH_MIN; /* 可信最大步长 */
      if ((dmech > max_step) || (dmech < -max_step))
      {
        float dconfirm = mech - ax->glitch_mech;

        if (dconfirm > M_PI) {
          dconfirm -= FOC_TWO_PI;
        } else if (dconfirm < -M_PI) {
          dconfirm += FOC_TWO_PI;
        }
        if ((ax->glitch_arm != 0U) &&
            (dconfirm <= max_step) && (dconfirm >= -max_step))
        {
          ax->mech_unw += dmech;         /* 连续两拍确认：接受跳变 */
          ax->vel = (dt > 1.0e-6f) ? (dmech / dt) : 0.0f;
          k = dt / (dt + FOC_VEL_LPF_TAU);
          ax->vel_lpf += k * (ax->vel - ax->vel_lpf);
          ax->mech = mech;
          ax->raw = raw;
          ax->glitch_arm = 0U;
          accept = 0U;                   /* 已在上面更新，跳过下面常规分支 */
        }
        else
        {
          ax->glitch_mech = mech;        /* 第一次可疑：先记下，下拍再确认 */
          ax->glitch_arm = 1U;
          accept = 0U;                   /* 本拍丢弃 */
        }
      }
      if (accept != 0U)
      {
        ax->glitch_arm = 0U;
        ax->mech_unw += dmech;           /* 多圈位置累加 */
        ax->vel = (dt > 1.0e-6f) ? (dmech / dt) : 0.0f; /* 差分速度 */
        k = dt / (dt + FOC_VEL_LPF_TAU);
        ax->vel_lpf += k * (ax->vel - ax->vel_lpf); /* 一阶低通 */
        ax->mech = mech;
        ax->raw = raw;
      }
    }
    else
    {
      ax->mech_unw = mech;               /* 第一次有效角：当起点 */
      ax->vel = 0.0f;
      ax->vel_lpf = 0.0f;
      ax->mech = mech;
      ax->raw = raw;
      ax->angle_valid = 1U;
    }
  }

  ax->current_ok = ((ia_ok != 0U) && (ib_ok != 0U)) ? 1U : 0U;
  if (ax->current_ok != 0U)
  {
    ax->ia = ia_new;
    ax->ib = ib_new;
  }

  if ((ax->angle_valid != 0U) && (ax->current_ok != 0U))
  {
    ic = -ax->ia - ax->ib;               /* 三相电流和为 0，补第三相 */
    FOC_Clarke(ax->ia, ax->ib, ic, &alpha, &beta); /* abc→αβ */
    el = elec_angle(ax);                 /* 当前电角度 */
    FOC_Park(alpha, beta, el, &id, &iq); /* αβ→dq */
    ax->id = ax->id + 0.35f * (id - ax->id); /* 轻滤波 */
    ax->iq = ax->iq + 0.35f * (iq - ax->iq);
  }
}

static float axis_velocity_to_iq(uint8_t i, float vel_cmd, float dt)
{
  float iq_ref;

  iq_ref = pi_run(&s_pi_vel[i], vel_cmd - s_ax[i].vel_lpf, dt); /* ω误差→Iq* */
  s_ax[i].iq_ref = iq_ref;
  return iq_ref;
}

static void axis_current_to_udq(uint8_t i, float iq_ref, float dt,
                                float *ud, float *uq)
{
  if (s_ax[i].current_ok != 0U)
  {
    *ud = pi_run(&s_pi_id[i], 0.0f - s_ax[i].id, dt);   /* Id*=0 → Ud */
    *uq = pi_run(&s_pi_iq[i], iq_ref - s_ax[i].iq, dt); /* Iq*跟踪 → Uq */
  }
  else
  {
    *ud = 0.0f;
    *uq = clampf(iq_ref * FOC_RQ_FF, -FOC_U_MAX, FOC_U_MAX); /* 无电流反馈时的电压顶一下 */
  }
}

void FOC_BeginAlign(void)
{
  foc_set_udq(MOTOR_1, 0.0f, FOC_ALIGN_UQ, FOC_3PI_2); /* 固定电角度强拉电机1 */
  foc_set_udq(MOTOR_2, 0.0f, FOC_ALIGN_UQ, FOC_3PI_2); /* 同上电机2 */
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
    s_ax[0].zero_el = wrap_pi2(FOC_PP * m0); /* 记下电角度零点 */
    s_ax[0].raw = r0;
    s_ax[0].angle_valid = 1U;
    s_pos_ref[0] = m0;                   /* 位置模式锁在此角 */
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

  foc_set_udq(MOTOR_1, 0.0f, 0.0f, FOC_3PI_2); /* 对齐结束，松力矩 */
  foc_set_udq(MOTOR_2, 0.0f, 0.0f, FOC_3PI_2);
}

void FOC_Init(void)
{
  uint8_t i;
  for (i = 0U; i < 2U; i++)
  {
    s_pi_vel[i].p = 0.35f;               /* 速度环 P */
    s_pi_vel[i].i = 1.20f;               /* 速度环 I */
    s_pi_vel[i].out_min = -FOC_IQ_MAX;   /* 输出 Iq* 下限 */
    s_pi_vel[i].out_max = FOC_IQ_MAX;

    s_pi_pos[i].p = 10.0f;               /* 位置环 P：误差→ω* */
    s_pi_pos[i].i = 0.0f;                /* 位置不用积分 */
    s_pi_pos[i].out_min = FOC_VEL_MIN;   /* ω* 限速 */
    s_pi_pos[i].out_max = FOC_VEL_MAX;

    s_pi_id[i].p = 1.5f;                 /* Id 电流环 */
    s_pi_id[i].i = 80.0f;
    s_pi_id[i].out_min = -FOC_U_MAX;
    s_pi_id[i].out_max = FOC_U_MAX;

    s_pi_iq[i].p = 1.5f;                 /* Iq 电流环 */
    s_pi_iq[i].i = 80.0f;
    s_pi_iq[i].out_min = -FOC_U_MAX;
    s_pi_iq[i].out_max = FOC_U_MAX;

    s_ax[i].angle_ok = 0U;
    s_ax[i].angle_valid = 0U;
    s_ax[i].current_ok = 0U;
    s_ax[i].id = 0.0f;
    s_ax[i].iq = 0.0f;
    s_ax[i].iq_ref = 0.0f;
    s_ax[i].ud = 0.0f;
    s_ax[i].uq = 0.0f;
    s_ax[i].glitch_arm = 0U;
  }
  for (i = 0U; i < INA240_CH_NUM; i++)
  {
    s_ina_raw[i] = 0U;
    s_ina_ok[i] = 0U;
  }
  s_vel_target = FOC_VEL_DEFAULT;
  s_vel_cmd = 0.0f;
  s_mode = FOC_MODE_STOP;
}

void FOC_SetMode(FOC_Mode_t mode)
{
  s_mode = mode;
  pi_reset(&s_pi_vel[0]);
  pi_reset(&s_pi_vel[1]);
  pi_reset(&s_pi_pos[0]);
  pi_reset(&s_pi_pos[1]);
  pi_reset(&s_pi_id[0]);
  pi_reset(&s_pi_id[1]);
  pi_reset(&s_pi_iq[0]);
  pi_reset(&s_pi_iq[1]);
  s_pos_ref[0] = s_ax[0].mech_unw;      /* 切模式：当前位置当位置给定 */
  s_pos_ref[1] = s_ax[1].mech_unw;
  s_ax[0].iq_ref = 0.0f;
  s_ax[1].iq_ref = 0.0f;
  if (mode == FOC_MODE_SPEED)
  {
    float v0 = 0.5f * (s_ax[0].vel_lpf + s_ax[1].vel_lpf);
    s_vel_cmd = clampf(v0, FOC_VEL_MIN, FOC_VEL_MAX); /* 斜坡从当前速度起步 */
  }
  else if (mode == FOC_MODE_STOP)
  {
    s_vel_cmd = 0.0f;
  }
}

FOC_Mode_t FOC_GetMode(void)
{
  return s_mode;
}

void FOC_SetSpeedRef(float rad_s)
{
  s_vel_target = clampf(rad_s, FOC_VEL_MIN, FOC_VEL_MAX); /* 只改目标，不瞬时改 ω* */
}

float FOC_GetSpeedRef(void)
{
  return s_vel_target;
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

uint8_t FOC_GetAngleOk(uint8_t axis)
{
  if (axis > 1U) {
    return 0U;
  }
  return s_ax[axis].angle_ok;
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
  uint8_t i;
  float iq_ref[2] = {0.0f, 0.0f};
  float ud[2] = {0.0f, 0.0f};
  float uq[2] = {0.0f, 0.0f};

  INA240_ReadAllRawOk(s_ina_raw, s_ina_ok); /* ① 读四路电流 ADC */

  aok0 = AS5600_ReadAngle1(&ang0);       /* ② 读电机1角度 */
  aok1 = AS5600_ReadAngle2(&ang1);       /* ② 读电机2角度 */

  update_axis(&s_ax[0], aok0, ang0,      /* ③ 轴0：速度 + Clarke/Park→id/iq */
              s_ina_ok[INA240_CH1], adc_to_amp(s_ina_raw[INA240_CH1]),
              s_ina_ok[INA240_CH2], adc_to_amp(s_ina_raw[INA240_CH2]),
              dt_s);
  update_axis(&s_ax[1], aok1, ang1,      /* ③ 轴1 同上 */
              s_ina_ok[INA240_CH3], adc_to_amp(s_ina_raw[INA240_CH3]),
              s_ina_ok[INA240_CH4], adc_to_amp(s_ina_raw[INA240_CH4]),
              dt_s);

  if (s_mode == FOC_MODE_SPEED)
  {
    s_vel_cmd = ramp_toward(s_vel_cmd, s_vel_target, FOC_VEL_ACCEL, dt_s); /* ④ 目标斜坡→ω* */
  }
  else if (s_mode == FOC_MODE_STOP)
  {
    s_vel_cmd = ramp_toward(s_vel_cmd, 0.0f, FOC_VEL_ACCEL, dt_s);
  }

  for (i = 0U; i < 2U; i++)
  {
    float vel_cmd = 0.0f;                /* 本轴本拍的 ω* */

    if (s_mode == FOC_MODE_STOP)
    {
      iq_ref[i] = 0.0f;
      s_ax[i].iq_ref = 0.0f;
      ud[i] = 0.0f;
      uq[i] = 0.0f;
      continue;                          /* STOP：无力矩 */
    }

    if ((s_ax[i].angle_valid == 0U) || (s_ax[i].angle_ok == 0U))
    {
      iq_ref[i] = 0.0f;
      s_ax[i].iq_ref = 0.0f;
      ud[i] = 0.0f;
      uq[i] = 0.0f;
      continue;                          /* 角度不可用：不敢闭环 */
    }

    if (s_mode == FOC_MODE_SPEED)
    {
      vel_cmd = s_vel_cmd;               /* 速度模式：用斜坡后的 ω* */
    }
    else
    {
      vel_cmd = pi_run(&s_pi_pos[i], s_pos_ref[i] - s_ax[i].mech_unw, dt_s); /* 位置误差→ω* */
    }

    iq_ref[i] = axis_velocity_to_iq(i, vel_cmd, dt_s); /* ⑤ 速度环→Iq* */
    axis_current_to_udq(i, iq_ref[i], dt_s, &ud[i], &uq[i]); /* ⑥ 电流环→Ud/Uq */
  }

  for (i = 0U; i < 2U; i++)
  {
    s_ax[i].ud = ud[i];
    s_ax[i].uq = uq[i];
  }

  if (s_ax[0].angle_valid != 0U)
  {
    foc_set_udq(MOTOR_1, ud[0], uq[0], elec_angle(&s_ax[0])); /* ⑦ 反Park+SVPWM */
  }
  else
  {
    foc_set_udq(MOTOR_1, 0.0f, 0.0f, 0.0f);
  }

  if (s_ax[1].angle_valid != 0U)
  {
    foc_set_udq(MOTOR_2, ud[1], uq[1], elec_angle(&s_ax[1]));
  }
  else
  {
    foc_set_udq(MOTOR_2, 0.0f, 0.0f, 0.0f);
  }
}
