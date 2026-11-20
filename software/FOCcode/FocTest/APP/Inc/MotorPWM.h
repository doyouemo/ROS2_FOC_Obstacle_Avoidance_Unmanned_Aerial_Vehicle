#ifndef __MOTOR_PWM_H__
#define __MOTOR_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

/* TIM clock 72MHz, PSC=0, ARR=3599 -> 20kHz */
#define MOTOR_PWM_PERIOD   3599U

typedef enum {
  MOTOR_1 = 0, /* PA0/PA1/PA2 -> TIM2 CH1/CH2/CH3 */
  MOTOR_2,     /* PA3/PA6/PA7 -> TIM2 CH4 + TIM3 CH1/CH2 */
  MOTOR_NUM
} MotorId_t;

void MotorPWM_Init(void);

/* Uα/Uβ 相对母线（pu），七段 SVPWM 写指定电机 */
void MotorPWM_SetVoltageAB(MotorId_t motor, float ualpha, float ubeta);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PWM_H__ */
