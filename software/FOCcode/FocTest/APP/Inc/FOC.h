#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* KEY1/2/3：停止 / 两台均速 / 锁当前位置。各环输出都是 Uq（电压型）。 */
typedef enum {
  FOC_MODE_STOP = 0,    /* KEY1：Uq=0，电机无力矩 */
  FOC_MODE_SPEED,       /* KEY2：两台按同一机械角速度转 */
  FOC_MODE_POSITION     /* KEY3：切入时记下当前角，拧开会往回拉 */
} FOC_Mode_t;

void _clack(float a, float b, float c, float *alpha, float *beta); /* Clarke */
void _park(float alpha, float beta, float theta, float *d, float *q);
void _inv_park(float d, float q, float theta, float *alpha, float *beta);

void FOC_Init(void);
void FOC_BeginAlign(void);   /* 给固定电角度电压，把转子拉到对齐位置 */
void FOC_FinishAlign(void);  /* 记下此时编码器角作为电角度零点 */
void FOC_SetMode(FOC_Mode_t mode);
FOC_Mode_t FOC_GetMode(void);
void FOC_Step(float dt_s);   /* 1kHz 调用：读传感器、跑当前环、写 PWM */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
