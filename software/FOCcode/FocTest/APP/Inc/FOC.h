#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* KEY1/2/3：停止 / 均速 / 锁位置。默认 STOP，电机无力矩。 */
typedef enum {
  FOC_MODE_STOP = 0,
  FOC_MODE_SPEED,
  FOC_MODE_POSITION
} FOC_Mode_t;

void FOC_Clarke(float a, float b, float c, float *alpha, float *beta);
void FOC_InvClarke(float alpha, float beta, float *a, float *b, float *c);
void FOC_Park(float alpha, float beta, float theta, float *d, float *q);
void FOC_InvPark(float d, float q, float theta, float *alpha, float *beta);

void FOC_Init(void);
void FOC_BeginAlign(void);
void FOC_FinishAlign(void);
void FOC_SetMode(FOC_Mode_t mode);
FOC_Mode_t FOC_GetMode(void);
void FOC_Step(float dt_s);

/* 日志 / 读数状态：1=本拍该路有效 */
float FOC_GetIq(uint8_t axis);
uint16_t FOC_GetAngleRaw(uint8_t axis);
uint16_t FOC_GetInaRaw(uint8_t ch);
uint8_t FOC_GetAngleOk(uint8_t axis);
uint8_t FOC_GetCurrentOk(uint8_t axis); /* 该轴两相电流本拍都成功 */
float FOC_GetUq(uint8_t axis);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
