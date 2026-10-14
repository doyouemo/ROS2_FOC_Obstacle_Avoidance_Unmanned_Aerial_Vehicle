#ifndef __INA240_H__
#define __INA240_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "adc.h"
#include <stdint.h>

/* INA240 OUT -> MCU ADC
 * PB1 -> ADC1_IN9
 * PB0 -> ADC1_IN8
 * PA4 -> ADC1_IN4
 * PA5 -> ADC1_IN5
 */
typedef enum {
  INA240_CH1 = 0, /* PB1 */
  INA240_CH2,     /* PB0 */
  INA240_CH3,     /* PA4 */
  INA240_CH4,     /* PA5 */
  INA240_CH_NUM
} INA240_Channel_t;

void INA240_Init(void);

/* 成功返回 1；失败返回 0，不改 *value */
uint8_t INA240_ReadRawOk(INA240_Channel_t ch, uint16_t *value);

/* ok[i]=1 表示该通道有效；失败通道保持 raw[i] 原值不变（调用前可先填旧值） */
void INA240_ReadAllRawOk(uint16_t raw[INA240_CH_NUM], uint8_t ok[INA240_CH_NUM]);

uint16_t INA240_ReadRaw(INA240_Channel_t ch);
void INA240_ReadAllRaw(uint16_t raw[INA240_CH_NUM]);

#ifdef __cplusplus
}
#endif

#endif /* __INA240_H__ */
