#ifndef __INA240_H__
#define __INA240_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "adc.h"

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
uint16_t INA240_ReadRaw(INA240_Channel_t ch);
void INA240_ReadAllRaw(uint16_t raw[INA240_CH_NUM]);

#ifdef __cplusplus
}
#endif

#endif /* __INA240_H__ */
