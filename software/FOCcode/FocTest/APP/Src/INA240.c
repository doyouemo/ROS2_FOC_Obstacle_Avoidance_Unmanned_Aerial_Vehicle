#include "INA240.h"

static const uint32_t ina240_adc_channel[INA240_CH_NUM] = {
  ADC_CHANNEL_9, /* PB1 */
  ADC_CHANNEL_8, /* PB0 */
  ADC_CHANNEL_4, /* PA4 */
  ADC_CHANNEL_5, /* PA5 */
};

void INA240_Init(void)
{
  HAL_ADCEx_Calibration_Start(&hadc1);
}

uint16_t INA240_ReadRaw(INA240_Channel_t ch)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint16_t value = 0;

  if (ch >= INA240_CH_NUM)
  {
    return 0;
  }

  sConfig.Channel = ina240_adc_channel[ch];
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return 0;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }

  HAL_ADC_Stop(&hadc1);
  return value;
}

void INA240_ReadAllRaw(uint16_t raw[INA240_CH_NUM])
{
  uint8_t i;

  for (i = 0; i < INA240_CH_NUM; i++)
  {
    raw[i] = INA240_ReadRaw((INA240_Channel_t)i);
  }
}
