/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AS5600_H__
#define __AS5600_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define AS5600_I2C_ADDR         0x36
#define AS5600_ANGLE_REG_H     0x0E
#define AS5600_ANGLE_REG_L     0x0F

/* USER CODE END Private defines */

/* USER CODE BEGIN FunctionPrototypes */

uint8_t readAS5600Data(uint16_t regAddr, uint8_t *data, uint16_t size);
uint16_t AS5600_GetAngle1(void);
uint16_t AS5600_GetAngle2(void);

/* USER CODE END FunctionPrototypes */

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H__ */