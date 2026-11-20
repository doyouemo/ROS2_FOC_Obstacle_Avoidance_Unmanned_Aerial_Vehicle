#ifndef __AS5600_H__
#define __AS5600_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include <stdint.h>

#define AS5600_I2C_ADDR        0x36
#define AS5600_ANGLE_REG_H     0x0E

/* 成功返回 1 并写入 angle；失败返回 0，不改 *angle */
uint8_t AS5600_ReadAngle1(uint16_t *angle);
uint8_t AS5600_ReadAngle2(uint16_t *angle);

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H__ */
