#ifndef __AS5600_H__
#define __AS5600_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include <stdint.h>

#define AS5600_I2C_ADDR        0x36
#define AS5600_ANGLE_REG_H     0x0E
#define AS5600_ANGLE_REG_L     0x0F

uint8_t readAS5600Data(uint16_t regAddr, uint8_t *data, uint16_t size);

/* 成功返回 1 并写入 angle；失败返回 0，不改 *angle（由调用方决定是否沿用旧值） */
uint8_t AS5600_ReadAngle1(uint16_t *angle);
uint8_t AS5600_ReadAngle2(uint16_t *angle);

/* 兼容旧接口：失败时返回 0xFFFF 表示无效（勿当角度用） */
uint16_t AS5600_GetAngle1(void);
uint16_t AS5600_GetAngle2(void);

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H__ */
