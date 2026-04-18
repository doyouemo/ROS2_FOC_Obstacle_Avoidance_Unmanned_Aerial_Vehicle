/**
 * @file AS5600.h
 * @brief AMS AS5600 magnetic rotary position sensor (I²C).
 * @note Register map and electrical limits per AS5600 datasheet v1-02 (2015-Nov-13).
 */
#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/** 7-bit I²C slave address (datasheet: 0x36). */
#define AS5600_I2C_ADDR_7BIT (0x36U)

/** STM32 HAL expects the 7-bit address shifted left by one bit. */
#define AS5600_I2C_ADDR_HAL ((uint16_t)(AS5600_I2C_ADDR_7BIT << 1))

/* Register addresses (word address) */
#define AS5600_REG_ZMCO      (0x00U)
#define AS5600_REG_ZPOS_H    (0x01U)
#define AS5600_REG_ZPOS_L    (0x02U)
#define AS5600_REG_MPOS_H    (0x03U)
#define AS5600_REG_MPOS_L    (0x04U)
#define AS5600_REG_MANG_H    (0x05U)
#define AS5600_REG_MANG_L    (0x06U)
/** Low byte of CONF (14-bit): PM, HYST, OUTS, PWMF — register 0x07. */
#define AS5600_REG_CONF_LO   (0x07U)
/** High byte of CONF: SF, FTH, WD — register 0x08. */
#define AS5600_REG_CONF_HI   (0x08U)
#define AS5600_REG_STATUS    (0x0BU)
#define AS5600_REG_RAW_ANGLE_H (0x0CU)
#define AS5600_REG_RAW_ANGLE_L (0x0DU)
#define AS5600_REG_ANGLE_H   (0x0EU)
#define AS5600_REG_ANGLE_L   (0x0FU)
#define AS5600_REG_AGC       (0x1AU)
#define AS5600_REG_MAG_H     (0x1BU)
#define AS5600_REG_MAG_L     (0x1CU)
#define AS5600_REG_BURN      (0xFFU)

/** 12-bit angle masks */
#define AS5600_ANGLE_MASK    (0x0FFFU)

/* STATUS (0x0B) bits */
#define AS5600_STATUS_MH     (1U << 3)
#define AS5600_STATUS_ML     (1U << 4)
#define AS5600_STATUS_MD     (1U << 5)

/* BURN (0xFF) command values (datasheet) */
#define AS5600_BURN_ANGLE    (0x80U)
#define AS5600_BURN_SETTING  (0x40U)

/* CONF as one 14-bit value (Figure 22): bits 0–13. */
#define AS5600_CONF_PM_Pos   (0U)
#define AS5600_CONF_PM_Msk   (0x3U << AS5600_CONF_PM_Pos)
#define AS5600_CONF_HYST_Pos (2U)
#define AS5600_CONF_HYST_Msk (0x3U << AS5600_CONF_HYST_Pos)
#define AS5600_CONF_OUTS_Pos (4U)
#define AS5600_CONF_OUTS_Msk (0x3U << AS5600_CONF_OUTS_Pos)
#define AS5600_CONF_PWMF_Pos (6U)
#define AS5600_CONF_PWMF_Msk (0x3U << AS5600_CONF_PWMF_Pos)
#define AS5600_CONF_SF_Pos   (8U)
#define AS5600_CONF_SF_Msk   (0x3U << AS5600_CONF_SF_Pos)
#define AS5600_CONF_FTH_Pos  (10U)
#define AS5600_CONF_FTH_Msk  (0x7U << AS5600_CONF_FTH_Pos)
#define AS5600_CONF_WD_Pos   (13U)
#define AS5600_CONF_WD_Msk   (0x1U << AS5600_CONF_WD_Pos)
#define AS5600_CONF_MASK14   (0x3FFFU)

#ifndef AS5600_I2C_TIMEOUT_MS
#define AS5600_I2C_TIMEOUT_MS (50U)
#endif

typedef struct {
  uint8_t zmco;
  uint16_t raw_angle;
  uint16_t angle;
  uint8_t status;
  uint8_t agc;
  uint16_t magnitude;
} AS5600_Snapshot_t;

HAL_StatusTypeDef AS5600_ReadReg8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val);
HAL_StatusTypeDef AS5600_WriteReg8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);
HAL_StatusTypeDef AS5600_ReadReg12(I2C_HandleTypeDef *hi2c, uint8_t reg_hi, uint16_t *val12);

HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle12);
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw12);
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);
HAL_StatusTypeDef AS5600_ReadAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc);
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *mag12);
HAL_StatusTypeDef AS5600_ReadZMCO(I2C_HandleTypeDef *hi2c, uint8_t *zmco);

HAL_StatusTypeDef AS5600_ReadConf(I2C_HandleTypeDef *hi2c, uint16_t *conf14);
HAL_StatusTypeDef AS5600_WriteConf(I2C_HandleTypeDef *hi2c, uint16_t conf14);

HAL_StatusTypeDef AS5600_ReadSnapshot(I2C_HandleTypeDef *hi2c, AS5600_Snapshot_t *out);

/** Convert 12-bit angle (0–4095) to degrees in [0.0f, 360.0f). */
float AS5600_Angle12_ToDegrees(uint16_t angle12);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */
