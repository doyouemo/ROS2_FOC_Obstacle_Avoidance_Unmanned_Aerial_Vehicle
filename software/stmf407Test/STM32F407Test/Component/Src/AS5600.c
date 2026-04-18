/**
 * @file AS5600.c
 * @brief AMS AS5600 I²C driver (HAL).
 */
#include "AS5600.h"

static inline uint32_t as5600_timeout(void)
{
  return (uint32_t)AS5600_I2C_TIMEOUT_MS;
}

HAL_StatusTypeDef AS5600_ReadReg8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val)
{
  if (hi2c == NULL || val == NULL) {
    return HAL_ERROR;
  }
  return HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR_HAL, reg,
                          I2C_MEMADD_SIZE_8BIT, val, 1U, as5600_timeout());
}

HAL_StatusTypeDef AS5600_WriteReg8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
  if (hi2c == NULL) {
    return HAL_ERROR;
  }
  return HAL_I2C_Mem_Write(hi2c, AS5600_I2C_ADDR_HAL, reg,
                           I2C_MEMADD_SIZE_8BIT, &val, 1U, as5600_timeout());
}

HAL_StatusTypeDef AS5600_ReadReg12(I2C_HandleTypeDef *hi2c, uint8_t reg_hi, uint16_t *val12)
{
  uint8_t buf[2];

  if (hi2c == NULL || val12 == NULL) {
    return HAL_ERROR;
  }
  if (HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR_HAL, reg_hi,
                       I2C_MEMADD_SIZE_8BIT, buf, 2U, as5600_timeout()) != HAL_OK) {
    return HAL_ERROR;
  }
  *val12 = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]) & AS5600_ANGLE_MASK;
  return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle12)
{
  return AS5600_ReadReg12(hi2c, AS5600_REG_ANGLE_H, angle12);
}

HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw12)
{
  return AS5600_ReadReg12(hi2c, AS5600_REG_RAW_ANGLE_H, raw12);
}

HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
  return AS5600_ReadReg8(hi2c, AS5600_REG_STATUS, status);
}

HAL_StatusTypeDef AS5600_ReadAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc)
{
  return AS5600_ReadReg8(hi2c, AS5600_REG_AGC, agc);
}

HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *mag12)
{
  return AS5600_ReadReg12(hi2c, AS5600_REG_MAG_H, mag12);
}

HAL_StatusTypeDef AS5600_ReadZMCO(I2C_HandleTypeDef *hi2c, uint8_t *zmco)
{
  return AS5600_ReadReg8(hi2c, AS5600_REG_ZMCO, zmco);
}

HAL_StatusTypeDef AS5600_ReadConf(I2C_HandleTypeDef *hi2c, uint16_t *conf14)
{
  uint8_t lo;
  uint8_t hi;

  if (hi2c == NULL || conf14 == NULL) {
    return HAL_ERROR;
  }
  if (AS5600_ReadReg8(hi2c, AS5600_REG_CONF_LO, &lo) != HAL_OK) {
    return HAL_ERROR;
  }
  if (AS5600_ReadReg8(hi2c, AS5600_REG_CONF_HI, &hi) != HAL_OK) {
    return HAL_ERROR;
  }
  *conf14 = (((uint16_t)lo & 0xFFU) | (((uint16_t)hi & 0x3FU) << 8)) & AS5600_CONF_MASK14;
  return HAL_OK;
}

HAL_StatusTypeDef AS5600_WriteConf(I2C_HandleTypeDef *hi2c, uint16_t conf14)
{
  uint16_t v;
  uint8_t lo;
  uint8_t hi;

  if (hi2c == NULL) {
    return HAL_ERROR;
  }
  v = conf14 & AS5600_CONF_MASK14;
  lo = (uint8_t)(v & 0xFFU);
  hi = (uint8_t)((v >> 8) & 0x3FU);
  if (AS5600_WriteReg8(hi2c, AS5600_REG_CONF_LO, lo) != HAL_OK) {
    return HAL_ERROR;
  }
  return AS5600_WriteReg8(hi2c, AS5600_REG_CONF_HI, hi);
}

HAL_StatusTypeDef AS5600_ReadSnapshot(I2C_HandleTypeDef *hi2c, AS5600_Snapshot_t *out)
{
  if (hi2c == NULL || out == NULL) {
    return HAL_ERROR;
  }
  if (AS5600_ReadZMCO(hi2c, &out->zmco) != HAL_OK) {
    return HAL_ERROR;
  }
  if (AS5600_ReadRawAngle(hi2c, &out->raw_angle) != HAL_OK) {
    return HAL_ERROR;
  }
  if (AS5600_ReadAngle(hi2c, &out->angle) != HAL_OK) {
    return HAL_ERROR;
  }
  if (AS5600_ReadStatus(hi2c, &out->status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (AS5600_ReadAGC(hi2c, &out->agc) != HAL_OK) {
    return HAL_ERROR;
  }
  if (AS5600_ReadMagnitude(hi2c, &out->magnitude) != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

float AS5600_Angle12_ToDegrees(uint16_t angle12)
{
  float a = (float)(angle12 & AS5600_ANGLE_MASK);
  return (a * 360.0f) / 4096.0f;
}
