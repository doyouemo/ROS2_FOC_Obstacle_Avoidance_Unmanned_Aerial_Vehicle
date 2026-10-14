#include "AS5600.h"

uint8_t readAS5600Data(uint16_t regAddr, uint8_t *data, uint16_t size)
{
  return (uint8_t)HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR << 1, regAddr,
                                   I2C_MEMADD_SIZE_8BIT, data, size, 100);
}

uint8_t AS5600_ReadAngle1(uint16_t *angle)
{
  uint8_t angle_h;
  uint8_t angle_l;

  if (angle == NULL)
  {
    return 0U;
  }

  if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_H,
                       I2C_MEMADD_SIZE_8BIT, &angle_h, 1, 2) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_L,
                       I2C_MEMADD_SIZE_8BIT, &angle_l, 1, 2) != HAL_OK)
  {
    return 0U;
  }

  *angle = (uint16_t)(((angle_h & 0x0FU) << 8) | angle_l);
  return 1U;
}

uint8_t AS5600_ReadAngle2(uint16_t *angle)
{
  uint8_t angle_h;
  uint8_t angle_l;

  if (angle == NULL)
  {
    return 0U;
  }

  if (HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_H,
                       I2C_MEMADD_SIZE_8BIT, &angle_h, 1, 2) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_L,
                       I2C_MEMADD_SIZE_8BIT, &angle_l, 1, 2) != HAL_OK)
  {
    return 0U;
  }

  *angle = (uint16_t)(((angle_h & 0x0FU) << 8) | angle_l);
  return 1U;
}

uint16_t AS5600_GetAngle1(void)
{
  uint16_t angle = 0xFFFFU;
  if (AS5600_ReadAngle1(&angle) == 0U)
  {
    return 0xFFFFU;
  }
  return angle;
}

uint16_t AS5600_GetAngle2(void)
{
  uint16_t angle = 0xFFFFU;
  if (AS5600_ReadAngle2(&angle) == 0U)
  {
    return 0xFFFFU;
  }
  return angle;
}
