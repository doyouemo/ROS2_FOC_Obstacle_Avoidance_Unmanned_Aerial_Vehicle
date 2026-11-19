#include "AS5600.h"

uint8_t readAS5600Data(uint16_t regAddr, uint8_t *data, uint16_t size)
{
  return (uint8_t)HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR << 1, regAddr,
                                   I2C_MEMADD_SIZE_8BIT, data, size, 100);
}

static uint8_t as5600_read_angle(I2C_HandleTypeDef *hi2c, uint16_t *angle)
{
  uint8_t buf[2];

  if (angle == NULL)
  {
    return 0U;
  }

  /* 一次读高低字节，避免两次读取中间角度翻转造成跳变 */
  if (HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_H,
                       I2C_MEMADD_SIZE_8BIT, buf, 2, 2) != HAL_OK)
  {
    return 0U;
  }

  *angle = (uint16_t)(((buf[0] & 0x0FU) << 8) | buf[1]);
  return 1U;
}

uint8_t AS5600_ReadAngle1(uint16_t *angle)
{
  return as5600_read_angle(&hi2c1, angle);
}

uint8_t AS5600_ReadAngle2(uint16_t *angle)
{
  return as5600_read_angle(&hi2c2, angle);
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
