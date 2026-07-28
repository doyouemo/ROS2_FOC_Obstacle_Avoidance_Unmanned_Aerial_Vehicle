/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "AS5600.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

uint8_t readAS5600Data(uint16_t regAddr, uint8_t *data, uint16_t size)
{
  /* USER CODE BEGIN 2 */
  return HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, size, 100);
  /* USER CODE END 2 */
}

uint16_t AS5600_GetAngle1(void)
{
  /* USER CODE BEGIN 3 */
  uint8_t angle_h, angle_l;
  uint16_t angle = 0;

  if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_H, I2C_MEMADD_SIZE_8BIT, &angle_h, 1, 100) == HAL_OK)
  {
    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_L, I2C_MEMADD_SIZE_8BIT, &angle_l, 1, 100) == HAL_OK)
    {
      angle = ((angle_h & 0x0F) << 8) | angle_l;
    }
  }

  return angle;
  /* USER CODE END 3 */
}

uint16_t AS5600_GetAngle2(void)
{
  /* USER CODE BEGIN 4 */
  uint8_t angle_h, angle_l;
  uint16_t angle = 0;

  if (HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_H, I2C_MEMADD_SIZE_8BIT, &angle_h, 1, 100) == HAL_OK)
  {
    if (HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR << 1, AS5600_ANGLE_REG_L, I2C_MEMADD_SIZE_8BIT, &angle_l, 1, 100) == HAL_OK)
    {
      angle = ((angle_h & 0x0F) << 8) | angle_l;
    }
  }

  return angle;
  /* USER CODE END 4 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
