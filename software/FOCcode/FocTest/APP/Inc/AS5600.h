/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    AS5600.h
  * @brief   This file contains all the function prototypes for
  *          the AS5600.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AS5600_H__
#define __AS5600_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define AS5600_I2C_ADDR         0x36

/* USER CODE END Private defines */

/* USER CODE BEGIN FunctionPrototypes */

uint16_t AS5600_GetAngle(void);

/* USER CODE END FunctionPrototypes */

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H__ */