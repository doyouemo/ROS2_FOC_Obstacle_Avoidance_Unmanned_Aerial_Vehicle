/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    FOC.h
  * @brief   This file contains all the function prototypes for
  *          the FOC.c file
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
#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN FunctionPrototypes */

void _clack(float a, float b, float c, float *alpha, float *beta);
void _park(float alpha, float beta, float theta, float *d, float *q);

/* USER CODE END FunctionPrototypes */

#ifdef __cplusplus
}
#endif

#endif /*__ FOC_H__ */