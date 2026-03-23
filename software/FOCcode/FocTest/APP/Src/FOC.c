/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    FOC.c
  * @brief   This file provides code for the FOC algorithm
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

/* Includes ------------------------------------------------------------------*/
#include "FOC.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

void _clack(float a, float b, float c, float *alpha, float *beta)
{
  /* USER CODE BEGIN 2 */
  *alpha = (2.0f/3.0f) * (a - 0.5f * b - 0.5f * c);
  *beta = (2.0f/3.0f) * (sqrtf(3.0f)/2.0f * b - sqrtf(3.0f)/2.0f * c);
  /* USER CODE END 2 */
}

void _park(float alpha, float beta, float theta, float *d, float *q)
{
  /* USER CODE BEGIN 3 */
  *d = cosf(theta) * alpha - sinf(theta) * beta;
  *q = sinf(theta) * alpha + cosf(theta) * beta;
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */