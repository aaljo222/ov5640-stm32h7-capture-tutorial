/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   I2C header file
  ******************************************************************************
  */

#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
