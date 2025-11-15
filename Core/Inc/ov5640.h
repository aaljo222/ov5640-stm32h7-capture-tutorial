#ifndef OV5640_H
#define OV5640_H

#include "stm32h7xx_hal.h"

#define OV5640_I2C_ADDR   0x3C   // 7bit address (0x78 >> 1)

HAL_StatusTypeDef OV5640_WriteReg(I2C_HandleTypeDef *hi2c,
                                  uint16_t reg,
                                  uint8_t val);

HAL_StatusTypeDef OV5640_ReadReg(I2C_HandleTypeDef *hi2c,
                                 uint16_t reg,
                                 uint8_t *val);

HAL_StatusTypeDef OV5640_InitRAW(I2C_HandleTypeDef *hi2c);
void OV5640_PowerUp(void);

#endif
