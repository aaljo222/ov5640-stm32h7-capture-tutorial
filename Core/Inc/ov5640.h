#ifndef __OV5640_H__
#define __OV5640_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef struct {
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef      *pwdn_port;
    uint16_t           pwdn_pin;
    GPIO_TypeDef      *rst_port;
    uint16_t           rst_pin;
} OV5640_Config_t;

extern OV5640_Config_t ov5640_config;

/* Public API */
void    OV5640_PowerUp(void);
void    OV5640_Reset(void);

uint8_t OV5640_WriteReg(uint16_t reg, uint8_t data);
uint8_t OV5640_ReadReg(uint16_t reg, uint8_t *data);

/* Main initialization for RGB565 QQVGA (160x120) */
HAL_StatusTypeDef OV5640_Init_RGB565_QQVGA(void);

#endif
