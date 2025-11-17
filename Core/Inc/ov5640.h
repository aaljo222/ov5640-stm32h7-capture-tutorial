/**
  ******************************************************************************
  * @file           : ov5640.h
  * @brief          : Header for ov5640.c file.
  *                   OV5640 camera driver for STM32H7
  ******************************************************************************
  */

#ifndef __OV5640_H
#define __OV5640_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* OV5640 I2C Address */
#define OV5640_I2C_ADDR         0x78  // 7-bit: 0x3C, 8-bit write: 0x78

/* OV5640 Register Addresses */
#define OV5640_CHIPID_HIGH      0x300A
#define OV5640_CHIPID_LOW       0x300B
#define OV5640_CHIP_ID          0x5640

/* Image Formats */
#define OV5640_FORMAT_RGB565    0
#define OV5640_FORMAT_JPEG      1

/* Image Resolutions */
typedef enum {
    OV5640_RES_QVGA = 0,    // 320x240
    OV5640_RES_VGA,         // 640x480
    OV5640_RES_SVGA,        // 800x600
    OV5640_RES_720P,        // 1280x720
} OV5640_Resolution_t;

/* OV5640 Configuration Structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef *pwdn_port;
    uint16_t pwdn_pin;
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;
    uint8_t image_format;
    OV5640_Resolution_t resolution;
} OV5640_Config_t;

/* Function Prototypes */
HAL_StatusTypeDef OV5640_Init(OV5640_Config_t *config);
HAL_StatusTypeDef OV5640_ReadID(OV5640_Config_t *config, uint16_t *id);
HAL_StatusTypeDef OV5640_SetResolution(OV5640_Config_t *config, OV5640_Resolution_t res);
HAL_StatusTypeDef OV5640_SetFormat(OV5640_Config_t *config, uint8_t format);
HAL_StatusTypeDef OV5640_StartCapture(OV5640_Config_t *config);
HAL_StatusTypeDef OV5640_StopCapture(OV5640_Config_t *config);
void OV5640_HardwareReset(OV5640_Config_t *config);
void OV5640_PowerDown(OV5640_Config_t *config, uint8_t enable);

/* Low-level I2C functions */
HAL_StatusTypeDef OV5640_WriteReg(OV5640_Config_t *config, uint16_t reg, uint8_t data);
HAL_StatusTypeDef OV5640_ReadReg(OV5640_Config_t *config, uint16_t reg, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __OV5640_H */