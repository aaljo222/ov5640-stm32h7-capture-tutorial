/**
  ******************************************************************************
  * @file           : ov5640.c
  * @brief          : OV5640 camera driver implementation
  ******************************************************************************
  */

#include "ov5640.h"
#include <string.h>

/* Register configuration arrays */
typedef struct {
    uint16_t reg;
    uint8_t val;
} OV5640_Reg_t;

/* OV5640 Initialization Register Settings */
static const OV5640_Reg_t OV5640_InitRegs[] = {
    // System Reset
    {0x3103, 0x11},
    {0x3008, 0x82},  // Software reset
    
    // IO Control
    {0x3017, 0xff},
    {0x3018, 0xff},
    
    // Clock Settings
    {0x3034, 0x1a},  // PLL settings
    {0x3035, 0x11},
    {0x3036, 0x46},
    {0x3037, 0x13},
    
    // System Control
    {0x3108, 0x01},
    {0x3630, 0x36},
    {0x3631, 0x0e},
    {0x3632, 0xe2},
    {0x3633, 0x12},
    
    // AEC/AGC Control
    {0x3a02, 0x03},
    {0x3a03, 0xd8},
    {0x3a08, 0x01},
    {0x3a09, 0x27},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0d, 0x04},
    {0x3a0e, 0x03},
    
    // 50/60Hz Detection
    {0x3c01, 0x34},
    {0x3c04, 0x28},
    {0x3c05, 0x98},
    {0x3c07, 0x07},
    {0x3c08, 0x00},
    {0x3c09, 0x1c},
    {0x3c0a, 0x9c},
    {0x3c0b, 0x40},
    
    // BLC Control
    {0x4001, 0x02},
    {0x4004, 0x02},
    
    // Format Control
    {0x4300, 0x61},  // RGB565
    
    // ISP Control
    {0x5000, 0xa7},
    {0x5001, 0xa3},
};

/* QVGA 320x240 Configuration */
static const OV5640_Reg_t OV5640_QVGA_Regs[] = {
    {0x3035, 0x14},  // PLL
    {0x3036, 0x38},
    {0x3c07, 0x08},
    {0x3820, 0x41},  // Timing
    {0x3821, 0x07},
    {0x3814, 0x31},  // X increment
    {0x3815, 0x31},  // Y increment
    {0x3800, 0x00},  // X start
    {0x3801, 0x00},
    {0x3802, 0x00},  // Y start
    {0x3803, 0x04},
    {0x3804, 0x0a},  // X end
    {0x3805, 0x3f},
    {0x3806, 0x07},  // Y end
    {0x3807, 0x9b},
    {0x3808, 0x01},  // X output size (320)
    {0x3809, 0x40},
    {0x380a, 0x00},  // Y output size (240)
    {0x380b, 0xf0},
    {0x380c, 0x07},  // HTS
    {0x380d, 0x68},
    {0x380e, 0x03},  // VTS
    {0x380f, 0xd8},
    {0x3813, 0x06},
    {0x3618, 0x00},
    {0x3612, 0x29},
    {0x3709, 0x52},
    {0x370c, 0x03},
    {0x4004, 0x02},
    {0x3002, 0x1c},
    {0x3006, 0xc3},
    {0x4713, 0x03},
    {0x4407, 0x04},
    {0x460b, 0x35},
    {0x460c, 0x22},
    {0x4837, 0x22},
};

/**
  * @brief  Write to OV5640 register
  */
HAL_StatusTypeDef OV5640_WriteReg(OV5640_Config_t *config, uint16_t reg, uint8_t data)
{
    uint8_t buf[3];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = data;
    
    return HAL_I2C_Master_Transmit(config->hi2c, OV5640_I2C_ADDR, buf, 3, 1000);
}

/**
  * @brief  Read from OV5640 register
  */
HAL_StatusTypeDef OV5640_ReadReg(OV5640_Config_t *config, uint16_t reg, uint8_t *data)
{
    uint8_t reg_addr[2];
    reg_addr[0] = (reg >> 8) & 0xFF;
    reg_addr[1] = reg & 0xFF;
    
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(config->hi2c, OV5640_I2C_ADDR, reg_addr, 2, 1000);
    if (status != HAL_OK) return status;
    
    return HAL_I2C_Master_Receive(config->hi2c, OV5640_I2C_ADDR, data, 1, 1000);
}

/**
  * @brief  Hardware reset OV5640
  */
void OV5640_HardwareReset(OV5640_Config_t *config)
{
    HAL_GPIO_WritePin(config->rst_port, config->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(config->rst_port, config->rst_pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

/**
  * @brief  Power down control
  */
void OV5640_PowerDown(OV5640_Config_t *config, uint8_t enable)
{
    if (enable) {
        HAL_GPIO_WritePin(config->pwdn_port, config->pwdn_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(config->pwdn_port, config->pwdn_pin, GPIO_PIN_RESET);
    }
    HAL_Delay(10);
}

/**
  * @brief  Read chip ID
  */
HAL_StatusTypeDef OV5640_ReadID(OV5640_Config_t *config, uint16_t *id)
{
    uint8_t id_high, id_low;
    HAL_StatusTypeDef status;
    
    status = OV5640_ReadReg(config, OV5640_CHIPID_HIGH, &id_high);
    if (status != HAL_OK) return status;
    
    status = OV5640_ReadReg(config, OV5640_CHIPID_LOW, &id_low);
    if (status != HAL_OK) return status;
    
    *id = (id_high << 8) | id_low;
    return HAL_OK;
}

/**
  * @brief  Initialize OV5640
  */
HAL_StatusTypeDef OV5640_Init(OV5640_Config_t *config)
{
    HAL_StatusTypeDef status;
    uint16_t chip_id;
    
    // Power up sequence
    OV5640_PowerDown(config, 0);
    HAL_Delay(5);
    OV5640_HardwareReset(config);
    HAL_Delay(20);
    
    // Check chip ID
    status = OV5640_ReadID(config, &chip_id);
    if (status != HAL_OK || chip_id != OV5640_CHIP_ID) {
        return HAL_ERROR;
    }
    
    // Software reset
    OV5640_WriteReg(config, 0x3008, 0x82);
    HAL_Delay(10);
    
    // Write initialization registers
    for (uint32_t i = 0; i < sizeof(OV5640_InitRegs) / sizeof(OV5640_Reg_t); i++) {
        status = OV5640_WriteReg(config, OV5640_InitRegs[i].reg, OV5640_InitRegs[i].val);
        if (status != HAL_OK) return status;
    }
    
    HAL_Delay(100);
    
    // Set default resolution (QVGA)
    status = OV5640_SetResolution(config, config->resolution);
    if (status != HAL_OK) return status;
    
    // Set format (RGB565)
    status = OV5640_SetFormat(config, config->image_format);
    
    return status;
}

/**
  * @brief  Set image resolution
  */
HAL_StatusTypeDef OV5640_SetResolution(OV5640_Config_t *config, OV5640_Resolution_t res)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    switch (res) {
        case OV5640_RES_QVGA:
            for (uint32_t i = 0; i < sizeof(OV5640_QVGA_Regs) / sizeof(OV5640_Reg_t); i++) {
                status = OV5640_WriteReg(config, OV5640_QVGA_Regs[i].reg, OV5640_QVGA_Regs[i].val);
                if (status != HAL_OK) return status;
            }
            break;
            
        // VGA, SVGA, 720P 설정 추가 가능
        default:
            return HAL_ERROR;
    }
    
    config->resolution = res;
    HAL_Delay(100);
    
    return status;
}

/**
  * @brief  Set image format
  */
HAL_StatusTypeDef OV5640_SetFormat(OV5640_Config_t *config, uint8_t format)
{
    HAL_StatusTypeDef status;
    
    if (format == OV5640_FORMAT_RGB565) {
        status = OV5640_WriteReg(config, 0x4300, 0x61);  // RGB565
    } else if (format == OV5640_FORMAT_JPEG) {
        status = OV5640_WriteReg(config, 0x4300, 0x30);  // JPEG
    } else {
        return HAL_ERROR;
    }
    
    config->image_format = format;
    return status;
}

/**
  * @brief  Start image capture
  */
HAL_StatusTypeDef OV5640_StartCapture(OV5640_Config_t *config)
{
    return OV5640_WriteReg(config, 0x3008, 0x02);  // Normal mode
}

/**
  * @brief  Stop image capture
  */
HAL_StatusTypeDef OV5640_StopCapture(OV5640_Config_t *config)
{
    return OV5640_WriteReg(config, 0x3008, 0x42);  // Power down mode
}