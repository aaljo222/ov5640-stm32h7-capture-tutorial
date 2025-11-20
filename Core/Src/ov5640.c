#include "ov5640.h"
#include "i2c.h"
#include "gpio.h"

OV5640_Config_t ov5640_config = {
    .hi2c = &hi2c1,
    .pwdn_port = GPIOD,
    .pwdn_pin  = GPIO_PIN_0,
    .rst_port  = GPIOD,
    .rst_pin   = GPIO_PIN_1
};

#define OV5640_ADDR 0x78  // 7-bit: 0x3C

/* -------------------------------------------------------------
 *  Low-level I2C
 * ------------------------------------------------------------- */
static HAL_StatusTypeDef I2C_Write(uint16_t reg, uint8_t data)
{
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, data };
    return HAL_I2C_Master_Transmit(ov5640_config.hi2c, OV5640_ADDR, buf, 3, 100);
}

uint8_t OV5640_WriteReg(uint16_t reg, uint8_t data)
{
    return I2C_Write(reg, data) == HAL_OK ? 0 : 1;
}

static HAL_StatusTypeDef I2C_Read(uint16_t reg, uint8_t *data)
{
    uint8_t buf[2] = { reg >> 8, reg & 0xFF };

    if (HAL_I2C_Master_Transmit(ov5640_config.hi2c, OV5640_ADDR, buf, 2, 100) != HAL_OK)
        return HAL_ERROR;

    return HAL_I2C_Master_Receive(ov5640_config.hi2c, OV5640_ADDR, data, 1, 100);
}

uint8_t OV5640_ReadReg(uint16_t reg, uint8_t *data)
{
    return I2C_Read(reg, data) == HAL_OK ? 0 : 1;
}

/* -------------------------------------------------------------
 *  Power & Reset
 * ------------------------------------------------------------- */
void OV5640_PowerUp(void)
{
    HAL_GPIO_WritePin(ov5640_config.pwdn_port, ov5640_config.pwdn_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ov5640_config.rst_port, ov5640_config.rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ov5640_config.rst_port, ov5640_config.rst_pin, GPIO_PIN_SET);
    HAL_Delay(20);
}

void OV5640_Reset(void)
{
    OV5640_WriteReg(0x3008, 0x82);  // software reset
    HAL_Delay(10);
    OV5640_WriteReg(0x3008, 0x42);
    HAL_Delay(20);
}

/* -------------------------------------------------------------
 *  RGB565 QQVGA (160x120)
 * ------------------------------------------------------------- */
HAL_StatusTypeDef OV5640_Init_RGB565_QQVGA(void)
{
    /* Sensor clock / reset */
    OV5640_Reset();

    /* RGB565 */
    OV5640_WriteReg(0x4300, 0x6F);

    /* Output size 160 x 120 */
    OV5640_WriteReg(0x3800, 0x00);
    OV5640_WriteReg(0x3801, 0x00);
    OV5640_WriteReg(0x3802, 0x00);
    OV5640_WriteReg(0x3803, 0x00);

    OV5640_WriteReg(0x3804, 0x0A);
    OV5640_WriteReg(0x3805, 0x3F);
    OV5640_WriteReg(0x3806, 0x07);
    OV5640_WriteReg(0x3807, 0x9F);

    OV5640_WriteReg(0x3808, 0x00); // width = 160
    OV5640_WriteReg(0x3809, 0xA0);

    OV5640_WriteReg(0x380A, 0x00); // height = 120
    OV5640_WriteReg(0x380B, 0x78);

    return HAL_OK;
}
