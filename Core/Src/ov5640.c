/* ========================================
   OV5640 드라이버 - 최종 정리 버전
   - 실제 핀: PWDN=PD0, RST=PD1
   - JPEG 모드 초기화
   ======================================== */

#include "ov5640.h"
#include "main.h"

#define OV5640_I2C_ADDR 0x3C

/* 레지스터 쓰기 */
HAL_StatusTypeDef OV5640_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t val)
{
    uint8_t buf[3];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = val;

    return HAL_I2C_Master_Transmit(hi2c, OV5640_I2C_ADDR << 1, buf, 3, 100);
}

/* 레지스터 읽기 */
HAL_StatusTypeDef OV5640_ReadReg(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t *val)
{
    uint8_t addr[2];
    addr[0] = (reg >> 8) & 0xFF;
    addr[1] = reg & 0xFF;

    if (HAL_I2C_Master_Transmit(hi2c, OV5640_I2C_ADDR << 1, addr, 2, 100) != HAL_OK)
        return HAL_ERROR;

    return HAL_I2C_Master_Receive(hi2c, OV5640_I2C_ADDR << 1, val, 1, 100);
}

/* 파워업 시퀀스 - 실제 핀 PD0, PD1 */
void OV5640_PowerUp(void)
{
    // PWDN = Low (정상 동작)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(5);

    // RESET 시퀀스
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(20);
}

/* JPEG 모드 초기화 */
HAL_StatusTypeDef OV5640_InitJPEG(I2C_HandleTypeDef *hi2c)
{
    uint8_t id_high = 0, id_low = 0;

    /* Chip ID 확인 */
    if (OV5640_ReadReg(hi2c, 0x300A, &id_high) != HAL_OK)
    {
        uprintf("Failed to read Chip ID High\r\n");
        return HAL_ERROR;
    }

    if (OV5640_ReadReg(hi2c, 0x300B, &id_low) != HAL_OK)
    {
        uprintf("Failed to read Chip ID Low\r\n");
        return HAL_ERROR;
    }

    uprintf("OV5640 Chip ID = 0x%02X%02X ", id_high, id_low);

    if (id_high != 0x56 || id_low != 0x40)
    {
        uprintf("(FAIL - Expected 0x5640)\r\n");
        return HAL_ERROR;
    }

    uprintf("(OK)\r\n");

    /* 소프트웨어 리셋 */
    uprintf("Software Reset...\r\n");
    OV5640_WriteReg(hi2c, 0x3008, 0x82);
    HAL_Delay(10);
    OV5640_WriteReg(hi2c, 0x3008, 0x02);
    HAL_Delay(5);

    /* 기본 설정 */
    uprintf("Configuring OV5640 for JPEG...\r\n");

    // System Clock
    OV5640_WriteReg(hi2c, 0x3103, 0x11);
    OV5640_WriteReg(hi2c, 0x3008, 0x02);

    // PLL 설정 (24MHz XCLK 기준)
    OV5640_WriteReg(hi2c, 0x3034, 0x18);
    OV5640_WriteReg(hi2c, 0x3035, 0x21);
    OV5640_WriteReg(hi2c, 0x3036, 0x69);
    OV5640_WriteReg(hi2c, 0x3037, 0x13);

    HAL_Delay(10);

    // JPEG 품질
    OV5640_WriteReg(hi2c, 0x4407, 0x08);
    OV5640_WriteReg(hi2c, 0x4300, 0x30);

    // 640x480 해상도
    OV5640_WriteReg(hi2c, 0x3808, 0x02);  // Width High
    OV5640_WriteReg(hi2c, 0x3809, 0x80);  // Width Low (640)
    OV5640_WriteReg(hi2c, 0x380A, 0x01);  // Height High
    OV5640_WriteReg(hi2c, 0x380B, 0xE0);  // Height Low (480)

    // ISP 제어
    OV5640_WriteReg(hi2c, 0x5000, 0xA7);
    OV5640_WriteReg(hi2c, 0x5001, 0xA3);

    HAL_Delay(50);

    uprintf("OV5640 JPEG Init Complete!\r\n");
    return HAL_OK;
}

/* RAW 모드는 JPEG로 통합 */
HAL_StatusTypeDef OV5640_InitRAW(I2C_HandleTypeDef *hi2c)
{
    return OV5640_InitJPEG(hi2c);
}
