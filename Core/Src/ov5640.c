/**
 ******************************************************************************
 * @file    ov5640.c
 * @brief   OV5640 Camera driver - WITH START/STOP
 ******************************************************************************
 */

#include "main.h"
#include "stm32h7xx_hal.h"
#include "ov5640.h"
#include "ov5640_reg.h"


extern I2C_HandleTypeDef hi2c1;

/* I2C address */
#define OV5640_I2C_ADDR   (0x78)

/* -----------------------------------------------------------
 * Low-level I2C functions
 * ----------------------------------------------------------- */
int32_t OV5640_ReadReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length)
{
    return (HAL_I2C_Mem_Read(&hi2c1, devAddr, reg,
                             I2C_MEMADD_SIZE_16BIT, pData, length,
                             HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

int32_t OV5640_WriteReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length)
{
    return (HAL_I2C_Mem_Write(&hi2c1, devAddr, reg,
                              I2C_MEMADD_SIZE_16BIT, pData, length,
                              HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

/* -----------------------------------------------------------
 * Register table loader
 * ----------------------------------------------------------- */
static int32_t write_table(const ov5640_reg_t *table)
{
    uint32_t i = 0;

    while (!(table[i].reg == 0xFFFF && table[i].val == 0xFF))
    {
        uint8_t v = table[i].val;

        OV5640_WriteReg(OV5640_I2C_ADDR, table[i].reg, &v, 1);
        HAL_Delay(1);
        i++;
    }

    return 0;
}

/* -----------------------------------------------------------
 * Probe()
 * ----------------------------------------------------------- */
int32_t OV5640_Probe(OV5640_Object_t *pObj)
{
    pObj->Ctx.ReadReg  = OV5640_ReadReg;
    pObj->Ctx.WriteReg = OV5640_WriteReg;
    pObj->Ctx.Address  = OV5640_I2C_ADDR;

    uint32_t id;
    if (OV5640_ReadID(pObj, &id) != 0)
        return -1;

    if (id != 0x5640)
        return -2;

    return 0;
}

/* -----------------------------------------------------------
 * Read ID
 * ----------------------------------------------------------- */
int32_t OV5640_ReadID(OV5640_Object_t *pObj, uint32_t *id)
{
    uint8_t high, low;

    if (pObj->Ctx.ReadReg(pObj->Ctx.Address, OV5640_CHIP_ID_HIGH, &high, 1) != 0)
        return -1;

    if (pObj->Ctx.ReadReg(pObj->Ctx.Address, OV5640_CHIP_ID_LOW, &low, 1) != 0)
        return -1;

    *id = ((uint32_t)high << 8) | low;
    return 0;
}

/* -----------------------------------------------------------
 * Generic INIT
 * ----------------------------------------------------------- */
int32_t OV5640_Init(OV5640_Object_t *pObj, uint32_t mode, uint32_t resolution)
{
    write_table(ov5640_init_reg);
    HAL_Delay(10);

    write_table(ov5640_rgb565_reg);
    HAL_Delay(10);

    write_table(ov5640_qqvga_reg);
    HAL_Delay(10);

    return 0;
}

/* -----------------------------------------------------------
 * START STREAMING - 중요!
 * ----------------------------------------------------------- */
int32_t OV5640_Start(OV5640_Object_t *pObj, uint32_t mode)
{
    write_table(ov5640_streaming_start);
    HAL_Delay(100);  // 스트리밍 시작 대기

    return 0;
}

/* -----------------------------------------------------------
 * STOP STREAMING
 * ----------------------------------------------------------- */
int32_t OV5640_Stop(OV5640_Object_t *pObj)
{
    uint8_t val = 0x0f;  // Stream OFF
    OV5640_WriteReg(OV5640_I2C_ADDR, 0x4202, &val, 1);

    return 0;
}

/* ============================================================
 *  Register Bus I/O
 * ============================================================ */
int32_t OV5640_RegisterBusIO(OV5640_Object_t *pObj, OV5640_IO_t *io)
{
    if (pObj == NULL || io == NULL)
        return OV5640_ERROR;

    pObj->Ctx.Address  = io->Address;
    pObj->Ctx.Init     = io->Init;
    pObj->Ctx.DeInit   = io->DeInit;
    pObj->Ctx.ReadReg  = io->ReadReg;
    pObj->Ctx.WriteReg = io->WriteReg;
    pObj->Ctx.GetTick  = io->GetTick;

    if (pObj->Ctx.Init != NULL)
        pObj->Ctx.Init();

    pObj->IsInitialized = 1;

    return OV5640_OK;
}

/* -----------------------------------------------------------
 * BSP Driver Structure
 * ----------------------------------------------------------- */
OV5640_CAMERA_Drv_t OV5640_CAMERA_Driver =
{
    .Init              = OV5640_Init,
    .ReadID            = OV5640_ReadID,
    .Start             = OV5640_Start,   // ← 추가!
    .Stop              = OV5640_Stop,    // ← 추가!
    .SetLightMode      = NULL,
    .SetColorEffect    = NULL,
    .SetBrightness     = NULL,
    .SetSaturation     = NULL,
    .SetContrast       = NULL,
    .MirrorFlipConfig  = NULL
};
