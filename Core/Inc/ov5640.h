/**
 ******************************************************************************
 * @file    ov5640.h
 * @brief   OV5640 Camera driver header
 ******************************************************************************
 */

#ifndef OV5640_H
#define OV5640_H

#include <stdint.h>
#include "ov5640_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Address */
#define OV5640_I2C_ADDR   (0x78)

/* Low-level I/O */
int32_t OV5640_ReadReg(uint16_t devAddr, uint16_t reg,
                       uint8_t *pData, uint16_t length);

int32_t OV5640_WriteReg(uint16_t devAddr, uint16_t reg,
                        uint8_t *pData, uint16_t length);

/* Chip ID Registers */
#define OV5640_CHIP_ID_HIGH   0x300A
#define OV5640_CHIP_ID_LOW    0x300B

/* Forward declaration */
struct OV5640_Object_s;
typedef struct OV5640_Object_s OV5640_Object_t;

/* I/O Context */
typedef struct
{
    int32_t (*Init)(void);
    int32_t (*DeInit)(void);

    uint16_t Address;

    int32_t (*ReadReg)(uint16_t, uint16_t, uint8_t*, uint16_t);
    int32_t (*WriteReg)(uint16_t, uint16_t, uint8_t*, uint16_t);

    uint32_t (*GetTick)(void);

} OV5640_IO_t;

/* OV5640 Object */
struct OV5640_Object_s
{
    OV5640_IO_t Ctx;
    uint32_t IsInitialized;
};

/* BSP Driver Structure */
typedef struct
{
    int32_t (*Init)(OV5640_Object_t*, uint32_t, uint32_t);
    int32_t (*ReadID)(OV5640_Object_t*, uint32_t *id);

    int32_t (*Start)(OV5640_Object_t*, uint32_t);
    int32_t (*Stop)(OV5640_Object_t*);

    int32_t (*SetLightMode)(OV5640_Object_t*, uint32_t);
    int32_t (*SetColorEffect)(OV5640_Object_t*, uint32_t);
    int32_t (*SetBrightness)(OV5640_Object_t*, int32_t);
    int32_t (*SetSaturation)(OV5640_Object_t*, int32_t);
    int32_t (*SetContrast)(OV5640_Object_t*, int32_t);

    int32_t (*MirrorFlipConfig)(OV5640_Object_t*, uint32_t);

} OV5640_CAMERA_Drv_t;

/* API Functions */
int32_t OV5640_RegisterBusIO(OV5640_Object_t *pObj, OV5640_IO_t *io);
int32_t OV5640_Probe(OV5640_Object_t *pObj);
int32_t OV5640_ReadID(OV5640_Object_t *pObj, uint32_t *id);
int32_t OV5640_Init(OV5640_Object_t *pObj, uint32_t mode, uint32_t resolution);
int32_t OV5640_Start(OV5640_Object_t *pObj, uint32_t mode);  // ← 추가!
int32_t OV5640_Stop(OV5640_Object_t *pObj);                  // ← 추가!

/* Global driver instance */
extern OV5640_CAMERA_Drv_t OV5640_CAMERA_Driver;

/* Return Codes */
#define OV5640_OK       0
#define OV5640_ERROR   -1

/* Color Modes */
#define MODE_RGB565     0x01

/* Resolution IDs */
#define RESOLUTION_R160x120    0x11  /* QQVGA */

#ifdef __cplusplus
}
#endif

#endif /* OV5640_H */
