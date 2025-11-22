/**
 ******************************************************************************
 * @file    ov5640_reg.h
 * @brief   OV5640 register table header
 ******************************************************************************
 */

#ifndef OV5640_REG_H
#define OV5640_REG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------
 * Register Table Struct
 * ----------------------------------------------------------- */
typedef struct
{
    uint16_t reg;   /* 16-bit OV5640 register address */
    uint8_t  val;   /* 8-bit value */
} ov5640_reg_t;

/* -----------------------------------------------------------
 * External register tables
 * ov5640_reg.c 에서 정의됨
 * ----------------------------------------------------------- */
extern const ov5640_reg_t ov5640_init_reg[];
extern const ov5640_reg_t ov5640_rgb565_reg[];
extern const ov5640_reg_t ov5640_qqvga_reg[];
extern const ov5640_reg_t ov5640_streaming_start[];  // ← 추가!

#ifdef __cplusplus
}
#endif

#endif /* OV5640_REG_H */
