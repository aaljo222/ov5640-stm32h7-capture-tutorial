#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define JPEG_BUF_SIZE (60 * 1024)

extern uint8_t  jpeg_buf[JPEG_BUF_SIZE];
extern uint32_t jpeg_size;

void Error_Handler(void);
void uprintf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
