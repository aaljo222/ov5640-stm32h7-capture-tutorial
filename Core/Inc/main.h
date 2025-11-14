#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"


/* OV5640 control pins */
#define CAM_PWDN_Pin          GPIO_PIN_0
#define CAM_PWDN_GPIO_Port    GPIOD
#define CAM_RESET_Pin         GPIO_PIN_1
#define CAM_RESET_GPIO_Port   GPIOD

void Error_Handler(void);
//void uprintf(const char *fmt, ...);
void dcmi_soft_analyzer(void);
#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
