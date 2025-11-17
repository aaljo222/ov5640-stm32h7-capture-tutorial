/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* Error handler */
void Error_Handler(void);

/* Pin Definitions */
#define OV5640_RST_Pin GPIO_PIN_1
#define OV5640_RST_GPIO_Port GPIOD
#define OV5640_PWDN_Pin GPIO_PIN_0
#define OV5640_PWDN_GPIO_Port GPIOD

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */