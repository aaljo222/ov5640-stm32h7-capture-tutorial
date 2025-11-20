/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dcmi.h
  * @brief   This file contains all the function prototypes for
  *          the dcmi.c file
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DCMI_H__
#define __DCMI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern DCMI_HandleTypeDef hdcmi;

void MX_DCMI_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DCMI_H__ */
