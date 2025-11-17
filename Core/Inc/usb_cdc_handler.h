/**
  ******************************************************************************
  * @file           : usb_cdc_handler.h
  * @brief          : Header for usb_cdc_handler.c file.
  ******************************************************************************
  */

#ifndef __USB_CDC_HANDLER_H
#define __USB_CDC_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include <stdint.h>

/* Buffer sizes */
#define CDC_TX_BUFFER_SIZE      (320 * 240 * 2)  // QVGA RGB565
#define CDC_PACKET_SIZE         512

/* Function Prototypes */
void USB_CDC_Init(void);
HAL_StatusTypeDef USB_CDC_TransmitImage(uint8_t *data, uint32_t length);
HAL_StatusTypeDef USB_CDC_TransmitPacket(uint8_t *data, uint16_t length);
uint8_t USB_CDC_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_CDC_HANDLER_H */