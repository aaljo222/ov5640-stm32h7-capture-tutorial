/**
  ******************************************************************************
  * @file           : usb_cdc_handler.c
  * @brief          : USB CDC data transmission handler - FIXED
  ******************************************************************************
  */

#include "usb_cdc_handler.h"
#include "usbd_cdc_if.h"
#include <string.h>

static volatile uint8_t cdc_busy = 0;

/**
  * @brief  Initialize USB CDC
  */
void USB_CDC_Init(void)
{
    cdc_busy = 0;
}

/**
  * @brief  Check if CDC is busy
  */
uint8_t USB_CDC_IsBusy(void)
{
    return cdc_busy;
}

/**
  * @brief  Transmit single packet via USB CDC
  */
HAL_StatusTypeDef USB_CDC_TransmitPacket(uint8_t *data, uint16_t length)
{
    if (length > CDC_PACKET_SIZE || length == 0) {
        return HAL_ERROR;
    }

    /* Wait if busy */
    uint32_t timeout = HAL_GetTick() + 1000;
    while (cdc_busy && HAL_GetTick() < timeout) {
        HAL_Delay(1);
    }

    if (cdc_busy) {
        return HAL_TIMEOUT;  // Still busy after timeout
    }

    cdc_busy = 1;
    uint8_t result = CDC_Transmit_FS(data, length);

    if (result != USBD_OK) {
        cdc_busy = 0;
        return HAL_ERROR;
    }

    /* Wait for transmission complete */
    timeout = HAL_GetTick() + 100;
    while (cdc_busy && HAL_GetTick() < timeout) {
        HAL_Delay(1);
    }

    if (cdc_busy) {
        cdc_busy = 0;  // Force reset
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

/**
  * @brief  Transmit entire image via USB CDC
  */
HAL_StatusTypeDef USB_CDC_TransmitImage(uint8_t *data, uint32_t length)
{
    uint32_t sent = 0;
    HAL_StatusTypeDef status;

    /* Send frame header: [0xAA 0x55] [size:4bytes] */
    uint8_t header[8];
    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = 0xAA;
    header[3] = 0x55;
    memcpy(&header[4], &length, 4);  // Little endian

    status = USB_CDC_TransmitPacket(header, 8);
    if (status != HAL_OK) {
        return status;
    }

    /* Send image data in packets */
    while (sent < length) {
        uint16_t chunk_size = (length - sent) > CDC_PACKET_SIZE ?
                              CDC_PACKET_SIZE : (length - sent);

        status = USB_CDC_TransmitPacket(&data[sent], chunk_size);
        if (status != HAL_OK) {
            return status;
        }

        sent += chunk_size;

        /* Small delay to prevent USB buffer overflow */
        if (chunk_size == CDC_PACKET_SIZE) {
            HAL_Delay(1);
        }
    }

    /* Send frame footer: [0x55 0xAA 0x55 0xAA] */
    uint8_t footer[4] = {0x55, 0xAA, 0x55, 0xAA};
    status = USB_CDC_TransmitPacket(footer, 4);

    return status;
}

/**
  * @brief  Transmission complete callback (called from USB interrupt)
  */
void USB_CDC_TxCplt(void)
{
    cdc_busy = 0;
}
