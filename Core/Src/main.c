#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "usb_device.h"

#include "usbd_cdc_if.h"
#include "ov5640.h"
#include "logic_analyser.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

/* -----------------------------------------------------------
 *  Modes
 * ----------------------------------------------------------- */
#define FW_MODE_ANALYZER   0
#define FW_MODE_CAMERA     1
#define FW_MODE    FW_MODE_ANALYZER     // ★ 여기서 모드를 선택!

/* -----------------------------------------------------------
 *  Configuration
 * ----------------------------------------------------------- */
#define FRAME_W     160
#define FRAME_H     120
#define FRAME_SIZE  (FRAME_W * FRAME_H * 2)

uint8_t frame_buf[FRAME_SIZE] __attribute__((section(".ram_d1")));
volatile uint8_t frame_captured = 0;

void SystemClock_Config(void);

/* -----------------------------------------------------------
 *  MCO1 = 24 MHz (XCLK to OV5640)
 * ----------------------------------------------------------- */
void MCO1_24MHz_Enable(void)
{
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_2);
}

/* -----------------------------------------------------------
 *  DCMI DMA Callback
 * ----------------------------------------------------------- */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    frame_captured = 1;
    uprintf("Frame IRQ\n");
}

/* -----------------------------------------------------------
 *  MAIN START
 * ----------------------------------------------------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_USB_DEVICE_Init();
    MX_I2C1_Init();
    MX_DCMI_Init();

    uprintf("\r\n=== System Boot ===\r\n");

    /* -----------------------------------------------------------
     *  Mode 0 : Logic Analyzer Mode
     * ----------------------------------------------------------- */
#if (FW_MODE == FW_MODE_ANALYZER)

    uprintf("Mode: LOGIC ANALYZER\r\n");
    HAL_Delay(100);

    // XCLK 필요 없음 / OV5640 init 필요 없음 → 순수 라인 파형 검사용
    while (1)
    {
        logic_analyzer_run();      // ★ 파형 측정
        HAL_Delay(200);            // 0.2초마다 한 번 출력
    }

#endif


    /* -----------------------------------------------------------
     *  Mode 1 : Camera Stream Mode (DCMI + USB CDC)
     * ----------------------------------------------------------- */
#if (FW_MODE == FW_MODE_CAMERA)

    uprintf("Mode: CAMERA STREAM\r\n");

    /* 1) Enable MCO clock */
    MCO1_24MHz_Enable();
    HAL_Delay(50);

    /* 2) OV5640 Power On / Reset */
    OV5640_PowerUp();
    OV5640_Reset();
    HAL_Delay(100);

    /* 3) OV5640 Init */
    if (OV5640_Init_RGB565_QQVGA() != 0)
    {
        uprintf("OV5640 init failed!\r\n");
    }
    else
    {
        uprintf("OV5640 init OK\r\n");
    }

    /* 4) Start continuous capture */
    if (HAL_DCMI_Start_DMA(&hdcmi,
                           DCMI_MODE_CONTINUOUS,
                           (uint32_t)frame_buf,
                           FRAME_SIZE / 4) != HAL_OK)
    {
        uprintf("DCMI DMA start failed!\r\n");
        while(1);
    }

    uprintf("DCMI DMA streaming...\r\n");

    /* Camera Mode Main Loop */
    while (1)
    {
        if (frame_captured)
        {
            frame_captured = 0;

            uint8_t header[4] = {0xAA, 0x55, FRAME_W, FRAME_H};
            CDC_Transmit_FS(header, 4);

            while (CDC_Transmit_FS(frame_buf, FRAME_SIZE) == USBD_BUSY);

            uprintf("Sent frame\r\n");
        }
    }
#endif

    // Should not reach here
    while (1);
}


/* -----------------------------------------------------------
 *  ERROR HANDLER
 * ----------------------------------------------------------- */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
