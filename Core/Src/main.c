#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "usb_device.h"

#include "usbd_cdc_if.h"
#include "ov5640.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

/* -----------------------------------------------------------
 *  Configuration
 * ----------------------------------------------------------- */
#define FRAME_W     160
#define FRAME_H     120
#define FRAME_SIZE  (FRAME_W * FRAME_H * 2)
DMA_HandleTypeDef hdma_dcmi;
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
}

/* -----------------------------------------------------------
 *  Main
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

    uprintf("\r\n--- OV5640 + USB CDC Stream Start ---\r\n");

    /* 1) Enable MCO clock */
    MCO1_24MHz_Enable();
    HAL_Delay(50);

    /* 2) OV5640 Power On / Reset */
    OV5640_PowerUp();
    OV5640_Reset();
    HAL_Delay(100);

    /* 3) OV5640 Init for RGB565 QQVGA(160x120) */
    // RGB565 + QVGA 기본 설정
    if (OV5640_Init_RGB565_QQVGA() != 0)
    {
        uprintf("OV5640 init failed!\r\n");
    }
    else
    {
        uprintf("OV5640 init OK\r\n");
    }

    uprintf("OV5640 Ready.\r\n");

    /* 4) Start DCMI DMA continuous capture */
    if (HAL_DCMI_Start_DMA(&hdcmi,
                           DCMI_MODE_CONTINUOUS,
                           (uint32_t)frame_buf,
                           FRAME_SIZE/4) != HAL_OK)
    {
        uprintf("DCMI DMA start failed!\r\n");
        while(1);
    }

    uprintf("DCMI DMA capture started.\r\n");

    /* -----------------------------------------------------------
     *  Main loop
     * ----------------------------------------------------------- */
    while (1)
    {

            frame_captured = 0;

            uint8_t header[4] = {0xAA, 0x55, FRAME_W, FRAME_H};
            CDC_Transmit_FS(header, 4);
            CDC_Transmit_FS(frame_buf, FRAME_SIZE);


    }
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
