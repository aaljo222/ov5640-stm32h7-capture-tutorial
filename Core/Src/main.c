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
 *  Debug print (USART3)
 * ----------------------------------------------------------- */
void uprintf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len > 0)
        HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

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
        if (frame_captured)
        {
            frame_captured = 0;

            /* USB CDC transmit */
            uint8_t status = CDC_Transmit_FS(frame_buf, FRAME_SIZE);

            if (status == USBD_OK)
                uprintf("Sent frame: %d bytes\r\n", FRAME_SIZE);
            else
                uprintf("USB busy...\r\n");
        }
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */



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
