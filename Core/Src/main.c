/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - FIXED
  * @details        : OV5640 Camera with UART Debug + USB CDC Image
  ******************************************************************************
  */

#include "main.h"
#include "ov5640.h"
#include "usb_cdc_handler.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Peripheral handles */
I2C_HandleTypeDef hi2c1;
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Buffers */
#define IMAGE_WIDTH  320
#define IMAGE_HEIGHT 240
#define IMAGE_SIZE   (IMAGE_WIDTH * IMAGE_HEIGHT * 2)  // RGB565

// ⭐ SRAM으로 변경 (SDRAM 초기화 복잡)
uint32_t camera_buffer[IMAGE_SIZE / 4] __attribute__((aligned(32)));
volatile uint8_t frame_ready = 0;

/* OV5640 Configuration */
OV5640_Config_t ov5640_config = {
    .hi2c = &hi2c1,
    .pwdn_port = OV5640_PWDN_GPIO_Port,
    .pwdn_pin = OV5640_PWDN_Pin,
    .rst_port = OV5640_RST_GPIO_Port,
    .rst_pin = OV5640_RST_Pin,
    .image_format = OV5640_FORMAT_RGB565,
    .resolution = OV5640_RES_QVGA
};

/* Function prototypes */
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MCO1_Init(void);
void uprintf(const char *fmt, ...);

int main(void)
{
    /* MPU & Cache */
    MPU_Config();

    /* HAL Init */
    HAL_Init();
    SystemClock_Config();

    /* Peripheral Init */
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_DCMI_Init();
    MCO1_Init();

    /* USB Init */
    MX_USB_DEVICE_Init();
    USB_CDC_Init();

    HAL_Delay(1000); // USB enumeration wait

    uprintf("\r\n========================================\r\n");
    uprintf("  OV5640 Camera System\r\n");
    uprintf("========================================\r\n");
    uprintf("UART Debug: PD8 (115200)\r\n");
    uprintf("USB CDC: Image data\r\n");
    uprintf("========================================\r\n\r\n");

    /* OV5640 Init */
    uprintf("Initializing OV5640...\r\n");
    if (OV5640_Init(&ov5640_config) != HAL_OK) {
        uprintf("[ERROR] OV5640 Init Failed!\r\n");
        Error_Handler();
    }
    uprintf("[OK] OV5640 Ready\r\n\r\n");

    /* Start Capture */
    uprintf("Starting DCMI capture...\r\n");
    if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                           (uint32_t)camera_buffer,
                           IMAGE_SIZE / 4) != HAL_OK) {
        uprintf("[ERROR] DCMI Start Failed!\r\n");
        Error_Handler();
    }

    uint32_t frame_count = 0;

    while (1)
    {
        if (frame_ready)
        {
            frame_ready = 0;
            frame_count++;

            uprintf("[Frame #%lu] Captured\r\n", frame_count);

            /* Send via USB CDC */
            HAL_StatusTypeDef status = USB_CDC_TransmitImage(
                (uint8_t*)camera_buffer, IMAGE_SIZE);

            if (status == HAL_OK) {
                uprintf("[USB] Frame sent OK\r\n");
            } else {
                uprintf("[USB] Frame send FAILED\r\n");
            }

            /* Restart capture */
            HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                               (uint32_t)camera_buffer,
                               IMAGE_SIZE / 4);
        }

        HAL_Delay(10);
    }
}

/**
  * @brief  UART printf
  */
void uprintf(const char *fmt, ...)
{
    char msg[256];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (len > 0 && len < 256) {
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, HAL_MAX_DELAY);
    }
}

/**
  * @brief  USART3 Init
  */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  DCMI Frame Event Callback
  */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    frame_ready = 1;
}

/**
  * @brief  Configure MCO1 to output 24MHz
  */
static void MCO1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* PA8 = MCO1 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* ⭐ HSE=8MHz, PLL=480MHz이므로 PLL/20 = 24MHz */
    /* 또는 HSI48/2 = 24MHz */
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_2);
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    
    __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 120;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                  |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x307075B1;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }
    
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief DCMI Initialization
  */
static void MX_DCMI_Init(void)
{
    hdcmi.Instance = DCMI;
    hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
    hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
    hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
    hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
    hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
    hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
    hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
    hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
    hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
    hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;

    if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
    {
        Error_Handler();
    }

    /* ⭐ DCMI 인터럽트 활성화 */
    HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
}

/**
  * @brief DMA Initialization
  */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    
    /* OV5640_PWDN_Pin (PD0), OV5640_RST_Pin (PD1) */
    GPIO_InitStruct.Pin = OV5640_PWDN_Pin|OV5640_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    /* Default: Power ON, Reset inactive */
    HAL_GPIO_WritePin(OV5640_PWDN_GPIO_Port, OV5640_PWDN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OV5640_RST_GPIO_Port, OV5640_RST_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Configure MPU
  */
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};
    
    HAL_MPU_Disable();
    
    /* SRAM (0x20000000 ~ 0x2001FFFF: 128KB) */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x20000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  Error Handler
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Blink LED or log error
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation to report error */
}
#endif
