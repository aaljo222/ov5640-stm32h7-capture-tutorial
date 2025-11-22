/**
  ******************************************************************************
  * @file    main.c
  * @brief   OV5640 Camera with DCMI Capture and USB CDC Transfer
  ******************************************************************************
  */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "ov5640.h"
#include <string.h>
#include <stdio.h>
#include "logic_analyser.h"  // ← 추가!

/* External variables */
extern I2C_HandleTypeDef hi2c1;

/* Private variables */
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
UART_HandleTypeDef huart3;

/* Image buffer - 160x120 RGB565 */
#define IMG_WIDTH   160
#define IMG_HEIGHT  120
#define IMG_SIZE    (IMG_WIDTH * IMG_HEIGHT * 2)

// Frame buffer in SRAM
uint8_t frame_buffer[IMG_SIZE] __attribute__((section(".sram")));
volatile uint8_t frame_ready = 0;

/* Function prototypes */
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_USART3_UART_Init(void);
void MX_I2C1_Init(void);  // i2c.c에서 정의

/* User functions */
static void Debug_Print(const char* msg);
static void Camera_PowerCycle(void);
static void Capture_Frame(void);
static void Send_Frame_USB(void);

/**
  * @brief  Debug output via UART and USB CDC
  */
static void Debug_Print(const char* msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(10);
}

/**
  * @brief  Camera power cycle
  */
static void Camera_PowerCycle(void)
{
    Debug_Print("\r\n=== Camera Power Cycle ===\r\n");

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    Debug_Print("Power OFF, Reset LOW\r\n");
    HAL_Delay(100);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    Debug_Print("Power ON\r\n");
    HAL_Delay(20);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    Debug_Print("Reset released\r\n");
    HAL_Delay(50);

    Debug_Print("=== Power Cycle Done ===\r\n\r\n");
}

/**
  * @brief  DCMI Frame capture
  */
static void Capture_Frame(void)
{
    frame_ready = 0;

    // Start DMA capture
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                       (uint32_t)frame_buffer, IMG_SIZE / 4);
}

/**
  * @brief  Send frame via USB CDC
  */
static void Send_Frame_USB(void)
{
    char msg[64];

    Debug_Print("\r\n=== Sending Frame ===\r\n");

    // Header
    sprintf(msg, "FRAME:%d:%d:%d\r\n", IMG_WIDTH, IMG_HEIGHT, IMG_SIZE);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(50);

    // Send image data in chunks
    const uint16_t chunk_size = 512;
    uint32_t sent = 0;

    while(sent < IMG_SIZE)
    {
        uint16_t to_send = (IMG_SIZE - sent) > chunk_size ?
                           chunk_size : (IMG_SIZE - sent);

        uint8_t result = CDC_Transmit_FS(&frame_buffer[sent], to_send);

        if(result == USBD_OK)
        {
            sent += to_send;

            // Progress
            if((sent * 10 / IMG_SIZE) > ((sent - to_send) * 10 / IMG_SIZE))
            {
                sprintf(msg, "  %lu%%\r\n", sent * 100 / IMG_SIZE);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
            }

            HAL_Delay(10);
        }
        else
        {
            HAL_Delay(20);
        }
    }

    // End marker
    CDC_Transmit_FS((uint8_t*)"END\r\n", 5);
    HAL_Delay(50);

    sprintf(msg, "Transfer complete: %lu bytes\r\n", sent);
    Debug_Print(msg);
}

/**
  * @brief  DCMI VSYNC 콜백
  */
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    Debug_Print(">>> VSYNC! <<<\r\n");
}

/**
  * @brief  DCMI Line 콜백
  */
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    static uint32_t line_count = 0;
    line_count++;

    if(line_count % 20 == 0)
    {
        char msg[32];
        sprintf(msg, "L:%lu ", line_count);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
    }
}

/**
  * @brief  DCMI Frame complete callback
  */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    frame_ready = 1;
    HAL_DCMI_Stop(hdcmi);
    Debug_Print("\r\n>>> FRAME COMPLETE! <<<\r\n");
}

/**
  * @brief  DCMI Error callback
  */
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
    char msg[64];
    sprintf(msg, "DCMI Error! Code: 0x%08lX\r\n", hdcmi->ErrorCode);
    Debug_Print(msg);
    frame_ready = 0;
}

/**
  * @brief  Main program
  */
int main(void)
{
    /* MPU Configuration */
    MPU_Config();

    /* HAL Initialization */
    HAL_Init();

    /* System Clock Configuration */
    SystemClock_Config();
    PeriphCommonClock_Config();

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_DCMI_Init();
    MX_I2C1_Init();
    MX_USART3_UART_Init();
    MX_USB_DEVICE_Init();

    /* Wait for USB CDC ready */
    HAL_Delay(1000);

    Debug_Print("\r\n\r\n");
    Debug_Print("=========================================\r\n");
    Debug_Print("OV5640 Camera with Image Capture\r\n");
    Debug_Print("=========================================\r\n\r\n");

    /* Camera power cycle */
    Camera_PowerCycle();

    /* OV5640 Driver Test */
    OV5640_Object_t camera;
    char msg[128];

    Debug_Print("Probing OV5640...\r\n");
    int32_t result = OV5640_Probe(&camera);

    if(result == 0)
    {
        Debug_Print(">>> OV5640 FOUND! <<<\r\n\r\n");

        uint32_t chip_id = 0;
        OV5640_ReadID(&camera, &chip_id);
        sprintf(msg, "Chip ID: 0x%04lX\r\n\r\n", chip_id);
        Debug_Print(msg);

        Debug_Print("Initializing OV5640...\r\n");
        result = OV5640_Init(&camera, MODE_RGB565, RESOLUTION_R160x120);

        if(result == 0)
        {
            Debug_Print(">>> OV5640 Initialized! <<<\r\n");
            sprintf(msg, "Mode: RGB565, Resolution: %dx%d\r\n\r\n",
                    IMG_WIDTH, IMG_HEIGHT);
            Debug_Print(msg);

            // ★★★ 스트리밍 시작 - 중요! ★★★
            Debug_Print("Starting streaming...\r\n");
            OV5640_Start(&camera, MODE_RGB565);
            Debug_Print(">>> Streaming started! <<<\r\n\r\n");

            // ★★★ Logic Analyzer 실행 ★★★
            Debug_Print("Running Logic Analyzer...\r\n");
            HAL_Delay(200);  // OV5640 안정화 대기
            logic_analyzer_run();
            Debug_Print("Logic Analyzer complete!\r\n\r\n");
        }
        else
        {
            Debug_Print("OV5640 Init FAILED\r\n\r\n");
            while(1);
        }
    }
    else
    {
        Debug_Print("ERROR: Cannot detect OV5640\r\n");
        while(1);
    }

    Debug_Print("=========================================\r\n");
    Debug_Print("Starting auto-capture (5 sec interval)\r\n");
    Debug_Print("=========================================\r\n\r\n");

    /* Infinite loop with image capture */
    uint32_t count = 0;

    while (1)
    {
        HAL_Delay(5000);

        count++;
        sprintf(msg, "\r\n[Frame #%lu] Capturing...\r\n", count);
        Debug_Print(msg);

        // Start capture
        Capture_Frame();

        // Wait for completion (timeout 1 sec)
        uint32_t timeout = HAL_GetTick() + 1000;
        while(!frame_ready && HAL_GetTick() < timeout)
        {
            HAL_Delay(10);
        }

        if(frame_ready)
        {
            Debug_Print("Capture complete!\r\n");
            Send_Frame_USB();
        }
        else
        {
            Debug_Print("Capture timeout!\r\n");
            HAL_DCMI_Stop(&hdcmi);
        }

        // Quick status check
        uint32_t id = 0;
        if(OV5640_ReadID(&camera, &id) == 0)
        {
            sprintf(msg, "Camera status: OK (ID: 0x%04lX)\r\n", id);
            Debug_Print(msg);
        }
        else
        {
            Debug_Print("Camera status: ERROR\r\n");
        }
    }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

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
        Error_Handler();

    // MCO1: 16MHz for OV5640 XCLK
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_4);
}

/**
  * @brief Peripherals Common Clock Configuration
  */
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.PLL3.PLL3M = 32;
    PeriphClkInitStruct.PLL3.PLL3N = 48;
    PeriphClkInitStruct.PLL3.PLL3P = 2;
    PeriphClkInitStruct.PLL3.PLL3Q = 2;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        Error_Handler();
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
        Error_Handler();
}

/**
  * @brief USART3 Initialization
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
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart3) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
        Error_Handler();
}

/**
  * @brief DMA Initialization
  */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Camera control pins */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* PA8 for MCO1 - VERY HIGH SPEED */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief MPU Configuration
  */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

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
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
