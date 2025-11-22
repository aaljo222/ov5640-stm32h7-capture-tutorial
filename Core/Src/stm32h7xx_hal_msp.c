/**
  ******************************************************************************
  * @file    stm32h7xx_hal_msp.c
  * @brief   HAL MSP module - DCMI DMA 설정 포함
  ******************************************************************************
  */

#include "main.h"

extern DMA_HandleTypeDef hdma_dcmi;

/**
  * @brief I2C MSP Initialization
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**I2C1 GPIO Configuration
        PB8     ------> I2C1_SCL
        PB9     ------> I2C1_SDA
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

/**
  * @brief I2C MSP De-Initialization
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
    if(hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
    }
}

/**
  * @brief DCMI MSP Initialization
  */
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hdcmi->Instance == DCMI)
    {
        /* Peripheral clock enable */
        __HAL_RCC_DCMI_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /**DCMI GPIO Configuration
        PA4  ------> DCMI_HSYNC
        PA6  ------> DCMI_PIXCLK
        PB7  ------> DCMI_VSYNC
        PC6  ------> DCMI_D0
        PC7  ------> DCMI_D1
        PC8  ------> DCMI_D2
        PC9  ------> DCMI_D3
        PE4  ------> DCMI_D4
        PD3  ------> DCMI_D5
        PE5  ------> DCMI_D6
        PE6  ------> DCMI_D7
        */
        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* DCMI DMA Init */
        hdma_dcmi.Instance = DMA2_Stream1;
        hdma_dcmi.Init.Request = DMA_REQUEST_DCMI;
        hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
        hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_dcmi.Init.Mode = DMA_NORMAL;
        hdma_dcmi.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
        hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_dcmi.Init.MemBurst = DMA_MBURST_INC4;
        hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;

        if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(hdcmi, DMA_Handle, hdma_dcmi);

        /* DCMI interrupt Init */
        HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DCMI_IRQn);
    }
}

/**
  * @brief DCMI MSP De-Initialization
  */
void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{
    if(hdcmi->Instance == DCMI)
    {
        __HAL_RCC_DCMI_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_6);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_3);
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

        HAL_DMA_DeInit(hdcmi->DMA_Handle);

        HAL_NVIC_DisableIRQ(DCMI_IRQn);
    }
}

/**
  * @brief USART MSP Initialization
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(huart->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /**USART3 GPIO Configuration
        PD8  ------> USART3_TX
        PD9  ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

/**
  * @brief USART MSP De-Initialization
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9);
    }
}
