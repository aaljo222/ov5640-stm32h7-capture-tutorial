#include "main.h"

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef  hdma_dcmi;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef  hi2c1;

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    //__HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

/* ============ DCMI MSP ============ */

void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hdcmi->Instance == DCMI)
    {
        /* 1) 클럭 */
        __HAL_RCC_DCMI_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /* 2) GPIO (핀맵 그림 기준) */
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;

        // HSYNC: PA4
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // PCLK: PA6
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // VSYNC: PB7
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // D0~D3: PC6,7,8,9
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        // D4: PE4
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        // D5: PD3
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        // D6: PE6, D7: PE1
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* 3) DMA1_Stream0 */
        hdma_dcmi.Instance                 = DMA1_Stream0;
        hdma_dcmi.Init.Request             = DMA_REQUEST_DCMI;
        hdma_dcmi.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_dcmi.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_dcmi.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_dcmi.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hdma_dcmi.Init.Mode                = DMA_NORMAL;      // 스냅샷
        hdma_dcmi.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_dcmi.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

        if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
            Error_Handler();

        __HAL_LINKDMA(hdcmi, DMA_Handle, hdma_dcmi);

        /* 4) NVIC */
        HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DCMI_IRQn);

        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    }
}

void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{
    if (hdcmi->Instance == DCMI)
    {
        __HAL_RCC_DCMI_CLK_DISABLE();
        HAL_DMA_DeInit(hdcmi->DMA_Handle);
        HAL_NVIC_DisableIRQ(DCMI_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    }
}

/* ============ UART3 MSP ============ */

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

/* ============ I2C1 MSP ============ */

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}
