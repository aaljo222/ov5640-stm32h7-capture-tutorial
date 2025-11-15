#include "dcmi.h"

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

void MX_DCMI_Init(void)
{
    hdcmi.Instance = DCMI;
    hdcmi.Init.SynchroMode        = DCMI_SYNCHRO_HARDWARE;
    hdcmi.Init.PCKPolarity        = DCMI_PCKPOLARITY_FALLING;  // ★ FALLING
    hdcmi.Init.VSPolarity         = DCMI_VSPOLARITY_LOW;       // ★ LOW
    hdcmi.Init.HSPolarity         = DCMI_HSPOLARITY_HIGH;      // ★ HIGH
    hdcmi.Init.CaptureRate        = DCMI_CR_ALL_FRAME;
    hdcmi.Init.ExtendedDataMode   = DCMI_EXTEND_DATA_8B;
    hdcmi.Init.JPEGMode           = DCMI_JPEG_DISABLE;         // ★ DISABLE
    hdcmi.Init.ByteSelectMode     = DCMI_BSM_ALL;
    hdcmi.Init.ByteSelectStart    = DCMI_OEBS_ODD;
    hdcmi.Init.LineSelectMode     = DCMI_LSM_ALL;
    hdcmi.Init.LineSelectStart    = DCMI_OELS_ODD;

    if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_DCMI_MspInit(DCMI_HandleTypeDef* dcmiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(dcmiHandle->Instance == DCMI)
    {
        __HAL_RCC_DCMI_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /* DCMI GPIO 설정 */
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;

        /* HSYNC(PA4), PCLK(PA6) */
        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* VSYNC(PB7) */
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* D0~D3 (PC6~9) */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* D5(PD3) */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* D7(PE1), D4(PE4), D6(PE6) ★ PE1이 D7! */
        GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* DMA 초기화 */
        hdma_dcmi.Instance                 = DMA1_Stream0;
        hdma_dcmi.Init.Request             = DMA_REQUEST_DCMI;
        hdma_dcmi.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_dcmi.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_dcmi.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_dcmi.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hdma_dcmi.Init.Mode                = DMA_NORMAL;
        hdma_dcmi.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        hdma_dcmi.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

        if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(dcmiHandle, DMA_Handle, hdma_dcmi);

        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

        HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DCMI_IRQn);
    }
}

void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* dcmiHandle)
{
    if(dcmiHandle->Instance == DCMI)
    {
        __HAL_RCC_DCMI_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_6);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_3);
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6);

        HAL_DMA_DeInit(dcmiHandle->DMA_Handle);
    }
}
