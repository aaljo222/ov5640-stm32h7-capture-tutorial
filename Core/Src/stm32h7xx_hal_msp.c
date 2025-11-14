#include "main.h"
#include "stm32h7xx_hal.h"        // ★ 추가 (모든 HAL 헤더 포함)
#include "stm32h7xx_hal_rcc.h"    // ★ 추가 (RCC 매크로들)
#include "stm32h7xx_hal_pwr_ex.h" // ★ 추가 (PWR 확장)


extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef  hdma_dcmi;
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart3;

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
//  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_DCMI_MspInit(DCMI_HandleTypeDef* h)
{
  if (h->Instance==DCMI)
  {
    __HAL_RCC_DCMI_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    // DMA1 Stream0 CH1 (예시: 데이터폭 8bit 메모리)
    hdma_dcmi.Instance = DMA1_Stream0;
    hdma_dcmi.Init.Request = DMA_REQUEST_DCMI;
    hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_dcmi.Init.Mode = DMA_NORMAL;
    hdma_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_dcmi.Init.MemBurst   = DMA_MBURST_SINGLE;
    hdma_dcmi.Init.PeriphBurst= DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(h, DMA_Handle, hdma_dcmi);

    HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* h)
{
  if (h->Instance == I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;  // PB8=SCL, PB9=SDA
    g.Mode      = GPIO_MODE_AF_OD;          // ★ Open-Drain
    g.Pull      = GPIO_PULLUP;              // ★ Pull-up
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;            // ★ AF4
    HAL_GPIO_Init(GPIOB, &g);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* h)
{
  if (h->Instance == USART3)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   // PD8=TX, PD9=RX
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &g);
  }
}

