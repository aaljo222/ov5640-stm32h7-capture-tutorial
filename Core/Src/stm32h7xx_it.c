#include "main.h"

extern DMA_HandleTypeDef  hdma_dcmi;
extern DCMI_HandleTypeDef hdcmi;

/* Cortex-M7 Handlers */

void NMI_Handler(void)              { }
void HardFault_Handler(void)        {
	  while (1) { }
}
void MemManage_Handler(void)        { while (1) {} }
void BusFault_Handler(void)         { while (1) {} }
void UsageFault_Handler(void)       { while (1) {} }
void SVC_Handler(void)              { }
void DebugMon_Handler(void)         { }
void PendSV_Handler(void)           { }

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ==================== Interrupt Handlers ==================== */
// ★ CRITICAL: These must be in stm32h7xx_it.c or here!

void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_dcmi);
}

void DCMI_IRQHandler(void)
{
  HAL_DCMI_IRQHandler(&hdcmi);
}
