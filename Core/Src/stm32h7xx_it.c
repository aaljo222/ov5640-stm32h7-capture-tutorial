#include "main.h"

extern DMA_HandleTypeDef  hdma_dcmi;
extern DCMI_HandleTypeDef hdcmi;

/* Cortex-M7 Handlers */

void NMI_Handler(void)              { }
void HardFault_Handler(void)        { while (1) {} }
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

/* DMA1 Stream0: DCMI */

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_dcmi);
}

/* DCMI global IRQ (startup.s 에 DCMI_IRQHandler로 연결되어 있음) */

void DCMI_IRQHandler(void)
{
    HAL_DCMI_IRQHandler(&hdcmi);
}
