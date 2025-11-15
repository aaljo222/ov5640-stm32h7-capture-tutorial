/* ========================================
   인터럽트 핸들러 - 최종 버전
   ======================================== */

#include "main.h"
#include "stm32h7xx_it.h"

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

/* 전역 변수 선언 (main.c에서 사용) */
extern volatile uint8_t dma_done;
extern volatile uint8_t frm_done;

/* DMA1 Stream0 인터럽트 핸들러 */
void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_dcmi);
}

/* DCMI 인터럽트 핸들러 */
void DCMI_IRQHandler(void)
{
    HAL_DCMI_IRQHandler(&hdcmi);
}

/* ========================================
   주의: 콜백 함수들은 main.c에 구현됨!

   main.c에 있는 콜백들:
   - HAL_DCMI_FrameEventCallback
   - HAL_DMA_XferCpltCallback
   - HAL_DMA_XferHalfCpltCallback
   - HAL_DMA_XferErrorCallback
   - HAL_DCMI_VsyncEventCallback
   - HAL_DCMI_ErrorCallback
   ======================================== */
