/* ========================================
   DMA 설정 - 최종 정리 버전
   - DMA1_Stream0 사용
   - DCMI 전용
   ======================================== */

#include "dma.h"

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

void MX_DMA_Init(void)
{
    /* 참고: DMA 초기화는 HAL_DCMI_MspInit()에서 수행됨 */
    /* 이 함수는 호출되지만 실제 초기화는 dcmi.c에서 처리 */
}
