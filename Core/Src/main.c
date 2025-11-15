/* ==========================================
   main.c - 최종 완성 버전
   ========================================== */

#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "ov5640.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* extern 선언 */
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

/* 다른 파일에 정의된 함수들 */
extern void SystemClock_Config(void);

/* 프레임 버퍼 크기 정의 (main.h에 없으므로 여기서 정의) */
#define FRAME_W 320
#define FRAME_H 240
#define BYTES_PER_PIXEL 2
#define FRAME_SIZE (FRAME_W * FRAME_H * BYTES_PER_PIXEL)

/* 버퍼 정의 (main.h의 JPEG_BUF_SIZE 사용) */
uint8_t frame_buf[FRAME_SIZE] __attribute__((section(".ram_d1")));
uint8_t jpeg_buf[JPEG_BUF_SIZE] __attribute__((section(".ram_d1")));
uint32_t jpeg_size = 0;

volatile uint8_t dma_done = 0;
volatile uint8_t frm_done = 0;
volatile uint32_t dcmi_errs = 0;

/* 로컬 함수 선언 */
void Error_Handler(void);
void MPU_Config(void);

static void Common_Init(void);
static void Camera_Mode_Init(void);
static HAL_StatusTypeDef capture_and_send_once(void);
static void send_frame_to_pc(uint32_t size);

/* 디버깅 함수 */
static void test_uart(void);
static void test_i2c_scan(void);
static void test_gpio(void);
static void test_clocks(void);
extern void uprintf(const char *fmt, ...);
/* ==========================================
   main 함수
   ========================================== */
int main(void)
{
    Common_Init();

    uprintf("\r\n");
    uprintf("====================================\r\n");
    uprintf("   STM32H753 + OV5640 Debug\r\n");
    uprintf("====================================\r\n");

    test_uart();
    HAL_Delay(500);

    test_gpio();
    HAL_Delay(500);

    test_clocks();
    HAL_Delay(500);

    Camera_Mode_Init();
    test_i2c_scan();
    HAL_Delay(500);

    OV5640_PowerUp();

    if (OV5640_InitRAW(&hi2c1) != HAL_OK) {
        uprintf("\r\n!!! OV5640 init FAIL !!!\r\n");
        while(1) {
            HAL_Delay(1000);
        }
    }

    uprintf("\r\n=== OV5640 init OK ===\r\n");
    uprintf("====================================\r\n");
    uprintf("   Starting Camera Loop\r\n");
    uprintf("====================================\r\n\r\n");

    uint32_t frame_num = 0;

    while (1) {
        uprintf("\r\n--- Frame #%lu ---\r\n", ++frame_num);

        HAL_StatusTypeDef st = capture_and_send_once();

        if (st != HAL_OK) {
            uprintf("Capture FAIL: %d\r\n", st);
            HAL_Delay(1000);
        } else {
            uprintf("Capture OK\r\n");
        }

        HAL_Delay(100);
    }
}

/* ==========================================
   Common_Init
   ========================================== */
static void Common_Init(void)
{
    MPU_Config();
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART3_UART_Init();
}

/* ==========================================
   Camera_Mode_Init
   ========================================== */
static void Camera_Mode_Init(void)
{
    MX_I2C1_Init();
    MX_DMA_Init();
    MX_DCMI_Init();
}

/* ==========================================
   capture_and_send_once
   ========================================== */
static HAL_StatusTypeDef capture_and_send_once(void)
{
    dma_done = 0;
    frm_done = 0;

    if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                           (uint32_t)frame_buf, FRAME_SIZE / 4) != HAL_OK)
    {
        uprintf("DCMI Start FAIL\r\n");
        return HAL_ERROR;
    }

    uprintf("DCMI Started, waiting...\r\n");

    uint32_t t0 = HAL_GetTick();
    while (!dma_done && (HAL_GetTick() - t0) < 10000)
    {
        // 대기
    }

    HAL_DCMI_Stop(&hdcmi);

    if (!dma_done) {
        uprintf("DCMI TIMEOUT!\r\n");
        return HAL_TIMEOUT;
    }

    uprintf("DMA Complete. frm=%d dma=%d\r\n", frm_done, dma_done);

    SCB_InvalidateDCache_by_Addr((uint32_t*)frame_buf, FRAME_SIZE);

    send_frame_to_pc(FRAME_SIZE);

    return HAL_OK;
}

/* ==========================================
   send_frame_to_pc
   ========================================== */
static void send_frame_to_pc(uint32_t size)
{
    uint8_t header[8];
    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = 0xAA;
    header[3] = 0x55;

    header[4] = (size >>  0) & 0xFF;
    header[5] = (size >>  8) & 0xFF;
    header[6] = (size >> 16) & 0xFF;
    header[7] = (size >> 24) & 0xFF;

    HAL_UART_Transmit(&huart3, header, 8, 1000);

    uint32_t chunk_size = 8192;
    for (uint32_t offset = 0; offset < size; offset += chunk_size)
    {
        uint32_t len = (size - offset) > chunk_size ? chunk_size : (size - offset);
        HAL_UART_Transmit(&huart3, &frame_buf[offset], len, 5000);
    }

    uprintf("Frame sent: %lu bytes\r\n", size);
}

/* ==========================================
   디버깅 함수들
   ========================================== */
static void test_uart(void)
{
    uprintf("\r\n=== UART Test ===\r\n");

    for(int i = 0; i < 5; i++) {
        uprintf("Test message %d\r\n", i);
        HAL_Delay(100);
    }

    uprintf("UART OK\r\n");
}

static void test_i2c_scan(void)
{
    uprintf("\r\n=== I2C Bus Scan ===\r\n");

    uint8_t found = 0;

    for(uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 100) == HAL_OK) {
            uprintf("Found device at 0x%02X\r\n", addr);
            found++;
        }
    }

    if (found == 0) {
        uprintf("No I2C devices found!\r\n");
    } else {
        uprintf("Found %d device(s)\r\n", found);
    }
}

static void test_gpio(void)
{
    uprintf("\r\n=== GPIO Status ===\r\n");

    GPIO_PinState pwdn = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);
    GPIO_PinState rst = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);

    uprintf("PWDN (PD0): %s\r\n", pwdn ? "HIGH" : "LOW");
    uprintf("RST (PD1): %s\r\n", rst ? "HIGH" : "LOW");
}

static void test_clocks(void)
{
    uprintf("\r\n=== System Clocks ===\r\n");

    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();

    uprintf("SYSCLK: %lu Hz\r\n", sysclk);
    uprintf("HCLK: %lu Hz\r\n", hclk);

    uint32_t cfgr = RCC->CFGR;
    uint32_t mco1_src = (cfgr >> 21) & 0x7;
    uint32_t mco1_div = (cfgr >> 24) & 0x7;

    uprintf("MCO1 Source: %lu, Divider: %lu\r\n", mco1_src, mco1_div);
}

/* ==========================================
   uprintf 구현 (main.h에 선언됨)
   ========================================== */
//void uprintf(const char *fmt, ...)
//{
//    char buf[256];
//    va_list ap;
//
//    va_start(ap, fmt);
//    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
//    va_end(ap);
//
//    if (n > 0 && n < (int)sizeof(buf)) {
//        HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
//    }
//}

/* ==========================================
   콜백 함수들
   ========================================== */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *h)
{
    frm_done = 1;
    uprintf("[CB] DCMI Frame Event\r\n");
}

void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_dcmi) {
        dma_done = 1;
        uprintf("[CB] DMA Complete\r\n");
    }
}

void HAL_DMA_XferHalfCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_dcmi) {
        uprintf("[CB] DMA Half\r\n");
    }
}

void HAL_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_dcmi) {
        uprintf("[CB] DMA Error!\r\n");
    }
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *h)
{
    static uint32_t vsync_count = 0;
    vsync_count++;

    if (vsync_count % 10 == 0) {
        uprintf("[CB] VSYNC #%lu\r\n", vsync_count);
    }
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *h)
{
    uprintf("[CB] DCMI Error! SR=0x%08lX\r\n", DCMI->SR);
    dcmi_errs++;
}

/* ==========================================
   MPU_Config
   ========================================== */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x24000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
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

/* ==========================================
   Error Handler
   ========================================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
