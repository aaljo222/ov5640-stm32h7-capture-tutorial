/* USER CODE BEGIN Header */
/**
  * OV5640 + DCMI(8-bit) + XCLK(24MHz) + UART log
  * MODE:
  *   - FW_MODE_ANALYZER : Logic analyzer for DVP signals (PCLK, HSYNC, VSYNC, D0-D7)
  *   - FW_MODE_CAMERA   : OV5640 + DCMI + DMA frame capture with UART transmission
  */
/* USER CODE END Header */

#include "main.h"
#include "logic_analyser.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ==================== Configuration ==================== */
#define FW_MODE_ANALYZER   0
#define FW_MODE_CAMERA     1

#ifndef FW_MODE
  #define FW_MODE   FW_MODE_CAMERA
#endif

#define STREAM_TO_PC   1   // 1: Send frame via UART, 0: Capture to memory only

/* ==================== Hardware Definitions ==================== */
#define OV5640_ADDR_7B   0x3C
#define OV5640_HAL_ADDR  (OV5640_ADDR_7B << 1)

#define FRAME_W           320  // 160 → 320
#define FRAME_H           240  // 120 → 240
#define BYTES_PER_PIXEL   2
#define FRAME_SIZE        (FRAME_W * FRAME_H * BYTES_PER_PIXEL)

/* ==================== Peripheral Handles ==================== */
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef  hdma_dcmi;
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart3;

/* ==================== Frame Buffer (D1 SRAM, 32B aligned) ==================== */
#if defined(__GNUC__)
__attribute__((section(".ram_d1"))) __attribute__((aligned(32)))
#endif
uint8_t frame_buf[FRAME_SIZE];

/* ==================== CAMERA Mode Flags ==================== */
volatile uint8_t  dma_done  = 0;
volatile uint8_t  frm_done  = 0;
volatile uint32_t dcmi_errs = 0;

/* ==================== Function Prototypes ==================== */
// System
static void MPU_Config(void);
void SystemClock_Config(void);
void Error_Handler(void);
void uprintf(const char *fmt, ...);

// Initialization
static void Common_Init(void);
static void Camera_Mode_Init(void);
static void Analyzer_Mode_Init(void);

// GPIO
static void MX_GPIO_Init(void);
static void MX_GPIO_DCMI_AF_Init(void);
static void MX_GPIO_DCMI_INPUT_Init(void);
static void MX_MCO_Init(void);

// Peripherals
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);

// OV5640
static void OV5640_PowerUp(void);
static HAL_StatusTypeDef OV5640_Init_RAW_QVGA(void);
static HAL_StatusTypeDef wr8(I2C_HandleTypeDef*, uint8_t, uint16_t, uint8_t);
static HAL_StatusTypeDef rd8(I2C_HandleTypeDef*, uint8_t, uint16_t, uint8_t*);

// Camera capture
static void send_frame_to_pc(uint32_t payload_len);
static HAL_StatusTypeDef capture_and_send_once(void);

/* ==================== Main Function ==================== */
int main(void)
{
  Common_Init();

#if (FW_MODE == FW_MODE_CAMERA)
  Camera_Mode_Init();

  while (1) {
    HAL_StatusTypeDef st = capture_and_send_once();
    if (st != HAL_OK) {
      uprintf("Capture failed: %d\r\n", st);
    }
    // HAL_Delay(10);  // ← 이 줄 주석 처리 (더 빠른 프레임레이트)
  }

#elif (FW_MODE == FW_MODE_ANALYZER)
  Analyzer_Mode_Init();

  while (1) {
    logic_analyzer_run();
    HAL_Delay(100);
  }
#endif
}

/* ==================== Initialization Functions ==================== */
static void Common_Init(void)
{
  MPU_Config();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();         // PD0=PWDN, PD1=RST
  MX_MCO_Init();          // PA8 -> 24MHz (HSI48/2)
  MX_USART3_UART_Init();
}

static void Camera_Mode_Init(void)
{
  uprintf("\r\nBOOT (CAMERA MODE)\r\n");

  MX_I2C1_Init();
  MX_GPIO_DCMI_AF_Init();
  MX_DMA_Init();
  MX_DCMI_Init();

  OV5640_PowerUp();

  if (OV5640_Init_RAW_QVGA() != HAL_OK) {
    uprintf("OV5640 init FAIL\r\n");
  } else {
    uprintf("OV5640 init OK\r\n");
  }
}

static void Analyzer_Mode_Init(void)
{
  uprintf("\r\nBOOT (ANALYZER MODE)\r\n");

  MX_GPIO_DCMI_INPUT_Init();  // Input + Pulldown
  MX_I2C1_Init();

  OV5640_PowerUp();

  if (OV5640_Init_RAW_QVGA() != HAL_OK) {
    uprintf("OV5640 init FAIL (I2C error?)\r\n");
  } else {
    uprintf("OV5640 init OK - DVP signals active\r\n");
  }
}

/* ==================== UART Printf ==================== */
void uprintf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;

  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0 || n > (int)sizeof(buf)) {
    n = (n > (int)sizeof(buf)) ? sizeof(buf) : 0;
  }

  if (n > 0) {
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
  }
}

/* ==================== Camera Capture Functions ==================== */
static void send_frame_to_pc(uint32_t payload_len)
{
#if STREAM_TO_PC
  uint8_t hdr[8] = {
    0xAA, 0x55, 0xAA, 0x55,
    (uint8_t)(payload_len),
    (uint8_t)(payload_len >> 8),
    (uint8_t)(payload_len >> 16),
    (uint8_t)(payload_len >> 24)
  };
  HAL_UART_Transmit(&huart3, hdr, sizeof(hdr), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart3, frame_buf, payload_len, HAL_MAX_DELAY);
#else
  (void)payload_len;
#endif
}
static HAL_StatusTypeDef capture_and_send_once(void)
{
  uprintf("capture_and_send_once: start\r\n");

  // Reset flags
  dma_done = 0;
  frm_done = 0;
  dcmi_errs = 0;

  // Clean DCache for DMA target (32B aligned)
  uint32_t addr  = (uint32_t)frame_buf;
  uint32_t start = addr & ~31U;
  uint32_t end   = (addr + FRAME_SIZE + 31U) & ~31U;
  SCB_CleanDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));

  // Enable DCMI interrupts
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_OVR | DCMI_IT_ERR);

  // Start DCMI capture
  HAL_StatusTypeDef st = HAL_DCMI_Start_DMA(
      &hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)frame_buf, FRAME_SIZE/4);

  if (st != HAL_OK) {
    uprintf("DCMI Start FAIL (%d)\r\n", st);
    return st;
  }

  uprintf("DCMI Start OK, waiting for DMA...\r\n");

  // Wait for DMA completion with timeout
  // Wait for DMA completion with timeout
  uint32_t t0 = HAL_GetTick();
  uint32_t last_check = t0;
  while (!dma_done && (HAL_GetTick() - t0) < 10000) {  // 10초 타임아웃
      // Print status every 500ms
      if ((HAL_GetTick() - last_check) >= 500) {
          uint32_t ndtr = DMA1_Stream0->NDTR;
          uprintf("  [%lu ms] Waiting... DCMI_SR=0x%08lX frm=%d dma=%d\r\n",
                  HAL_GetTick() - t0,
                  (unsigned long)DCMI->SR,
                  frm_done, dma_done);
          uprintf("    DMA NDTR=%lu (expected=38400, transferred=%lu bytes)\r\n",
                  ndtr, (38400 - ndtr) * 4);
          last_check = HAL_GetTick();
      }
  }
  HAL_DCMI_Stop(&hdcmi);

  if (!dma_done) {
      // DMA 인터럽트가 안 왔지만 NDTR 확인
      uint32_t final_ndtr = DMA1_Stream0->NDTR;

      uprintf("DMA interrupt not received. Final NDTR=%lu\r\n", final_ndtr);

      if (final_ndtr == 0) {
          // DMA 전송은 완료됨! 강제로 처리
          uprintf("DMA transfer complete (NDTR=0), forcing frame send...\r\n");

          // DCache invalidate (DMA가 쓴 데이터를 CPU가 읽을 수 있도록)
          uint32_t addr  = (uint32_t)frame_buf;
          uint32_t start = addr & ~31U;
          uint32_t end   = (addr + FRAME_SIZE + 31U) & ~31U;
          SCB_InvalidateDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));

          send_frame_to_pc(FRAME_SIZE);
          return HAL_OK;
      }

      // 진짜 타임아웃 (NDTR이 0이 아님)
      uprintf("DCMI TIMEOUT! SR=0x%08lX MISR=0x%08lX ERR=0x%08lX\r\n",
              (unsigned long)DCMI->SR,
              (unsigned long)DCMI->MISR,
              (unsigned long)dcmi_errs);

      uprintf("Pin states - PA4(HSYNC)=%d PA6(PCLK)=%d PB7(VSYNC)=%d\r\n",
              HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4),
              HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6),
              HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));

      return HAL_TIMEOUT;
  }

  uprintf("capture_and_send_once: DMA done, sending frame\r\n");
  send_frame_to_pc(FRAME_SIZE);
  return HAL_OK;
  }


/* ==================== IRQ Callbacks (CAMERA Mode) ==================== */
void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  if (hdma->Instance == DMA1_Stream0) {
    // Invalidate DCache after DMA (32B aligned)
    uint32_t addr  = (uint32_t)frame_buf;
    uint32_t start = addr & ~31U;
    uint32_t end   = (addr + FRAME_SIZE + 31U) & ~31U;
    SCB_InvalidateDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));

    dma_done = 1;
    uprintf("[IRQ] DMA XferCplt\r\n");
  }
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *h)
{
  if (h->Instance == DCMI) {
    frm_done = 1;
    uprintf("[IRQ] DCMI FrameEvent\r\n");
  }
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *h)
{
  if (h->Instance == DCMI) {
    dcmi_errs |= h->ErrorCode;
    uprintf("[IRQ] DCMI Error=0x%08lX, SR=0x%08lX <<<<<<\r\n",  // <<<< 추가
            (unsigned long)h->ErrorCode, (unsigned long)DCMI->SR);
  }
}

/* ==================== OV5640 Control Functions ==================== */
static void OV5640_PowerUp(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); // PWDN=0
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // RST=0
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);   // RST=1
  HAL_Delay(50);
  uprintf("OV5640 PowerUp done\r\n");
}

static HAL_StatusTypeDef wr8(I2C_HandleTypeDef *hi2c, uint8_t hal_addr,
                             uint16_t reg, uint8_t val)
{
  uint8_t pkt[3] = { (uint8_t)(reg>>8), (uint8_t)reg, val };
  return HAL_I2C_Master_Transmit(hi2c, hal_addr, pkt, 3, 100);
}

static HAL_StatusTypeDef rd8(I2C_HandleTypeDef *hi2c, uint8_t hal_addr,
                             uint16_t reg, uint8_t *val)
{
  return HAL_I2C_Mem_Read(hi2c, hal_addr, reg,
                          I2C_MEMADD_SIZE_16BIT, val, 1, 100);
}

static HAL_StatusTypeDef OV5640_Init_RAW_QVGA(void)
{
  uint16_t id = 0;
  uint8_t v = 0;

  // Read and verify chip ID
  if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x300A, &v) != HAL_OK) {
    uprintf("Failed to read 0x300A\r\n");
    return HAL_ERROR;
  }
  id = v << 8;
  if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x300B, &v) != HAL_OK) {
    uprintf("Failed to read 0x300B\r\n");
    return HAL_ERROR;
  }
  id |= v;

  if (id != 0x5640) {
    uprintf("Wrong chip ID: 0x%04X (expected 0x5640)\r\n", id);
    return HAL_ERROR;
  }
  uprintf("OV5640 ID=0x%04X\r\n", id);

  // Software reset
  uprintf("OV5640 software reset...\r\n");
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3008, 0x82);
  HAL_Delay(100);

  // ========== SYSTEM CONTROL ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3103, 0x11); // System clock from PLL, pad output enable
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3008, 0x82); // Software reset
  HAL_Delay(10);

  // ========== CLOCK/PLL SETTINGS ==========
  // Input clock: 24MHz, Target: ~56MHz system clock for QVGA
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3034, 0x18); // MIPI bit mode
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3035, 0x11); // PLL pre-divider (system clock divider)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3036, 0x38); // PLL multiplier (56x)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3037, 0x13); // PLL root divider
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3108, 0x01); // PCLK root divider
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3824, 0x01); // PCLK manual divider

  // ========== AUTO FUNCTIONS ENABLE (CRITICAL FOR REAL CAMERA) ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3503, 0x00); // AE/AGC auto, delay 1 frame
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3c07, 0x08); // 50Hz/60Hz auto detection

  // Auto White Balance
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3406, 0x00); // AWB manual disable (auto mode)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5183, 0x80); // AWB advanced
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5191, 0xff); // AWB top limit
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5192, 0x00); // AWB bottom limit

  // ========== ISP CONTROL (Image Signal Processor) ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5000, 0xa7); // LENC on, BPC on, WPC on
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5001, 0x83); // AWB on, auto BLC

  // Color Matrix (CMX)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5580, 0x06); // Saturation, Contrast, UV adjust enable
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5583, 0x40); // Saturation U
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5584, 0x40); // Saturation V
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5588, 0x01); // Auto UV adjust enable

  // Sharpness/Denoise
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5308, 0x25); // Sharpness
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5302, 0x10); // Denoise threshold

  // Gamma, UV average, color correction, AWB, interpolation, denoise, sharpness, edge enhancement
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5025, 0x00); // Low sum

  // ========== TIMING CONTROL FOR QVGA (320x240) ==========
  // Sensor array settings
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3800, 0x00); // X address start high
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3801, 0x00); // X address start low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3802, 0x00); // Y address start high
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3803, 0x00); // Y address start low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3804, 0x0a); // X address end high (2623)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3805, 0x3f); // X address end low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3806, 0x07); // Y address end high (1951)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3807, 0x9f); // Y address end low

  // Output size (after scaling)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3808, 0x01); // DVP output horizontal width high (320)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3809, 0x40); // DVP output horizontal width low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x380a, 0x00); // DVP output vertical height high (240)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x380b, 0xf0); // DVP output vertical height low

  // Total size (sensor timing)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x380c, 0x07); // Total horizontal size high (1896)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x380d, 0x68); // Total horizontal size low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x380e, 0x03); // Total vertical size high (740)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x380f, 0xd8); // Total vertical size low

  // Offset
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3810, 0x00); // X offset high
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3811, 0x10); // X offset low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3812, 0x00); // Y offset high
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3813, 0x06); // Y offset low

  // Increment/subsampling
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3814, 0x31); // X increment (odd/even pixel skip)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3815, 0x31); // Y increment (odd/even line skip)

  // ========== TIMING TC ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3820, 0x41); // Vertical flip off, binning on
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3821, 0x07); // Horizontal mirror off, binning on

  // ========== 50/60Hz DETECTOR ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a08, 0x01); // B50 step high
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a09, 0x27); // B50 step low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a0a, 0x00); // B60 step high
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a0b, 0xf6); // B60 step low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a0e, 0x03); // Max 60Hz bands
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a0d, 0x04); // Max 50Hz bands

  // ========== AEC/AGC CONTROL ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a02, 0x03); // 60Hz max exposure high
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a03, 0xd8); // 60Hz max exposure low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a14, 0x03); // 50Hz max exposure high
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a15, 0xd8); // 50Hz max exposure low
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a18, 0x00); // Gain ceiling high (4x)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3a19, 0xf8); // Gain ceiling low

  // ========== BLC (Black Level Calibration) ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x4001, 0x02); // BLC start line
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x4004, 0x02); // BLC line number

  // ========== FORMAT CONTROL ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x4300, 0x61); // RGB565 format
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x501F, 0x01); // ISP RGB output enable

  // ========== DVP CONTROL (Digital Video Port) ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x4740, 0x20); // VSYNC active low, HREF/HSYNC active high, PCLK not inverted
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x4730, 0x00); // VSYNC/HSYNC/PCLK delay
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x4407, 0x04); // Quantization range
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x460b, 0x35); // DVP PCLK divider manual
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x460c, 0x22); // DVP CLK divider (2x)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x4837, 0x22); // PCLK period

  // ========== OUTPUT ENABLE ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3017, 0xff); // PAD output enable (all pins)
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3018, 0xff); // PAD output enable (all pins)

  // ========== LENS CORRECTION ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5800, 0x23); // Lens correction coefficients
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5801, 0x14);
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5802, 0x0f);
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5803, 0x0f);
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5804, 0x12);
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x5805, 0x26);

  // ========== TEST PATTERN ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x503D, 0x00); // Test pattern OFF (real camera mode)
  uprintf("Test pattern disabled - real camera mode\r\n");

  // ========== STREAM ON ==========
  wr8(&hi2c1, OV5640_HAL_ADDR, 0x3008, 0x02); // Wake up from standby, enable streaming
  HAL_Delay(200); // Wait for auto exposure/white balance to stabilize

  // Verify critical registers
  if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x503D, &v) == HAL_OK)
    uprintf("OV5640 0x503D=0x%02X (test pattern)\r\n", v);
  if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x3008, &v) == HAL_OK)
    uprintf("OV5640 0x3008=0x%02X (streaming)\r\n", v);
  if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x3503, &v) == HAL_OK)
    uprintf("OV5640 0x3503=0x%02X (AE/AGC auto)\r\n", v);
  if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x3406, &v) == HAL_OK)
    uprintf("OV5640 0x3406=0x%02X (AWB auto)\r\n", v);
  if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x5001, &v) == HAL_OK)
    uprintf("OV5640 0x5001=0x%02X (ISP AWB)\r\n", v);

  uprintf("OV5640 Init Complete: QVGA 320x240 RGB565 with AE/AWB\r\n");
  return HAL_OK;
}

/* ==================== GPIO Initialization ==================== */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin   = GPIO_PIN_0 | GPIO_PIN_1;  // PD0=PWDN, PD1=RST
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &g);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
}

static void MX_GPIO_DCMI_AF_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF13_DCMI;

  // HSYNC(PA4), PCLK(PA6)
  g.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &g);

  // VSYNC(PB7)
  g.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &g);

  // D0-D3 (PC6-PC9)
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &g);

  // D5(PD3)
  g.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOD, &g);

  // D7(PE1), D4(PE4), D6(PE6)
  g.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOE, &g);
}

static void MX_GPIO_DCMI_INPUT_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = GPIO_PULLDOWN;  // Prevent floating
  g.Speed = GPIO_SPEED_FREQ_LOW;

  // PCLK(PA6), HSYNC(PA4)
  g.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &g);

  // VSYNC(PB7)
  g.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &g);

  // D0-D3 (PC6-PC9)
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &g);

  // D4(PE4), D6(PE6), D7(PE1)
  g.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOE, &g);

  // D5(PD3)
  g.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOD, &g);
}

/* ==================== Peripheral Initialization ==================== */
static void MX_USART3_UART_Init(void)
{
  __HAL_RCC_USART3_CLK_ENABLE();

  huart3.Instance                    = USART3;
  huart3.Init.BaudRate               = 2000000;
  huart3.Init.WordLength             = UART_WORDLENGTH_8B;
  huart3.Init.StopBits               = UART_STOPBITS_1;
  huart3.Init.Parity                 = UART_PARITY_NONE;
  huart3.Init.Mode                   = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
  HAL_UARTEx_DisableFifoMode(&huart3);
}

static void MX_I2C1_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();

  hi2c1.Instance                  = I2C1;
  hi2c1.Init.Timing               = 0x10707DBC;  // ~400kHz
  hi2c1.Init.OwnAddress1          = 0;
  hi2c1.Init.AddressingMode       = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode      = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2          = 0;
  hi2c1.Init.OwnAddress2Masks     = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode      = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode        = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DCMI -> DMA1 Stream0 설정
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
  hdma_dcmi.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_dcmi.Init.MemBurst            = DMA_MBURST_SINGLE;
  hdma_dcmi.Init.PeriphBurst         = DMA_PBURST_SINGLE;

  if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK) {
    uprintf("DMA Init FAIL!\r\n");
    Error_Handler();
  }

  // DCMI <-> DMA 연결
  __HAL_LINKDMA(&hdcmi, DMA_Handle, hdma_dcmi);

  // NVIC 설정
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  // DCMI 인터럽트도 활성화
  HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);

  uprintf("DMA Init OK\r\n");
}
static void MX_DCMI_Init(void)
{
  __HAL_RCC_DCMI_CLK_ENABLE();

  hdcmi.Instance              = DCMI;
  hdcmi.Init.SynchroMode      = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity      = DCMI_PCKPOLARITY_FALLING;  // ← FALLING으로 변경
  hdcmi.Init.VSPolarity       = DCMI_VSPOLARITY_LOW;       // ← LOW로 변경
  hdcmi.Init.HSPolarity       = DCMI_HSPOLARITY_HIGH;      // ← HIGH로 변경     // ★ HSYNC active LOW
  hdcmi.Init.CaptureRate      = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode         = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode   = DCMI_BSM_ALL;
  hdcmi.Init.LineSelectMode   = DCMI_LSM_ALL;

  if (HAL_DCMI_Init(&hdcmi) != HAL_OK) Error_Handler();

  uprintf("DCMI Init: VS_LOW HS_HIGH PCK_FALLING\r\n");  // ← 메시지 변경
}

static void MX_MCO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_8;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &g);

  // Output 24MHz (HSI48 / 2)
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_2);
}

/* ==================== System Configuration ==================== */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef R = {0};
  HAL_MPU_Disable();

  R.Enable           = MPU_REGION_ENABLE;
  R.Number           = MPU_REGION_NUMBER0;
  R.BaseAddress      = 0x24000000;  // D1 SRAM
  R.Size             = MPU_REGION_SIZE_512KB;
  R.SubRegionDisable = 0x00;
  R.TypeExtField     = MPU_TEX_LEVEL1;
  R.AccessPermission = MPU_REGION_FULL_ACCESS;
  R.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  R.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  R.IsCacheable      = MPU_ACCESS_CACHEABLE;
  R.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&R);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef O = {0};
  RCC_ClkInitTypeDef C = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)){}

  O.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  O.HSIState            = RCC_HSI_ON;
  O.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  O.PLL.PLLState        = RCC_PLL_ON;
  O.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  O.PLL.PLLM            = 4;
  O.PLL.PLLN            = 24;
  O.PLL.PLLP            = 2;  // SYSCLK ≈ 64MHz
  O.HSI48State          = RCC_HSI48_ON;

  if (HAL_RCC_OscConfig(&O) != HAL_OK) Error_Handler();

  C.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  C.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  C.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  C.AHBCLKDivider  = RCC_HCLK_DIV1;
  C.APB3CLKDivider = RCC_APB3_DIV1;
  C.APB1CLKDivider = RCC_APB1_DIV1;
  C.APB2CLKDivider = RCC_APB2_DIV1;
  C.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&C, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

/* ==================== Error Handler ==================== */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    // System halted
  }
}

