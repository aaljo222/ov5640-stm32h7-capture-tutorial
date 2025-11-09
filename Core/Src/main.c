/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : OV5640 + DCMI + RAW(QVGA, RGB565) + UART Snapshot
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ================== Global Handles ================== */
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef  hdma_dcmi;
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart3;

/* ================== Camera / Frame Buff ================== */

#define OV5640_ADDR_7B      0x3C
#define OV5640_HAL_ADDR     (OV5640_ADDR_7B << 1)

#define FRAME_W           320
#define FRAME_H           240
#define BYTES_PER_PIXEL   2           // RGB565
#define FRAME_SIZE        (FRAME_W * FRAME_H * BYTES_PER_PIXEL)

/* D1 SRAM (.ram_d1), 32-byte aligned for DCache-safe DMA */
#if defined(__ICCARM__)
#pragma location = 0x24000000
__attribute__((aligned(32))) uint8_t frame_buf[FRAME_SIZE];
#elif defined(__GNUC__)
__attribute__((section(".ram_d1")))
__attribute__((aligned(32)))
uint8_t frame_buf[FRAME_SIZE];
#else
__attribute__((aligned(32))) uint8_t frame_buf[FRAME_SIZE];
#endif

volatile uint32_t g_frame_ready        = 0;
volatile uint32_t g_actual_payload_len = 0;
volatile uint32_t g_hsync_cnt          = 0;
volatile uint32_t g_vsync_cnt          = 0;
static  uint32_t last_print            = 0;

/* ================== Prototypes ================== */

static void MPU_Config(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_MCO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);

static void OV5640_PowerUp(void);
static HAL_StatusTypeDef OV5640_ReadID(uint16_t *id);
static HAL_StatusTypeDef OV5640_Init_RAW_QVGA(void);

static HAL_StatusTypeDef wr8(I2C_HandleTypeDef *hi2c,
                             uint8_t hal_addr,
                             uint16_t reg,
                             uint8_t val);
static HAL_StatusTypeDef rd8(I2C_HandleTypeDef *hi2c,
                             uint8_t hal_addr,
                             uint16_t reg,
                             uint8_t *val);

static void dcmi_pin_test(void);
static void dcmi_signal_test(void); /* 필요시 사용 */

void uprintf(const char *fmt, ...);
void Error_Handler(void);

/* =========================================================
 *                       main
 * =======================================================*/
int main(void)
{
    /* MPU & HAL & Clock */
    MPU_Config();
    HAL_Init();
    SystemClock_Config();

    /* D2 Domain Clock ready (H7 DCMI 관련 중요) */
    //while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) {}

    /* Peripherals */
    MX_GPIO_Init();       // PWDN, RESET
    MX_MCO_Init();        // MCO1 -> 24MHz XCLK to OV5640
    MX_USART3_UART_Init();
    uprintf("\r\n=== DCMI RAW Snapshot Start ===\r\n");
        uprintf("frame_buf addr = 0x%08lX, size=%lu\r\n",
                (uint32_t)frame_buf, (uint32_t)FRAME_SIZE)    ;

    MX_I2C1_Init();
    MX_DMA_Init();
    MX_DCMI_Init();


    /* Camera Power & Init */
    OV5640_PowerUp();
    if (OV5640_Init_RAW_QVGA() != HAL_OK)
    {
        uprintf("OV5640 RAW init FAIL\r\n");
        Error_Handler();
    }

    /* DCMI 신호 토글 확인 (테스트 패턴 기준) */
    dcmi_pin_test();

    /* Frame buffer 초기화 + Cache 정리 */
    memset(frame_buf, 0, FRAME_SIZE);
    SCB_CleanDCache_by_Addr((uint32_t*)frame_buf, FRAME_SIZE);
    SCB_InvalidateDCache_by_Addr((uint32_t*)frame_buf, FRAME_SIZE);

    /* Snapshot DMA 시작 (Word count = FRAME_SIZE/4) */
    HAL_StatusTypeDef ret =
        HAL_DCMI_Start_DMA(&hdcmi,
                           DCMI_MODE_SNAPSHOT,
                           (uint32_t)frame_buf,
                           FRAME_SIZE / 4);
    uprintf("HAL_DCMI_Start_DMA ret = %d\r\n", ret);
    if (ret != HAL_OK)
        Error_Handler();

    __HAL_DCMI_ENABLE_IT(&hdcmi,
        DCMI_IT_FRAME | DCMI_IT_VSYNC | DCMI_IT_LINE | DCMI_IT_ERR);
    /* ===== Main Loop ===== */
    while (1)
    {
        /* 1초마다 상태 출력 */
        if (HAL_GetTick() - last_print > 1000)
        {
            last_print = HAL_GetTick();
            uprintf("VSYNC=%lu, HSYNC=%lu, DMA_state=%d, DCMI_state=%d\r\n",
                    g_vsync_cnt,
                    g_hsync_cnt,
                    (int)hdcmi.DMA_Handle->State,
                    (int)hdcmi.State);
        }

        /* 프레임 캡처 완료 처리 (DMA 콜백이 g_frame_ready 세팅) */
        if (g_frame_ready)
        {
            g_frame_ready = 0;

            uint32_t payload_len = g_actual_payload_len;
            if (payload_len == 0 || payload_len > FRAME_SIZE)
                payload_len = FRAME_SIZE;

            /* 헤더 [0..3]=MAGIC, [4..7]=LEN(LE) */
            uint8_t hdr[8] = {
                0xAA, 0x55, 0xAA, 0x55,
                (uint8_t)(payload_len),
                (uint8_t)(payload_len >> 8),
                (uint8_t)(payload_len >> 16),
                (uint8_t)(payload_len >> 24)
            };

            HAL_UART_Transmit(&huart3, hdr, sizeof(hdr), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart3, frame_buf, payload_len, HAL_MAX_DELAY);

            uprintf("\r\n*** FRAME SENT (%lu bytes) ***\r\n", payload_len);

            /* 다음 프레임용 버퍼 초기화 + DMA 재시작
               - 한 장만 찍고 싶으면 이 블록을 지우면 됨 */
            memset(frame_buf, 0, FRAME_SIZE);
            SCB_CleanDCache_by_Addr((uint32_t*)frame_buf, FRAME_SIZE);
            SCB_InvalidateDCache_by_Addr((uint32_t*)frame_buf, FRAME_SIZE);

            HAL_StatusTypeDef r =
                HAL_DCMI_Start_DMA(&hdcmi,
                                   DCMI_MODE_SNAPSHOT,
                                   (uint32_t)frame_buf,
                                   FRAME_SIZE / 4);
            if (r != HAL_OK)
                Error_Handler();
        }

        HAL_Delay(10);
    }
}

/* =========================================================
 *                   Camera Power / I2C Helpers
 * =======================================================*/

static void OV5640_PowerUp(void)
{
    /* PWDN=0, RESET: low -> high, XCLK already provided by MCO */
    HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);

    HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    uprintf("OV5640 PowerUp: RESET=1, PWDN=0\r\n");
}

static HAL_StatusTypeDef wr8(I2C_HandleTypeDef *hi2c,
                             uint8_t hal_addr,
                             uint16_t reg,
                             uint8_t val)
{
    uint8_t pkt[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, val };
    return HAL_I2C_Master_Transmit(hi2c, hal_addr, pkt, 3, 100);
}

static HAL_StatusTypeDef rd8(I2C_HandleTypeDef *hi2c,
                             uint8_t hal_addr,
                             uint16_t reg,
                             uint8_t *val)
{
    return HAL_I2C_Mem_Read(hi2c,
                            hal_addr,
                            reg,
                            I2C_MEMADD_SIZE_16BIT,
                            val,
                            1,
                            100);
}

static HAL_StatusTypeDef OV5640_ReadID(uint16_t *id)
{
    uint8_t high = 0, low = 0;
    if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x300A, &high) != HAL_OK) return HAL_ERROR;
    if (rd8(&hi2c1, OV5640_HAL_ADDR, 0x300B, &low)  != HAL_OK) return HAL_ERROR;

    *id = ((uint16_t)high << 8) | low;
    return HAL_OK;
}

/* =========================================================
 *           OV5640 RAW QVGA (RGB565) Init (with test pattern)
 * =======================================================*/

static HAL_StatusTypeDef OV5640_Init_RAW_QVGA(void)
{
    HAL_StatusTypeDef ret;
    uint16_t id = 0;
    uint8_t v;

    ret = OV5640_ReadID(&id);
    if (ret != HAL_OK)
    {
        uprintf("OV5640 ID read FAIL\r\n");
        return ret;
    }
    uprintf("OV5640 ID = 0x%04X\r\n", id);
    if (id != 0x5640)
    {
        uprintf("OV5640 ID mismatch\r\n");
        return HAL_ERROR;
    }

    /* Soft reset */
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3008, 0x82);
    HAL_Delay(100);

    /* Clock / PLL (완전 튜닝은 아니고 안정적인 저속 설정) */
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3103, 0x11); // system clock from PLL
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3034, 0x1A);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3035, 0x21);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3036, 0x46);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3037, 0x13);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3108, 0x01);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3824, 0x02); // PCLK divider

    /* Window: full sensor to QVGA (binning/scale) */
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3800, 0x00);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3801, 0x00);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3802, 0x00);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3803, 0x00);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3804, 0x0A);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3805, 0x3F);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3806, 0x07);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3807, 0x9F);

    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3808, 0x01); // 320
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3809, 0x40);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x380A, 0x00); // 240
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x380B, 0xF0);

    /* Timing */
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x380C, 0x07);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x380D, 0x68);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x380E, 0x03);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x380F, 0xD8);

    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3814, 0x31);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3815, 0x31);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3820, 0x47);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3821, 0x07);

    /* RGB565 출력 */
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x4300, 0x61); // RGB565
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x501F, 0x01); // ISP -> DVP (no JPEG)

    /* DVP pads enable */
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3017, 0xFF);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3018, 0xFF);
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x4740, 0x21); // VS/HS/PCLK, active high

    /* Test pattern ON (색바 표시, DCMI 확인용) */
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x503D, 0x80);
    uprintf("Test pattern ENABLED (RAW)\r\n");

    /* Stream ON */
    wr8(&hi2c1, OV5640_HAL_ADDR, 0x3008, 0x02);
    HAL_Delay(100);

    rd8(&hi2c1, OV5640_HAL_ADDR, 0x3008, &v);
    uprintf("0x3008 = 0x%02X\r\n", v);
    rd8(&hi2c1, OV5640_HAL_ADDR, 0x4300, &v);
    uprintf("0x4300 = 0x%02X (format)\r\n", v);

    uprintf("OV5640 RAW QVGA Init OK\r\n");
    return HAL_OK;
}

/* =========================================================
 *                   DCMI Pin Toggle Test
 * =======================================================*/

static void dcmi_pin_test(void)
{
    uprintf("\r\n=== DCMI Pin Toggle Test ===\r\n");

    struct {
        GPIO_TypeDef *port;
        uint16_t pin;
        const char *name;
    } pins[] = {
            {GPIOA, GPIO_PIN_4, "HSYNC(PA4)"},
            {GPIOA, GPIO_PIN_6, "PCLK(PA6)"},
            {GPIOB, GPIO_PIN_7, "VSYNC(PB7)"},

            // OV5640 D2~D9 → DCMI_D0~D7 매핑 예시
            {GPIOC, GPIO_PIN_6, "D2 -> DCMI_D0 (PC6)"},
            {GPIOC, GPIO_PIN_7, "D3 -> DCMI_D1 (PC7)"},
            {GPIOC, GPIO_PIN_8, "D4 -> DCMI_D2 (PC8)"},
            {GPIOC, GPIO_PIN_9, "D5 -> DCMI_D3 (PC9)"},
            {GPIOE, GPIO_PIN_4, "D6 -> DCMI_D4 (PE4)"},
            {GPIOD, GPIO_PIN_3, "D7 -> DCMI_D5 (PD3)"},
            {GPIOE, GPIO_PIN_6, "D8 -> DCMI_D6 (PE6)"},
            {GPIOE, GPIO_PIN_1, "D9 -> DCMI_D7 (PE1)"},
        };
    for (uint32_t i = 0; i < sizeof(pins)/sizeof(pins[0]); i++)
    {
        int toggles = 0;
        GPIO_PinState last = HAL_GPIO_ReadPin(pins[i].port, pins[i].pin);

        uint32_t start = HAL_GetTick();
        while (HAL_GetTick() - start < 200)
        {
            GPIO_PinState now = HAL_GPIO_ReadPin(pins[i].port, pins[i].pin);
            if (now != last)
            {
                toggles++;
                last = now;
            }
        }

        uprintf("  %s: %4d toggles", pins[i].name, toggles);
        if (toggles > 100)       uprintf(" ✓\r\n");
        else if (toggles > 0)    uprintf(" ? (low)\r\n");
        else                     uprintf(" ✗ NO SIGNAL\r\n");
    }
}

/* ===================== UART printf ===================== */

void uprintf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) return;
    if (n > (int)sizeof(buf))
        n = sizeof(buf) - 1;

    HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
}

/* ===================== MPU / System / Init ===================== */

static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    /* D1 SRAM (0x24000000 512KB) - cacheable, bufferable */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x24000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void SystemClock_Config(void)
{
    /* 사용 중이던 간단한 HSI 기반 설정 유지 (400MHz까지 안 올려도 OK)
       필요하면 여기서 HCLK/DCMI 클럭 여유 있게 설정 */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4;
    RCC_OscInitStruct.PLL.PLLN            = 24;
    RCC_OscInitStruct.PLL.PLLP            = 2;
    RCC_OscInitStruct.PLL.PLLQ            = 8;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                       RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2|
                                       RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    /* PWDN/RESET 핀만 설정 (나머지 DCMI 핀은 MSP에서 설정) */
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PWDN */
    GPIO_InitStruct.Pin   = CAM_PWDN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAM_PWDN_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);

    /* RESET */
    GPIO_InitStruct.Pin = CAM_RESET_Pin;
    HAL_GPIO_Init(CAM_RESET_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_RESET);
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

    /* 24MHz for OV5640 XCLK */
    HAL_RCC_MCOConfig(RCC_MCO1,
                      RCC_MCO1SOURCE_PLL1QCLK,
                      RCC_MCODIV_2);  // 48MHz / 2 = 24MHz
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 2000000;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling   = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler   = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
        Error_Handler();
    HAL_UARTEx_DisableFifoMode(&huart3);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x10707DBC;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.OwnAddress2Masks= I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();

    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

static void MX_DCMI_Init(void)
{
    __HAL_RCC_DCMI_CLK_ENABLE();

    /* 하드 리셋 */
    __HAL_RCC_DCMI_FORCE_RESET();
    HAL_Delay(10);
    __HAL_RCC_DCMI_RELEASE_RESET();
    HAL_Delay(10);

    hdcmi.Instance         = DCMI;
    hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
    hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
    hdcmi.Init.VSPolarity  = DCMI_VSPOLARITY_HIGH;
    hdcmi.Init.HSPolarity  = DCMI_HSPOLARITY_HIGH;
    hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
    hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    hdcmi.Init.JPEGMode    = DCMI_JPEG_DISABLE;
    hdcmi.Init.ByteSelectMode  = DCMI_BSM_ALL;
    hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
    hdcmi.Init.LineSelectMode  = DCMI_LSM_ALL;
    hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;

    if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
        Error_Handler();

//    __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_ERR);
}

/* ===================== Callbacks ===================== */

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi_cb)
{
	uprintf("[IRQ] FRAME\r\n");
	HAL_DCMI_Stop(hdcmi_cb);

    uint32_t remaining_words = __HAL_DMA_GET_COUNTER(hdcmi_cb->DMA_Handle);
    uint32_t total_words     = FRAME_SIZE / 4;
    if (remaining_words > total_words)
        remaining_words = 0;

    uint32_t done_words = total_words - remaining_words;
    g_actual_payload_len = done_words * 4;
    if (g_actual_payload_len > FRAME_SIZE)
        g_actual_payload_len = FRAME_SIZE;

    SCB_InvalidateDCache_by_Addr((uint32_t*)frame_buf, FRAME_SIZE);

    //uprintf("\r\n[FRAME IRQ] bytes=%lu\r\n", g_actual_payload_len);
    g_frame_ready = 1;
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi_cb)
{
    g_vsync_cnt++;
    uprintf("[IRQ] VSYNC %lu\r\n", g_vsync_cnt);
}

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi_cb)
{
    g_hsync_cnt++;
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi_cb)
{
    uprintf("\r\n!!! DCMI ERROR: 0x%08lX !!!\r\n", hdcmi_cb->ErrorCode);
}
void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_dcmi)
    {
        /* 한 프레임(320x240x2) 다 채움 */
        HAL_DCMI_Stop(&hdcmi);

        g_actual_payload_len = FRAME_SIZE;
        SCB_InvalidateDCache_by_Addr((uint32_t*)frame_buf, FRAME_SIZE);

        g_frame_ready = 1;
    }
}

/* ===================== Error ===================== */

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // 에러시 여기서 멈춤. 필요하면 LED 토글.
    }
}
