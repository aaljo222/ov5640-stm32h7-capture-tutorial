/**
 ******************************************************************************
 * @file    logic_analyser.c
 * @brief   DCMI Signal Logic Analyzer - FINAL
 ******************************************************************************
 */

#include "main.h"
#include "logic_analyser.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>  // va_list를 위해 필수!

// UART 전송 함수
extern UART_HandleTypeDef huart3;

static void uprintf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 1000);
}

/*
 * 샘플링 윈도우 (ms)
 */
#define LA_WINDOW_MS   100u   // 100ms

// 핀 읽기 매크로
#define PIN_IS_HIGH(port, pinmask)   (((port) & (pinmask)) ? 1u : 0u)

/**
 * @brief  Logic Analyzer 실행
 */
void logic_analyzer_run(void)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t now_ms   = start_ms;
    uint32_t samples = 0;

    // 이전 상태 (edge 검출용)
    uint8_t prev_pclk  = 0;
    uint8_t prev_hsync = 0;
    uint8_t prev_vsync = 0;
    uint8_t prev_d0 = 0, prev_d1 = 0, prev_d2 = 0, prev_d3 = 0;
    uint8_t prev_d4 = 0, prev_d5 = 0, prev_d6 = 0, prev_d7 = 0;

    // 토글 카운트
    uint32_t pclk_toggles  = 0;
    uint32_t hsync_toggles = 0;
    uint32_t vsync_toggles = 0;
    uint32_t d0_toggles = 0, d1_toggles = 0, d2_toggles = 0, d3_toggles = 0;
    uint32_t d4_toggles = 0, d5_toggles = 0, d6_toggles = 0, d7_toggles = 0;

    // HIGH 샘플 카운트
    uint32_t pclk_high  = 0;
    uint32_t hsync_high = 0;
    uint32_t vsync_high = 0;
    uint32_t d0_high = 0, d1_high = 0, d2_high = 0, d3_high = 0;
    uint32_t d4_high = 0, d5_high = 0, d6_high = 0, d7_high = 0;

    // 첫 샘플 (prev 초기화)
    {
        uint32_t a = GPIOA->IDR;
        uint32_t b = GPIOB->IDR;
        uint32_t c = GPIOC->IDR;
        uint32_t d = GPIOD->IDR;
        uint32_t e = GPIOE->IDR;

        prev_pclk  = PIN_IS_HIGH(a, GPIO_PIN_6);   // PA6
        prev_hsync = PIN_IS_HIGH(a, GPIO_PIN_4);   // PA4
        prev_vsync = PIN_IS_HIGH(b, GPIO_PIN_7);   // PB7

        prev_d0 = PIN_IS_HIGH(c, GPIO_PIN_6);   // PC6
        prev_d1 = PIN_IS_HIGH(c, GPIO_PIN_7);   // PC7
        prev_d2 = PIN_IS_HIGH(c, GPIO_PIN_8);   // PC8
        prev_d3 = PIN_IS_HIGH(c, GPIO_PIN_9);   // PC9
        prev_d4 = PIN_IS_HIGH(e, GPIO_PIN_4);   // PE4
        prev_d5 = PIN_IS_HIGH(d, GPIO_PIN_3);   // PD3
        prev_d6 = PIN_IS_HIGH(e, GPIO_PIN_5);   // PE5
        prev_d7 = PIN_IS_HIGH(e, GPIO_PIN_6);   // PE6
    }

    uprintf("\r\n[Logic Analyzer] Starting %lu ms sampling...\r\n", LA_WINDOW_MS);

    // ==== 샘플링 루프 ====
    while (1)
    {
        now_ms = HAL_GetTick();
        if ((now_ms - start_ms) >= LA_WINDOW_MS) {
            break;
        }

        // GPIO 레지스터 읽기
        uint32_t a = GPIOA->IDR;
        uint32_t b = GPIOB->IDR;
        uint32_t c = GPIOC->IDR;
        uint32_t d = GPIOD->IDR;
        uint32_t e = GPIOE->IDR;

        // 현재 상태
        uint8_t cur_pclk  = PIN_IS_HIGH(a, GPIO_PIN_6);
        uint8_t cur_hsync = PIN_IS_HIGH(a, GPIO_PIN_4);
        uint8_t cur_vsync = PIN_IS_HIGH(b, GPIO_PIN_7);

        uint8_t cur_d0 = PIN_IS_HIGH(c, GPIO_PIN_6);
        uint8_t cur_d1 = PIN_IS_HIGH(c, GPIO_PIN_7);
        uint8_t cur_d2 = PIN_IS_HIGH(c, GPIO_PIN_8);
        uint8_t cur_d3 = PIN_IS_HIGH(c, GPIO_PIN_9);
        uint8_t cur_d4 = PIN_IS_HIGH(e, GPIO_PIN_4);
        uint8_t cur_d5 = PIN_IS_HIGH(d, GPIO_PIN_3);
        uint8_t cur_d6 = PIN_IS_HIGH(e, GPIO_PIN_5);
        uint8_t cur_d7 = PIN_IS_HIGH(e, GPIO_PIN_6);

        // HIGH 샘플 수
        if (cur_pclk)  pclk_high++;
        if (cur_hsync) hsync_high++;
        if (cur_vsync) vsync_high++;
        if (cur_d0) d0_high++;
        if (cur_d1) d1_high++;
        if (cur_d2) d2_high++;
        if (cur_d3) d3_high++;
        if (cur_d4) d4_high++;
        if (cur_d5) d5_high++;
        if (cur_d6) d6_high++;
        if (cur_d7) d7_high++;

        // 토글 검출
        if (cur_pclk  != prev_pclk)  { pclk_toggles++;  prev_pclk  = cur_pclk;  }
        if (cur_hsync != prev_hsync) { hsync_toggles++; prev_hsync = cur_hsync; }
        if (cur_vsync != prev_vsync) { vsync_toggles++; prev_vsync = cur_vsync; }
        if (cur_d0 != prev_d0) { d0_toggles++; prev_d0 = cur_d0; }
        if (cur_d1 != prev_d1) { d1_toggles++; prev_d1 = cur_d1; }
        if (cur_d2 != prev_d2) { d2_toggles++; prev_d2 = cur_d2; }
        if (cur_d3 != prev_d3) { d3_toggles++; prev_d3 = cur_d3; }
        if (cur_d4 != prev_d4) { d4_toggles++; prev_d4 = cur_d4; }
        if (cur_d5 != prev_d5) { d5_toggles++; prev_d5 = cur_d5; }
        if (cur_d6 != prev_d6) { d6_toggles++; prev_d6 = cur_d6; }
        if (cur_d7 != prev_d7) { d7_toggles++; prev_d7 = cur_d7; }

        samples++;
    }

    if (samples == 0) {
        uprintf("[LA] No samples!\r\n");
        return;
    }

    uint32_t elapsed_ms = now_ms - start_ms;
    float elapsed_s = (float)elapsed_ms / 1000.0f;

    // HIGH 비율 (%)
    uint32_t pclk_high_pct  = (pclk_high  * 100u) / samples;
    uint32_t hsync_high_pct = (hsync_high * 100u) / samples;
    uint32_t vsync_high_pct = (vsync_high * 100u) / samples;
    uint32_t d0_high_pct = (d0_high * 100u) / samples;
    uint32_t d1_high_pct = (d1_high * 100u) / samples;
    uint32_t d2_high_pct = (d2_high * 100u) / samples;
    uint32_t d3_high_pct = (d3_high * 100u) / samples;
    uint32_t d4_high_pct = (d4_high * 100u) / samples;
    uint32_t d5_high_pct = (d5_high * 100u) / samples;
    uint32_t d6_high_pct = (d6_high * 100u) / samples;
    uint32_t d7_high_pct = (d7_high * 100u) / samples;

    // PCLK 주파수 추정 (토글 2개 = 1주기)
    float pclk_freq_hz = 0.0f;
    if (elapsed_s > 0.0f) {
        pclk_freq_hz = ((float)pclk_toggles / 2.0f) / elapsed_s;
    }

    // ==== 결과 출력 ====
    uprintf("\r\n");
    uprintf("========================================\r\n");
    uprintf("  DCMI Logic Analyzer Results\r\n");
    uprintf("========================================\r\n");
    uprintf("Window: %lu ms, Samples: %lu\r\n\r\n", elapsed_ms, samples);

    uprintf("Control Signals:\r\n");
    uprintf("  PCLK  (PA6): %6lu toggles, %3lu%% HIGH", pclk_toggles,  pclk_high_pct);
    if (pclk_toggles > 0) {
        uprintf(" --> %.1f kHz\r\n", pclk_freq_hz / 1000.0f);
    } else {
        uprintf(" --> NO ACTIVITY!\r\n");
    }

    uprintf("  HSYNC (PA4): %6lu toggles, %3lu%% HIGH", hsync_toggles, hsync_high_pct);
    if (hsync_toggles == 0) uprintf(" --> NO ACTIVITY!");
    uprintf("\r\n");

    uprintf("  VSYNC (PB7): %6lu toggles, %3lu%% HIGH", vsync_toggles, vsync_high_pct);
    if (vsync_toggles == 0) uprintf(" --> NO ACTIVITY!");
    uprintf("\r\n\r\n");

    uprintf("Data Lines:\r\n");
    uprintf("  D0 (PC6): %6lu toggles, %3lu%% HIGH\r\n", d0_toggles, d0_high_pct);
    uprintf("  D1 (PC7): %6lu toggles, %3lu%% HIGH\r\n", d1_toggles, d1_high_pct);
    uprintf("  D2 (PC8): %6lu toggles, %3lu%% HIGH\r\n", d2_toggles, d2_high_pct);
    uprintf("  D3 (PC9): %6lu toggles, %3lu%% HIGH\r\n", d3_toggles, d3_high_pct);
    uprintf("  D4 (PE4): %6lu toggles, %3lu%% HIGH\r\n", d4_toggles, d4_high_pct);
    uprintf("  D5 (PD3): %6lu toggles, %3lu%% HIGH\r\n", d5_toggles, d5_high_pct);
    uprintf("  D6 (PE5): %6lu toggles, %3lu%% HIGH\r\n", d6_toggles, d6_high_pct);
    uprintf("  D7 (PE6): %6lu toggles, %3lu%% HIGH\r\n", d7_toggles, d7_high_pct);

    uprintf("========================================\r\n\r\n");

    // 진단 메시지
    if (pclk_toggles == 0) {
        uprintf("!!! PCLK NOT TOGGLING - Check PA6 connection !!!\r\n");
    }
    if (vsync_toggles == 0) {
        uprintf("!!! VSYNC NOT TOGGLING - Check PB7 connection !!!\r\n");
    }
    if (hsync_toggles == 0) {
        uprintf("!!! HSYNC NOT TOGGLING - Check PA4 connection !!!\r\n");
    }

    uint32_t data_activity = d0_toggles + d1_toggles + d2_toggles + d3_toggles +
                             d4_toggles + d5_toggles + d6_toggles + d7_toggles;
    if (data_activity == 0) {
        uprintf("!!! NO DATA LINE ACTIVITY - Check D0-D7 connections !!!\r\n");
    }

    uprintf("\r\n");
}
