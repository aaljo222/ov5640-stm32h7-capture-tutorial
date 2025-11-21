// Core/Src/logic_analyser.c
#include "main.h"
#include "logic_analyser.h"

// main.c 쪽 uprintf 사용
extern void uprintf(const char *fmt, ...);

/*
 * 샘플링 윈도우 (ms 단위)
 *  - 너무 길면 측정은 정확하지만 출력이 느려지고
 *  - 너무 짧으면 통계가 거칠어짐
 */
#define LA_WINDOW_MS   10u   // 10ms 동안 샘플링

// 내부 편의를 위한 매크로
#define PIN_IS_HIGH(port, pinmask)   (((port) & (pinmask)) ? 1u : 0u)

/*
 * 논리 분석기 1회 실행
 * - FW_MODE_ANALYZER 모드에서 main() 루프에서 주기적으로 호출
 * - PCLK, HSYNC, VSYNC, D0..D7의 토글/High비율/대략적인 PCLK 주파수 출력
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

    // 첫 샘플은 "prev" 초기화를 위해 한 번 읽고 시작
    {
        uint32_t a = GPIOA->IDR;
        uint32_t b = GPIOB->IDR;
        uint32_t c = GPIOC->IDR;
        uint32_t d = GPIOD->IDR;
        uint32_t e = GPIOE->IDR;

        prev_pclk  = PIN_IS_HIGH(a, GPIO_PIN_6);  // PCLK = PA6
        prev_hsync = PIN_IS_HIGH(a, GPIO_PIN_4);  // HSYNC = PA4
        prev_vsync = PIN_IS_HIGH(b, GPIO_PIN_7);  // VSYNC = PB7

        prev_d0 = PIN_IS_HIGH(c, GPIO_PIN_6);     // PC6 = OV5640 D2
        prev_d1 = PIN_IS_HIGH(c, GPIO_PIN_7);     // PC7 = OV5640 D3
        prev_d2 = PIN_IS_HIGH(c, GPIO_PIN_8);     // PC8 = OV5640 D4
        prev_d3 = PIN_IS_HIGH(c, GPIO_PIN_9);     // PC9 = OV5640 D5
        prev_d4 = PIN_IS_HIGH(e, GPIO_PIN_4);     // PE4 = OV5640 D6
        prev_d5 = PIN_IS_HIGH(d, GPIO_PIN_3);     // PD3 = OV5640 D7
        prev_d6 = PIN_IS_HIGH(e, GPIO_PIN_5);     // PE5 = OV5640 D8
        prev_d7 = PIN_IS_HIGH(e, GPIO_PIN_6);     // PE6 = OV5640 D9
    }

    // ==== 샘플링 루프 ====
    while (1)
    {
        now_ms = HAL_GetTick();
        if ((now_ms - start_ms) >= LA_WINDOW_MS) {
            break;
        }

        uint32_t a = GPIOA->IDR;
        uint32_t b = GPIOB->IDR;
        uint32_t c = GPIOC->IDR;
        uint32_t d = GPIOD->IDR;
        uint32_t e = GPIOE->IDR;

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
        uprintf("[LA] No samples (window too short?)\r\n");
        return;
    }

    float elapsed_ms = (float)(now_ms - start_ms);
    float elapsed_s  = elapsed_ms / 1000.0f;

    // HIGH 비율 계산 (%)
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

    // PCLK 대략적인 주파수 (토글 2개 = 1주기)
    float pclk_freq = 0.0f;
    if (elapsed_s > 0.0f) {
        pclk_freq = ((float)pclk_toggles / 2.0f) / elapsed_s;
    }

    // ==== 출력 ====
    uprintf("\r\n[LOGIC ANALYZER] window=%lu ms, samples=%lu\r\n",
            (uint32_t)elapsed_ms, (uint32_t)samples);

    uprintf("PCLK(PA6):  toggles=%6lu, HIGH=%3lu%%\r\n", pclk_toggles,  pclk_high_pct);
    uprintf("HSYNC(PA4): toggles=%6lu, HIGH=%3lu%%\r\n", hsync_toggles, hsync_high_pct);
    uprintf("VSYNC(PB7): toggles=%6lu, HIGH=%3lu%%\r\n", vsync_toggles, vsync_high_pct);

    uprintf("D2(PC6):    toggles=%6lu, HIGH=%3lu%%\r\n", d0_toggles, d0_high_pct);
    uprintf("D3(PC7):    toggles=%6lu, HIGH=%3lu%%\r\n", d1_toggles, d1_high_pct);
    uprintf("D4(PC8):    toggles=%6lu, HIGH=%3lu%%\r\n", d2_toggles, d2_high_pct);
    uprintf("D5(PC9):    toggles=%6lu, HIGH=%3lu%%\r\n", d3_toggles, d3_high_pct);

    uprintf("D6(PE4):    toggles=%6lu, HIGH=%3lu%%\r\n", d4_toggles, d4_high_pct);
    uprintf("D7(PD3):    toggles=%6lu, HIGH=%3lu%%\r\n", d5_toggles, d5_high_pct);
    uprintf("D8(PE5):    toggles=%6lu, HIGH=%3lu%%\r\n", d6_toggles, d6_high_pct);
    uprintf("D9(PE6):    toggles=%6lu, HIGH=%3lu%%\r\n", d7_toggles, d7_high_pct);

    uprintf("PCLK approx: %.1f Hz\r\n", pclk_freq);
}

