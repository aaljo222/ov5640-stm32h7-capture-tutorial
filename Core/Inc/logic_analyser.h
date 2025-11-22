/**
 ******************************************************************************
 * @file    logic_analyser.h
 * @brief   DCMI Signal Logic Analyzer
 ******************************************************************************
 */

#ifndef LOGIC_ANALYSER_H
#define LOGIC_ANALYSER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Run logic analyzer for DCMI signals
 * @note   Samples PCLK, HSYNC, VSYNC, D0-D7 for 10ms
 * @retval None
 */
void logic_analyzer_run(void);

#ifdef __cplusplus
}
#endif

#endif /* LOGIC_ANALYSER_H */
