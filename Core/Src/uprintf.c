#include "uprintf.h"
#include <stdarg.h>
#include <stdio.h>

extern UART_HandleTypeDef huart3;

void uprintf(const char *fmt, ...)
{
    char buf[256];

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if(len > 0)
        HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, HAL_MAX_DELAY);
}
