
#pragma once

#include "SEGGER_RTT.h"
void log_inf(const char *fmt, ...);

//#define LOG_INF(...) log_inf(__VA_ARGS__)
#define LOG_INF(...) SEGGER_RTT_printf(0,__VA_ARGS__)
int log_init();


