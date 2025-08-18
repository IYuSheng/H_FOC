#ifndef __FOC_DEBUG_H
#define __FOC_DEBUG_H

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "freertos.h"
#include "task.h"
#include "bsp_uart.h"

void debug_init(void);
void debug_printf(const char *format, ...);

void vDebugProcessTask(void *pvParameters);

#endif /* __FOC_DEBUG_H */
