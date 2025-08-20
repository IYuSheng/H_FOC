#ifndef __FOC_GATHER_CurrentH
#define __FOC_GATHER_CurrentH

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "freertos.h"
#include "task.h"
#include "bsp_adc.h"
#include "foc_debug.h"
#include "Config.h"

void foc_gather_init(void);

void vGatherProcessTask(void *pvParameters);

#endif /* __FOC_GATHER_CurrentH */
