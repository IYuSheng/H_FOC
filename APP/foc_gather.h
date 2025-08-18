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

// FOC电流转换结构体
typedef struct {
    float ia;    // A相电流
    float ib;    // B相电流
    float ic;    // C相电流
    float va;    // A相电压
    float vb;    // B相电压
    float vc;    // C相电压
    float vbus;  // 母线电压
} foc_data;

void foc_gather_init(void);

void vGatherProcessTask(void *pvParameters);

#endif /* __FOC_GATHER_CurrentH */
