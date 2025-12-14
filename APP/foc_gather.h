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
#include "foc_encoder.h"
#include "Config.h"
#include "bsp_timer.h"

// 外部声明电角度及速度
extern float g_total_angle;
extern float g_angle;
extern float32_t g_speed;
extern float32_t bemf_angle;
extern float32_t bemf_speed;

void vGatherProcessTask(void *pvParameters);

#endif /* __FOC_GATHER_CurrentH */
