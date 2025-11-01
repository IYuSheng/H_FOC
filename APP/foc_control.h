#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "stm32f4xx.h"
#include "freertos.h"
#include "task.h"
#include "arm_math.h"
#include "foc_encoder.h"
#include "bsp_timer.h"
#include "Config.h"
#include "foc_debug.h"
#include "foc_gather.h"
#include "foc_conversion.h"
#include "arm_math.h"
#include "foc_settings.h"

// 函数声明
void foc_control_init(void);
void foc_open_loop_control(void);
void foc_position_control(void);
void foc_current_control(void);
void foc_speed_control(void);
void foc_set_open_loop_speed(float speed_rpm);
void vFOCControlTask(void *pvParameters);

#endif /* __FOC_CONTROL_H */
