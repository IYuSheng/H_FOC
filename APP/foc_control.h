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
#include "arm_math.h"

// FOC控制参数结构体
typedef struct
{
  float32_t u_d;          // D轴电压（使用DSP库的float32_t类型）
  float32_t u_q;          // Q轴电压
  float32_t angle;        // 电角度
  float32_t speed;        // 速度（rad/s）
  float32_t speed_rpm;    // 速度（rpm）
  float32_t target_speed; // 目标速度
} foc_control_t;

// 函数声明
void foc_control_init(void);
void foc_open_loop_control(void);
void foc_set_open_loop_speed(float speed_rpm);
void vFOCControlTask(void *pvParameters);

#endif /* __FOC_CONTROL_H */
