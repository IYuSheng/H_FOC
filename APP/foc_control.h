#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "stm32f4xx.h"
#include "freertos.h"
#include "task.h"
#include "foc_conversion.h"
#include "foc_encoder.h"
#include "bsp_timer.h"
#include "Config.h"
#include "math.h"
#include "foc_debug.h"

// FOC控制参数结构体
typedef struct
{
  float u_d;              // D轴电压
  float u_q;              // Q轴电压
  float angle;            // 电角度
  float speed;            // 速度（rad/s）
  float speed_rpm;        // 速度（rpm）
  float target_speed;     // 目标速度
} foc_control_t;

// 函数声明
void foc_control_init(void);
void foc_open_loop_control(void);
void foc_set_open_loop_speed(float speed_rpm);
void vFOCControlTask(void *pvParameters);

#endif /* __FOC_CONTROL_H */
