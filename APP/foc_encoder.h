#ifndef __FOC_ENCODER_H
#define __FOC_ENCODER_H

#include "stm32f4xx.h"
#include "freertos.h"
#include "task.h"
#include "Config.h"
#include "foc_debug.h"
#include "bsp_timer.h"

// HALL传感器引脚定义
#define HALL_A_PIN GPIO_Pin_6
#define HALL_B_PIN GPIO_Pin_7
#define HALL_C_PIN GPIO_Pin_8
#define HALL_PORT GPIOC

typedef struct {
    float angle;          // 估计角度
    float speed;          // 估计速度
} hall_get;

void foc_encoder_init(void);
void hall_update_position_and_speed_PLL(uint32_t current_time);
extern inline void hall_update_PLL(hall_get* hall_data);
float calculate_motor_speed_rpm(void);

#endif /* __FOC_ENCODER_H */
