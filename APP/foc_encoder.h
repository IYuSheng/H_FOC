#ifndef __FOC_ENCODER_H
#define __FOC_ENCODER_H

#include "stm32f4xx.h"
#include "arm_math.h"
#include "Config.h"
#include "bsp_timer.h"

// HALL传感器引脚定义
#define HALL_A_PIN GPIO_Pin_6
#define HALL_B_PIN GPIO_Pin_7
#define HALL_C_PIN GPIO_Pin_8
#define HALL_PORT GPIOC

#define PLL_HALL_FBW        6                           // 带宽频率(HZ)
#define PLL_HALL_WN         (2 * PI * PLL_HALL_FBW)     // 带宽(rad/s)
#define PLL_HALL_KP         (2 * 0.707 * PLL_HALL_WN)   // PLL比例增益
#define PLL_HALL_KI         (PLL_HALL_WN * PLL_HALL_WN) // PLL积分增益
#define MAX_PLL_SPEED       1000.0f // PLL最大速度限幅 (rad/s)

typedef struct {
    float angle;          // 估计电角度 [0, 2π)
    float speed;          // 估计电角速度 (rad/s)，带符号
    float elec_speed;     // 电角速度 (rad/s)
    int8_t direction;     // 方向: 1正转, -1反转, 0停止
} hall_get_t;

extern hall_get_t hall_data;

// 编码器接口函数
void foc_encoder_init(void);
void hall_update_PLL(hall_get_t* hall_data);
float calculate_motor_speed_rpm(hall_get_t* hall_data);
int8_t hall_get_direction(void);

#endif /* __FOC_ENCODER_H */
