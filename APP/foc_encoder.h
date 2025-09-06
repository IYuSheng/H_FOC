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

// 函数声明
void foc_encoder_init(void);
uint8_t hall_sensor_read(void);
float hall_get_electrical_angle_rad(uint8_t hall_state);
float hall_update_position_and_speed(uint32_t current_time);
float hall_get_speed_rad_per_sec(void);
float hall_get_speed_rpm(void);
float hall_get_mechanical_angle(void);
float hall_get_total_mechanical_angle(void);
float hall_get_total_rotations(void);
void hall_reset_counts(void);

#endif /* __FOC_ENCODER_H */
