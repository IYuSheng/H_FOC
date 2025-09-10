#ifndef __FOC_SETTINGS_H
#define __FOC_SETTINGS_H

#include "foc_control.h"

void foc_control_init(void);
void foc_control_set_params(float32_t* u_d_ptr, float32_t* u_q_ptr, float32_t* angle_ptr, float32_t* speed_ptr, float32_t* target_speed_ptr,float32_t* target_position_ptr);
void foc_speed_pid_init(float32_t kp, float32_t ki, float32_t ki_limit);
void foc_current_pid_init(float32_t kp, float32_t ki, float32_t ki_limit);
void foc_position_pid_init(float32_t kp, float32_t ki, float32_t integral_limit);

#endif /* __FOC_SETTINGS_H */
