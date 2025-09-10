#include "foc_settings.h"

extern foc_control_t foc_ctrl;
extern position_pid_t id_pid;
extern position_pid_t iq_pid;
extern position_pid_t speed_pid;
extern position_pid_t position_pid;

/**
 * @brief FOC控制初始化
 */
void foc_control_init(void)
{
  // 初始化控制参数
  foc_ctrl.u_d = 0.0f;
  foc_ctrl.u_q = 0.0f;
  foc_ctrl.angle = 0.0f;
  foc_ctrl.speed = 0.0f;
  foc_ctrl.speed_rpm = 0.0f;
  foc_ctrl.target_speed = 0.0f;
}

/**
 * @brief 设置FOC控制参数（选择性设置）
 * @param u_d_ptr D轴电压指针，若为NULL则不改变该项
 * @param u_q_ptr Q轴电压指针，若为NULL则不改变该项
 * @param angle_ptr 电角度指针，若为NULL则不改变该项
 * @param speed_ptr 速度指针，若为NULL则不改变该项
 * @param target_speed_ptr 目标速度指针，若为NULL则不改变该项
 */
void foc_control_set_params(float32_t* u_d_ptr, 
                            float32_t* u_q_ptr, 
                            float32_t* angle_ptr, 
                            float32_t* speed_ptr, 
                            float32_t* target_speed_ptr,
                            float32_t* target_position_ptr)
{
  if (u_d_ptr != NULL) {
    foc_ctrl.u_d = *u_d_ptr;
  }
  
  if (u_q_ptr != NULL) {
    foc_ctrl.u_q = *u_q_ptr;
  }
  
  if (angle_ptr != NULL) {
    foc_ctrl.angle = *angle_ptr;
  }
  
  if (speed_ptr != NULL) {
    foc_ctrl.speed = *speed_ptr;
  }
  
  if (target_speed_ptr != NULL) {
    foc_ctrl.target_speed = *target_speed_ptr;
  }

  if(target_position_ptr != NULL)
  {
    foc_ctrl.target_position = *target_position_ptr;
  }
}

/**
 * @brief 电流环PID控制器初始化
 * @param kp 比例系数
 * @param ki 积分系数
 * @param ki_limit 积分限幅值
 */
void foc_current_pid_init(float32_t kp, float32_t ki, float32_t ki_limit)
{
    // 初始化D轴电流环PID参数
    id_pid.kp = kp;
    id_pid.ki = ki;
    id_pid.integral_limit = ki_limit;
    id_pid.integral = 0.0f;
    
    // 初始化Q轴电流环PID参数
    iq_pid.kp = kp;
    iq_pid.ki = ki;
    iq_pid.integral_limit = ki_limit;
    iq_pid.integral = 0.0f;
}

/**
 * @brief 速度环PID控制器初始化
 * @param kp 比例系数
 * @param ki 积分系数
 * @param ki_limit 积分限幅值
 */
void foc_speed_pid_init(float32_t kp, float32_t ki, float32_t ki_limit)
{
    // 初始化速度环PID参数
    speed_pid.kp = kp;
    speed_pid.ki = ki;
    speed_pid.integral_limit = ki_limit;
    speed_pid.integral = 0.0f;
}

/**
 * @brief 初始化位置环PI控制器
 * @param kp 比例增益
 * @param ki 积分增益
 * @param integral_limit 积分限幅值
 */
void foc_position_pid_init(float32_t kp, float32_t ki, float32_t integral_limit)
{
    position_pid.kp = kp;
    position_pid.ki = ki;
    position_pid.integral_limit = integral_limit;
    position_pid.integral = 0.0f;
    position_pid.error = 0.0f;
    position_pid.output = 0.0f;
    position_pid.target = 0.0f;
    position_pid.current = 0.0f;
}
