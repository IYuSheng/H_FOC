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
  foc_ctrl.out_d = 0.0f;
  foc_ctrl.out_q = 0.0f;
  foc_ctrl.angle = 0.0f;
  foc_ctrl.speed = 0.0f;
  foc_ctrl.speed_rpm = 0.0f;
  foc_ctrl.target_speed = 0.0f;
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
