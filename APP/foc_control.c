#include "foc_control.h"

// 定义PI常量
#define _PI 3.14159265359f
#define _2PI 6.28318530718f

// FOC控制变量
foc_control_t foc_ctrl;

// 任务句柄
TaskHandle_t xFOCControlTaskHandle = NULL;

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
 * @brief 设置开环速度
 * @param speed_rpm 目标速度 (RPM)
 */
void foc_set_open_loop_speed(float speed_rpm)
{
  foc_ctrl.target_speed = speed_rpm;
}

/**
 * @brief FOC开环控制实现
 */
void foc_open_loop_control(void)
{
  static uint32_t last_time = 0;
  static float angle_accum = 0.0f;  // 角度累计值

  uint32_t current_time = xTaskGetTickCount();
  float dt = (float)(current_time - last_time) / configTICK_RATE_HZ;
  last_time = current_time;

  // 限制dt，防止异常值
  if (dt <= 0.0f || dt > 0.1f)
    {
      dt = 1.0f / CONTROL_LOOP_FREQ;
    }

  // 将RPM转换为电角度速度（rad/s）
  // 电角度速度 = 2*PI * RPM/60 * 极对数
  foc_ctrl.speed = _2PI * foc_ctrl.target_speed / 60.0f * MOTOR_POLE_PAIRS;
  foc_ctrl.speed_rpm = foc_ctrl.target_speed;

  // 积分计算电角度
  angle_accum += foc_ctrl.speed * dt;

  // 保持角度在0到2*PI之间
  while (angle_accum >= _2PI)
    {
      angle_accum -= _2PI;
    }
  while (angle_accum < 0.0f)
    {
      angle_accum += _2PI;
    }

  foc_ctrl.angle = angle_accum;

  // 在开环控制中，直接设定DQ轴电压
  foc_ctrl.u_d = 0.0f;  // D轴电压设为0
  foc_ctrl.u_q = 1.0f;  // Q轴电压决定转矩，这里设为固定值

  // 限制电压幅值，防止超过母线电压
  float voltage_magnitude = sqrtf(foc_ctrl.u_d * foc_ctrl.u_d + foc_ctrl.u_q * foc_ctrl.u_q);
  float max_voltage = VOLTAGE_LIMIT * VOLTAGE_AMPLITUDE_LIMIT; // 最大电压设为母线电压的60%

  if (voltage_magnitude > max_voltage && voltage_magnitude > 0.0f)
    {
      foc_ctrl.u_d = foc_ctrl.u_d * max_voltage / voltage_magnitude;
      foc_ctrl.u_q = foc_ctrl.u_q * max_voltage / voltage_magnitude;
    }

  // 反Park变换: dq -> alpha_beta
  DQTypeDef dq;
  AlphaBetaTypeDef alpha_beta;

  dq.d = foc_ctrl.u_d;
  dq.q = foc_ctrl.u_q;

  inv_park_transform(&dq, &alpha_beta, foc_ctrl.angle);

  // 反Clark变换: alpha_beta -> abc
  ABCTypeDef abc;
  inv_clark_transform(&alpha_beta, &abc);

  // 电压转换为PWM占空比
  // 归一化电压到占空比 (0-1)
  float duty_a = 0.5f + (abc.a / VOLTAGE_LIMIT / 2.0f);
  float duty_b = 0.5f + (abc.b / VOLTAGE_LIMIT / 2.0f);
  float duty_c = 0.5f + (abc.c / VOLTAGE_LIMIT / 2.0f);

  // 限制占空比范围在0-1之间
  duty_a = (duty_a < 0.0f) ? 0.0f : ((duty_a > 1.0f) ? 1.0f : duty_a);
  duty_b = (duty_b < 0.0f) ? 0.0f : ((duty_b > 1.0f) ? 1.0f : duty_b);
  duty_c = (duty_c < 0.0f) ? 0.0f : ((duty_c > 1.0f) ? 1.0f : duty_c);

  // 转换为定时器比较值
  uint16_t pwm_a = (uint16_t)(duty_a * PWM_PERIOD);
  uint16_t pwm_b = (uint16_t)(duty_b * PWM_PERIOD);
  uint16_t pwm_c = (uint16_t)(duty_c * PWM_PERIOD);

//   debug_printf("%.4f,%.4f,%.4f", duty_a, duty_b, duty_c);
  // debug_printf("%.4f",angle_accum);
  // debug_printf("%.4f",dt * 1000.0f);

  // 设置PWM占空比
  // bsp_pwm_set_duty(pwm_a, pwm_b, pwm_c);
}

/**
 * @brief FOC控制任务
 */
void vFOCControlTask(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const float xFrequency = 10000.0f / CONTROL_LOOP_FREQ; // 控制频率

  // 初始化xLastWakeTime变量
  xLastWakeTime = xTaskGetTickCount();

  foc_set_open_loop_speed(10);

  for (;;)
    {
      // 执行开环控制
      foc_open_loop_control();

      // 按固定频率延迟
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
    }
}
