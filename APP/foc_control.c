#include "foc_control.h"

// 定义PI常量（使用DSP库精度更高的定义）
#define _PI         PI
#define _2PI        (2.0f * PI)

// 采集电压及母线电压结构体
extern foc_data_v foc_datav;

float32_t duty_a;
float32_t duty_b;
float32_t duty_c;

float32_t voltage_limit = VOLTAGE_AMPLITUDE_LIMIT;  // 电压限制百分比
float32_t max_voltage;  // 最大电压限制

// FOC控制变量
foc_control_t foc_ctrl;

// 任务句柄
TaskHandle_t xFOCControlTaskHandle = NULL;

void foc_control_init(void);
void foc_open_loop_control(void);

/**
 * @brief FOC控制任务
 */
void vFOCControlTask(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const float xFrequency = 10000.0f / CONTROL_LOOP_FREQ; // 控制频率

  // 初始化xLastWakeTime变量
  xLastWakeTime = xTaskGetTickCount();

  foc_set_open_loop_speed(1);

  for (;;)
    {

      debug_printf("%.4f,%.4f,%.4f", duty_a, duty_b, duty_c);

      // 按固定频率延迟
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
    }
}

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
 * @brief 角度归一化（0到2*PI）
 * @param angle 输入角度（可能超出范围）
 * @return 归一化后的角度
 */
static inline float32_t angle_normalize(float32_t angle)
{
  while (angle > _2PI) {
    angle -= _2PI;
  }
  while (angle < 0.0f) {
    angle += _2PI;
  }
  return angle;
}

/**
 * @brief FOC开环控制实现
 */
void foc_open_loop_control(void)
{
  static float angle_accum = 0.0f;  // 角度累计值

  // 将RPM转换为电角度速度（rad/s）
  // 电角度速度 = 2*PI * RPM/60 * 极对数
  foc_ctrl.speed = _2PI * foc_ctrl.target_speed / 60.0f * MOTOR_POLE_PAIRS;
  foc_ctrl.speed_rpm = foc_ctrl.target_speed;

  // 积分计算电角度 angle = angle + speed * dt(100us)
  angle_accum += foc_ctrl.speed * 0.0001f;

  // 归一化角度（0到2*PI）
  angle_accum = angle_normalize(angle_accum);
  foc_ctrl.angle = angle_accum;

  // 在开环控制中，直接设定DQ轴电压
  foc_ctrl.u_d = 0.0f;  // D轴电压设为0
  foc_ctrl.u_q = 0.6f;  // Q轴电压决定转矩，这里设为固定值

  // 限制电压幅值，使用DSP库的平方根函数
  float32_t voltage_magnitude;
  arm_status status = arm_sqrt_f32(foc_ctrl.u_d * foc_ctrl.u_d + foc_ctrl.u_q * foc_ctrl.u_q, &voltage_magnitude);
  // 限制最大输出电压
  arm_mult_f32(&foc_datav.vbus, &voltage_limit, &max_voltage, 1);
  
  // 若电压幅值超过最大值，按比例缩放
  if (status == ARM_MATH_SUCCESS && voltage_magnitude > max_voltage)
    {
      // 电压幅值限制（比例缩放）
      float32_t scale = max_voltage / voltage_magnitude;
      arm_scale_f32(&foc_ctrl.u_d, scale, &foc_ctrl.u_d, 1);
      arm_scale_f32(&foc_ctrl.u_q, scale, &foc_ctrl.u_q, 1);
    }

  // 反Park变换: dq -> alpha_beta
  float32_t sin_val, cos_val;
  arm_sin_cos_f32(foc_ctrl.angle * 180.0f / PI, &sin_val, &cos_val); // 转换为角度制
  float32_t alpha, beta;
  arm_inv_park_f32(foc_ctrl.u_d, foc_ctrl.u_q, &alpha, &beta, sin_val, cos_val);

  // 反Clark变换: alpha_beta -> abc
  float32_t a, b;
  arm_inv_clarke_f32(alpha, beta, &a, &b);
  float32_t c = -a - b; // 第三个相位电压

  // 电压转换为PWM占空比
  // 归一化电压到占空比 (0-1)
  duty_a = 0.5f + (a / foc_datav.vbus / 2.0f);
  duty_b = 0.5f + (b / foc_datav.vbus / 2.0f);
  duty_c = 0.5f + (c / foc_datav.vbus / 2.0f);

  // 限制占空比范围在0-1之间
  if (duty_a > 1.0f) duty_a = 1.0f;
  if (duty_a < 0.0f) duty_a = 0.0f;
  if (duty_b > 1.0f) duty_b = 1.0f;
  if (duty_b < 0.0f) duty_b = 0.0f;
  if (duty_c > 1.0f) duty_c = 1.0f;
  if (duty_c < 0.0f) duty_c = 0.0f;

  // 转换为定时器比较值
  uint16_t pwm_a = (uint16_t)(duty_a * PWM_PERIOD);
  uint16_t pwm_b = (uint16_t)(duty_b * PWM_PERIOD);
  uint16_t pwm_c = (uint16_t)(duty_c * PWM_PERIOD);

  // 设置PWM占空比
  // bsp_pwm_set_duty(pwm_a, pwm_b, pwm_c);
}
