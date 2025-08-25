#include "foc_control.h"

float32_t duty_a;
float32_t duty_b;
float32_t duty_c;

// FOC控制变量
foc_control_t foc_ctrl;

void foc_control_init(void);
void foc_open_loop_control(void);
void foc_control_set_params(float32_t* u_d_ptr, float32_t* u_q_ptr, float32_t* angle_ptr, float32_t* speed_ptr, float32_t* target_speed_ptr);

/**
 * @brief FOC控制任务
 */
void vFOCControlTask(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const float xFrequency = 10000.0f / PWM_FREQ; // 控制频率

  // 初始化xLastWakeTime变量
  xLastWakeTime = xTaskGetTickCount();

  float32_t new_target_speed = 2.0f;
  float32_t ud = 0.0f;
  float32_t uq = 1.0f;
  foc_control_set_params(&ud, &uq, NULL, NULL, &new_target_speed);

  for (;;)
    {

      #if DEBUG_MODE
      // 最大合成的为母线电压的0.577倍
      debug_printf("%.4f,%.4f,%.4f", duty_a * foc_datav.vbus, duty_b * foc_datav.vbus, duty_c * foc_datav.vbus);
      // debug_printf("%.4f", foc_datav.vbus);
      #endif

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
                            float32_t* target_speed_ptr)
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
}

/**
 * @brief FOC开环控制核心（SVPWM调制实现）
 * 流程：开环电角度计算 → DQ电压设定 → 电压限制 → 反Park变换 → SVPWM（扇区→时间→占空比）
 */
void foc_open_loop_control(void)
{
    static float32_t angle_accum = 0.0f;  // 电角度累加器（保持积分状态）
    float32_t T1, T2, T0;                 // SVPWM作用时间（T1：基本矢量1，T2：基本矢量2，T0：零矢量）
    float32_t Ta, Tb, Tc;                 // 三相桥臂导通时间（s）

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（rad/s）：ω_e = 2π * (n_rpm / 60) * 极对数
    arm_scale_f32(&foc_ctrl.target_speed, SPEED_FACTOR, &foc_ctrl.speed, 1);

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize(angle_accum);
    foc_ctrl.angle = angle_accum;

    /************************** 2. 反Park变换（DQ→αβ） **************************/
    float32_t sin_theta, cos_theta;
    // 使用角度制输入
    float32_t angle_deg;
    arm_scale_f32(&foc_ctrl.angle, RAD_TO_DEG, &angle_deg, 1);
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    float32_t sin_val, cos_val;
    arm_sin_cos_f32(foc_ctrl.angle * RAD_TO_DEG, &sin_val, &cos_val); // 转换为角度制
    float32_t u_alpha, u_beta;
    // 反Park变换公式
    u_alpha = foc_ctrl.u_d * cos_theta - foc_ctrl.u_q * sin_theta;
    u_beta = foc_ctrl.u_d * sin_theta + foc_ctrl.u_q * cos_theta;

    /************************** 3. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    int32_t current_sector = svpwm_sector_calc(u_alpha, u_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(current_sector, u_alpha, u_beta, foc_datav.vbus, &T1, &T2, &T0);

    float32_t T0_half = T0 / 2.0f;

    // 按扇区计算三相导通时间
    switch (current_sector)
    {
        case 1:  // 扇区1
            Ta = T0_half + T1 + T2;
            Tb = T0_half + T2;
            Tc = T0_half;
            break;
        case 2:  // 扇区2
            Ta = T0_half + T2;
            Tb = T0_half + T1 + T2;
            Tc = T0_half;
            break;
        case 3:  // 扇区3
            Ta = T0_half;
            Tb = T0_half + T1 + T2;
            Tc = T0_half + T2;
            break;
        case 4:  // 扇区4
            Ta = T0_half;
            Tb = T0_half + T2;
            Tc = T0_half + T1 + T2;
            break;
        case 5:  // 扇区5
            Ta = T0_half + T2;
            Tb = T0_half;
            Tc = T0_half + T1 + T2;
            break;
        case 6:  // 扇区6
            Ta = T0_half + T1 + T2;
            Tb = T0_half;
            Tc = T0_half + T2;
            break;
        default:  // 异常扇区
            Ta = T0_half;
            Tb = T0_half;
            Tc = T0_half;
            break;
    }
    // 导通时间→占空比(此处需要将占空比除以2，因为定时器是中心对齐模式,算出来的Ta,Tb,Tc是两个PWM周期的时间下的有效时间)
    duty_a = Ta * HALF_PWM_FREQ;
    duty_b = Tb * HALF_PWM_FREQ;
    duty_c = Tc * HALF_PWM_FREQ;

    /************************** 4. 占空比→PWM定时器比较值 **************************/
    uint16_t pwm_a = (uint16_t)(duty_a * PWM_PERIOD);
    uint16_t pwm_b = (uint16_t)(duty_b * PWM_PERIOD);
    uint16_t pwm_c = (uint16_t)(duty_c * PWM_PERIOD);

    // 输出PWM到定时器
    bsp_pwm_set_duty(pwm_a, pwm_b, pwm_c);
}
