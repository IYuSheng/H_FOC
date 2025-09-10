#include "foc_control.h"

float32_t duty_a;
float32_t duty_b;
float32_t duty_c;

// FOC控制变量
foc_control_t foc_ctrl;
DQTypeDef current_dq;

/**
 * @brief FOC控制任务
 */
void vFOCControlTask(void *pvParameters)
{
  TickType_t xLastWakeTime;

  // 初始化xLastWakeTime变量
  xLastWakeTime = xTaskGetTickCount();

  // 140大约5码
  float32_t new_target_speed = 130.0f;
  foc_ctrl.target_q = 4.0f;
  foc_ctrl.u_q = 6.0f;
//   float32_t target_position = 6.28f;  // 目标位置（弧度）

  // 初始化位置环PI参数
  // foc_position_pid_init(1.0f, 0.01f, 10.0f); // kp, ki, 积分限幅

  // 初始化电流环PI参数
  foc_current_pid_init(0.6f, 0.04f, 6.0f);

  // 初始化速度环PI参数
  foc_speed_pid_init(1.0f, 0.005f, 1.0f);

  // 设置FOC控制初始参数
  foc_control_set_params(NULL, NULL, NULL, NULL, &new_target_speed, NULL);

  for (;;)
    {
    #if DEBUG_MODE

    // debug_printf("%.4f,%.4f,%.4f,%.4f,%.4f", foc_ctrl.target_q, current_dq.d, current_dq.q, foc_ctrl.u_d, foc_ctrl.u_q);
    debug_printf("%.4f,%.4f,%.4f", bemf_angle, g_angle, bemf_speed);
    // debug_printf("%.4f,%.4f,%.4f", duty_a, duty_b, duty_c);
    // debug_printf("%.4f", foc_datav.vbus);
    #endif

      // 按固定频率延迟
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}


/**
 * @brief FOC开环控制核心（SVPWM调制实现）
 * 流程：开环电角度计算 → DQ电压设定 → 电压限制 → 反Park变换 → SVPWM（扇区→时间→占空比）
 */
void foc_open_loop_control(void)
{
    static float32_t angle_accum = 0.0f;  // 电角度累加器（保持积分状态）
    float32_t u_alpha, u_beta;            // αβ轴电压
    float32_t T1, T2, T0;                 // SVPWM作用时间（T1：基本矢量1，T2：基本矢量2，T0：零矢量）
    

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（rad/s）：ω_e = 2π * (n_rpm / 60) * 极对数
    arm_scale_f32(&foc_ctrl.target_speed, SPEED_FACTOR, &foc_ctrl.speed, 1);

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize(angle_accum);
    foc_ctrl.angle = angle_accum;

    /************************** 2. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl.u_d, &foc_ctrl.u_q, &u_alpha, &u_beta, foc_ctrl.angle);

    /************************** 3. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    int32_t current_sector = svpwm_sector_calc(u_alpha, u_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(current_sector, u_alpha, u_beta, foc_datav.vbus, &T1, &T2, &T0);

    svpwm_duty_calc(current_sector, T1, T2, T0, &duty_a, &duty_b, &duty_c);

    /************************** 4. 占空比→PWM定时器比较值 **************************/
    uint16_t pwm_a = (uint16_t)(duty_a * PWM_PERIOD);
    uint16_t pwm_b = (uint16_t)(duty_b * PWM_PERIOD);
    uint16_t pwm_c = (uint16_t)(duty_c * PWM_PERIOD);

    // 输出PWM到定时器
    bsp_pwm_set_duty(pwm_a, pwm_b, pwm_c);
}

/**
 * @brief FOC位置闭环控制函数
 * 该函数使用PI控制器实现精确位置控制
 */
void foc_position_control(void)
{
    float32_t u_alpha, u_beta;
    float32_t current_position;
    float32_t T1, T2, T0;

    // 获取电角度(弧度值)
    foc_ctrl.angle = g_angle;

    // 获取当前位置（机械弧度值角度）
    current_position = g_total_angle;

    // 使用PI控制器计算目标力矩
    foc_ctrl.u_q = foc_position_pid_calculate(foc_ctrl.target_position, current_position) * _2_PI;

    // 限制Uq范围，防止过大
    if (foc_ctrl.u_q > 2.0f)
    {
        foc_ctrl.u_q = 2.0f;
    }
    else if (foc_ctrl.u_q < -2.0f)
    {
        foc_ctrl.u_q = -2.0f;
    }

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl.u_d, &foc_ctrl.u_q, &u_alpha, &u_beta, foc_ctrl.angle);

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    int32_t current_sector = svpwm_sector_calc(u_alpha, u_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(current_sector, u_alpha, u_beta, foc_datav.vbus, &T1, &T2, &T0);

    svpwm_duty_calc(current_sector, T1, T2, T0, &duty_a, &duty_b, &duty_c);

    /************************** 3. 占空比→PWM定时器比较值 **************************/
    uint16_t pwm_a = (uint16_t)(duty_a * PWM_PERIOD);
    uint16_t pwm_b = (uint16_t)(duty_b * PWM_PERIOD);
    uint16_t pwm_c = (uint16_t)(duty_c * PWM_PERIOD);

    // 输出PWM到定时器
    bsp_pwm_set_duty(pwm_a, pwm_b, pwm_c);
}

/**
 * @brief 电流闭环控制
 */
void foc_current_control(void)
{
    float32_t target_id = 0.0f;          // 目标D轴电流
    float32_t target_iq = foc_ctrl.target_q;  // 目标Q轴电流（来自上层控制）
    float32_t u_d, u_q;                  // DQ轴电压输出
    float32_t u_alpha, u_beta;           // αβ轴电压
    int32_t current_sector;              // 当前SVPWM扇区
    float32_t T1, T2, T0;

    static uint32_t countsss = 0;
    
    bemf_angle = bsp_adc_calculate_bemf_angle(
          foc_datai.ia, foc_datai.ib, foc_datai.ic,
          foc_datav.va, foc_datav.vb, foc_datav.vc,
          bsp_get_micros());             // 获取反电动势电角度

    if(countsss < 2000)
    {
        countsss++;
        foc_ctrl.angle = g_angle;
    }
    else
    {
        foc_ctrl.angle = bemf_angle;
    }

    // 将ABC坐标系电流转换为DQ坐标系电流
    abc_to_dq_current((void*)&foc_datai, &current_dq, foc_ctrl.angle);

    // 电流环PID计算
    u_d = foc_id_pid_calculate(target_id, current_dq.d);
    u_q = foc_iq_pid_calculate(target_iq, current_dq.q);

    // 保存输出电压
    foc_ctrl.u_d = u_d;
    foc_ctrl.u_q = u_q;

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl.u_d, &foc_ctrl.u_q, &u_alpha, &u_beta, foc_ctrl.angle);

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    current_sector = svpwm_sector_calc(u_alpha, u_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(current_sector, u_alpha, u_beta, foc_datav.vbus, &T1, &T2, &T0);

    // 计算占空比
    svpwm_duty_calc(current_sector, T1, T2, T0, &duty_a, &duty_b, &duty_c);

    /************************** 3. 占空比→PWM定时器比较值 **************************/
    uint16_t pwm_a = (uint16_t)(duty_a * PWM_PERIOD);
    uint16_t pwm_b = (uint16_t)(duty_b * PWM_PERIOD);
    uint16_t pwm_c = (uint16_t)(duty_c * PWM_PERIOD);

    // 输出PWM到定时器
    bsp_pwm_set_duty(pwm_a, pwm_b, pwm_c);
}

/**
 * @brief FOC速度闭环控制函数
 * 该函数使用PI控制器实现精确速度控制
 */
void foc_speed_control(void)
{
    float32_t target_current_q;          // 目标Q轴电流

    static volatile uint32_t countsb = 0;
    static uint32_t countss = 0;
    
    
    if(countss < 2000)
    {
        countss++;
        // 获取电机编码器速度
        foc_ctrl.speed = g_speed_rpm;
    }
    else
    {
        foc_ctrl.speed = bemf_speed;
    }
    countsb++;
    if(countsb >= 100)
    {
        countsb = 0;
        // 使用PI控制器计算目标Q轴电流
        target_current_q = foc_speed_pid_calculate(foc_ctrl.target_speed, foc_ctrl.speed);
    
        // 限制目标Q轴电流范围，防止饱和
        if (target_current_q > CURRENT_LIMIT) {
            target_current_q = CURRENT_LIMIT;
        } else if (target_current_q < -CURRENT_LIMIT) {
            target_current_q = -CURRENT_LIMIT;
        }

        // 设置目标Q轴电流
        foc_ctrl.target_q = target_current_q;
    }

    // 调用电流环控制
    foc_current_control();
}
