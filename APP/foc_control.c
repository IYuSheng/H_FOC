#include "foc_control.h"

// FOC控制变量
foc_control_t foc_ctrl;
AlphaBetaTypeDef alpha_beta;
SVPWM_t svpwm;

/**
 * @brief FOC控制任务
 */
void vFOCControlTask(void *pvParameters)
{
  // 140大约5码
  foc_ctrl.target_speed = 240.0f;   // 设置目标速度
  foc_ctrl.out_q = 1.5f; // 设置q轴输出电压(开环用)
  foc_ctrl.target_q = 0.3f;  // 设置目标Q轴电流
  foc_ctrl.target_position = 6.28f;  // 设置目标位置

  // 初始化电流环PI参数
  foc_current_pid_init(I_P_GAIN, I_I_GAIN, I_I_LIMIT);

  // 初始化速度环PI参数
  foc_speed_pid_init(SPEED_P_GAIN, SPEED_I_GAIN, SPEED_I_LIMIT);

  // 初始化位置环PI参数
  foc_position_pid_init(POSITION_P_GAIN, POSITION_I_GAIN, POSITION_I_LIMIT);
  for (;;)
    {
    #if DEBUG_MODE
    debug_printf("%.4f,%.4f,%.4f,%.4f,%.4f", foc_ctrl.target_q, foc_ctrl.abc_dq.current_q, foc_ctrl.abc_dq.current_d, foc_ctrl.out_q, foc_ctrl.out_d);
    // debug_printf("%.4f,%.4f", bemf_angle, g_angle);
    // debug_printf("%.4f, %.4f", Angle_SMOPare.Ealpha, bemf_angle);
    // debug_printf("%.4f,%.4f", g_speed_rpm, bemf_speed);
    // debug_printf("%.4f", foc_datav.vbus);
    // debug_printf("%d,%d,%d", svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    // debug_printf("%.4f", g_speed_rpm);
    #endif

      // 按固定频率延迟
      vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief FOC速度开环控制
 */
void foc_open_loop_control(void)
{
    static float32_t angle_accum = 0.0f;  // 电角度累加器（保持积分状态）
    
    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（rad/s）：ω_e = 2π * (n_rpm / 60) * 极对数
    arm_scale_f32(&foc_ctrl.target_speed, SPEED_FACTOR, &foc_ctrl.speed, 1);

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize(angle_accum);
    foc_ctrl.angle = angle_accum;

    /************************** 2. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, foc_ctrl.angle);

    /************************** 3. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, foc_datav.vbus);

    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器
    bsp_pwm_set_duty(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);

    // 更新无感估算电角度(在开环测试用)
    bemf_angle = SMO_bemf_angle(&alpha_beta);
}

/**
 * @brief 电流闭环控制
 */
void foc_current_control(void)
{
    // 将ABC坐标系采集电压电流转换为alpha-beta坐标系电压电流
    clark_transform(&foc_datai,&foc_datav, &alpha_beta);

    // 滑模观察器估算电角度
    bemf_angle = SMO_bemf_angle(&alpha_beta);

    // 有感无感切换
    // static uint32_t countsss = 0;
    // if(countsss < 10000)
    // {
    //     countsss++;
        foc_ctrl.angle = g_angle;
    // }
    // else
    // {
    //     foc_ctrl.angle = bemf_angle;
    // }

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl);

    // 电流环PID计算
    foc_ctrl.out_d = foc_id_pid_calculate(foc_ctrl.target_d, foc_ctrl.abc_dq.current_d);
    foc_ctrl.out_q = foc_iq_pid_calculate(foc_ctrl.target_q, foc_ctrl.abc_dq.current_q);

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, foc_ctrl.angle);

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, foc_datav.vbus);

    // 计算得到PWM定时器比较值
    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器 
    bsp_pwm_set_duty(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

/**
 * @brief FOC速度闭环控制函数
 * 该函数使用PI控制器实现精确速度控制
 */
void foc_speed_control(void)
{
    float32_t target_current_q;          // 目标Q轴电流

    static volatile uint32_t count_cycle = 0;
    // static uint32_t count = 0;
    // if(count < 20000)
    // {
    //     count++;
        // 获取电机编码器速度
        foc_ctrl.speed = g_speed_rpm;
    // }
    // else
    // {
    //     foc_ctrl.speed = bemf_speed;
    // }

    // 使用固定频率更新速度环
    count_cycle++;
    if(count_cycle >= SPEED_Cycle)
    {
        count_cycle = 0;
        // 使用PI控制器计算目标Q轴电流
        target_current_q = foc_speed_pid_calculate(foc_ctrl.target_speed, foc_ctrl.speed);

        // 设置目标Q轴电流
        foc_ctrl.target_q = target_current_q;
    }

    // 调用电流环控制
    foc_current_control();
}

/**
 * @brief FOC位置闭环控制函数
 * 该函数使用PI控制器实现精确位置控制
 */
void foc_position_control(void)
{
    // 获取当前位置（机械弧度值角度）
    float32_t current_position = g_total_angle;
    // 获取电角度(弧度值)
    foc_ctrl.angle = g_angle;

    // 使用PI控制器计算目标力矩
    foc_ctrl.out_q = foc_position_pid_calculate(foc_ctrl.target_position, current_position) * _2_PI;

    // 限制Uq范围，防止过大
    if (foc_ctrl.out_q > 2.0f)
    {
        foc_ctrl.out_q = 2.0f;
    }
    else if (foc_ctrl.out_q < -2.0f)
    {
        foc_ctrl.out_q = -2.0f;
    }

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, foc_ctrl.angle);

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, foc_datav.vbus);

    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器
    bsp_pwm_set_duty(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}
