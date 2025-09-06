#include "foc_control.h"

float32_t duty_a;
float32_t duty_b;
float32_t duty_c;

// FOC控制变量
foc_control_t foc_ctrl;
DQTypeDef current_dq;

// 电流PID控制器
static position_pid_t id_pid;
static position_pid_t iq_pid;

extern foc_data_i foc_datai;

extern position_pid_t position_pid;
static position_pid_t speed_pid;

extern float g_speed_rpm;
extern float g_total_angle;
extern float g_total_rotations;
extern float g_angle;

void foc_control_init(void);
void foc_open_loop_control(void);
void foc_position_control(void);
void abc_to_dq_current(foc_data_i *current_abc, DQTypeDef *current_dq, float angle);
void foc_control_set_params(float32_t* u_d_ptr, float32_t* u_q_ptr, float32_t* angle_ptr, float32_t* speed_ptr, float32_t* target_speed_ptr, float32_t* target_position_ptr);
void foc_current_pid_init(float32_t kp, float32_t ki, float32_t ki_limit);
void foc_speed_pid_init(float32_t kp, float32_t ki, float32_t ki_limit);

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
//   foc_ctrl.target_q = 1.0f;
//   float32_t target_position = 6.28f;  // 目标位置（弧度）

  // 初始化位置环PI参数
  // foc_position_pid_init(1.0f, 0.01f, 10.0f); // kp, ki, 积分限幅

  // 初始化电流环PI参数
  foc_current_pid_init(0.6f, 0.04f, 8.0f);

  // 初始化速度环PI参数
  foc_speed_pid_init(0.1f, 0.00001f, 2.0f);

  // 设置FOC控制初始参数
  foc_control_set_params(NULL, NULL, NULL, NULL, &new_target_speed, NULL);


  for (;;)
    {
    //   abc_to_dq_current(&foc_datai, &current_dq, foc_ctrl.angle);
      debug_printf("%.4f,%.4f,%.4f,%.4f", new_target_speed, current_dq.q, foc_ctrl.u_q,g_speed_rpm);
    //   debug_printf("%.4f,%.4f,%.4f,%.4f,%.4f", foc_ctrl.target_q, current_dq.d, current_dq.q, foc_ctrl.u_d, foc_ctrl.u_q);

      #if DEBUG_MODE
      // 最大合成的为母线电压的0.577倍
    //   debug_printf("%.4f,%.4f,%.4f", duty_a, duty_b, duty_c);
      // debug_printf("%.4f", foc_datav.vbus);
      #endif

      // 按固定频率延迟
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
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
 * @brief D轴电流环PID计算
 * @param target_id 目标D轴电流
 * @param actual_id 实际D轴电流
 * @return D轴电压输出
 */
float32_t foc_id_pid_calculate(float32_t target_id, float32_t actual_id)
{
    float32_t error = target_id - actual_id;
    float32_t p_term = id_pid.kp * error;
    
    // 积分项计算与限幅
    id_pid.integral += id_pid.ki * error;
    if (id_pid.integral > id_pid.integral_limit) {
        id_pid.integral = id_pid.integral_limit;
    } else if (id_pid.integral < -id_pid.integral_limit) {
        id_pid.integral = -id_pid.integral_limit;
    }
    
    return p_term + id_pid.integral;
}

/**
 * @brief Q轴电流环PID计算
 * @param target_iq 目标Q轴电流
 * @param actual_iq 实际Q轴电流
 * @return Q轴电压输出
 */
float32_t foc_iq_pid_calculate(float32_t target_iq, float32_t actual_iq)
{
    float32_t error = target_iq - actual_iq;
    float32_t p_term = iq_pid.kp * error;
    
    // 积分项计算与限幅
    iq_pid.integral += iq_pid.ki * error;
    if (iq_pid.integral > iq_pid.integral_limit) {
        iq_pid.integral = iq_pid.integral_limit;
    } else if (iq_pid.integral < -iq_pid.integral_limit) {
        iq_pid.integral = -iq_pid.integral_limit;
    }
    
    return p_term + iq_pid.integral;
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
 * @brief 将三相电流转换为dq坐标系下的电流值
 * @param current_abc 三相电流值 (ia, ib, ic)
 * @param current_dq 输出的dq轴电流值 (id, iq)
 * @param angle 电角度(弧度)
 */
void abc_to_dq_current(foc_data_i *current_abc, DQTypeDef *current_dq, float angle)
{
  ABCTypeDef abc_current;
  AlphaBetaTypeDef alpha_beta_current;
  
  // 将 foc_data_i 结构体转换为 ABCTypeDef 结构体
  abc_current.a = current_abc->ia;
  abc_current.b = current_abc->ib;
  abc_current.c = current_abc->ic;
  
  // 执行 Clarke 变换 (从三相坐标系转换到两相静止坐标系)
  clark_transform(&abc_current, &alpha_beta_current);
  
  // 执行 Park 变换 (从两相静止坐标系转换到两相旋转坐标系)
  park_transform(&alpha_beta_current, current_dq, angle);
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
 * @brief 速度环PID计算
 * @param target_speed 目标速度(RPM)
 * @param actual_speed 实际速度(RPM)
 * @return 输出值(Q轴电流)
 */
float32_t foc_speed_pid_calculate(float32_t target_speed, float32_t actual_speed)
{
    float32_t error = target_speed - actual_speed;
    float32_t p_term = speed_pid.kp * error;
    
    // 积分项计算与限幅
    speed_pid.integral += speed_pid.ki * error;
    if (speed_pid.integral > speed_pid.integral_limit) {
        speed_pid.integral = speed_pid.integral_limit;
    } else if (speed_pid.integral < -speed_pid.integral_limit) {
        speed_pid.integral = -speed_pid.integral_limit;
    }
    
    return p_term + speed_pid.integral;
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

/**
 * @brief FOC位置闭环控制函数
 * 该函数使用PI控制器实现精确位置控制
 */
void foc_position_control(void)
{
    // float32_t target_speed;
    float32_t current_position;
    float32_t T1, T2, T0;
    float32_t Ta, Tb, Tc;

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
    
    // // 使用PI控制器计算目标速度
    // target_speed = foc_position_pid_calculate(foc_ctrl.target_position, current_position);
    
    // // 限制目标速度范围
    // if (target_speed > MAX_SPEED_RPM * SPEED_FACTOR)
    // {
    //     target_speed = MAX_SPEED_RPM * SPEED_FACTOR;
    // }
    // else if (target_speed < -MAX_SPEED_RPM * SPEED_FACTOR)
    // {
    //     target_speed = -MAX_SPEED_RPM * SPEED_FACTOR;
    // }
    
    // // 设置目标速度
    // foc_ctrl.target_speed = target_speed;

    // debug_printf("%.2f,%.2f", current_position, foc_ctrl.u_q);

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    float32_t sin_theta, cos_theta;
    // 将电角度从弧度转换为角度
    float32_t angle_deg;
    arm_scale_f32(&foc_ctrl.angle, RAD_TO_DEG, &angle_deg, 1);
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    float32_t u_alpha, u_beta;
    // 反Park变换公式
    u_alpha = foc_ctrl.u_d * cos_theta - foc_ctrl.u_q * sin_theta;
    u_beta = foc_ctrl.u_d * sin_theta + foc_ctrl.u_q * cos_theta;

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
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
    float32_t target_id = 0.0f;          // 目标D轴电流（通常为0）
    float32_t target_iq = foc_ctrl.target_q;  // 目标Q轴电流（来自上层控制）
    float32_t u_d, u_q;                  // DQ轴电压输出
    float32_t u_alpha, u_beta;           // αβ轴电压
    float32_t T1, T2, T0;
    float32_t Ta, Tb, Tc;
    float32_t sin_theta, cos_theta;

    g_angle = hall_update_position_and_speed(bsp_get_micros());

    // 获取电角度(弧度值)
    foc_ctrl.angle = g_angle;

    // 将ABC坐标系电流转换为DQ坐标系电流
    abc_to_dq_current(&foc_datai, &current_dq, foc_ctrl.angle);

    // 电流环PID计算
    u_d = foc_id_pid_calculate(target_id, current_dq.d);
    u_q = foc_iq_pid_calculate(target_iq, current_dq.q);

    // 限制电压幅值，防止饱和
    float32_t voltage_magnitude = sqrtf(u_d * u_d + u_q * u_q);
    float32_t max_voltage = foc_datav.vbus * 1.0f; // 最大相电压
    if (voltage_magnitude > max_voltage && voltage_magnitude > 0.0f) {
        u_d = u_d * max_voltage / voltage_magnitude;
        u_q = u_q * max_voltage / voltage_magnitude;
    }

    // 保存输出电压
    foc_ctrl.u_d = u_d;
    foc_ctrl.u_q = u_q;

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    // 将电角度从弧度转换为角度
    float32_t angle_deg;
    arm_scale_f32(&foc_ctrl.angle, RAD_TO_DEG, &angle_deg, 1);
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    // 反Park变换公式
    u_alpha = foc_ctrl.u_d * cos_theta - foc_ctrl.u_q * sin_theta;
    u_beta = foc_ctrl.u_d * sin_theta + foc_ctrl.u_q * cos_theta;

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
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
    // float32_t T1, T2, T0;                // SVPWM作用时间（T1：基本矢量1，T2：基本矢量2，T0：零矢量）
    // float32_t Ta, Tb, Tc;                // 三相桥臂导通

    // 使用PI控制器计算目标Q轴电流
    target_current_q = foc_speed_pid_calculate(foc_ctrl.target_speed, g_speed_rpm);

    // 限制目标Q轴电流范围，防止饱和
    if (target_current_q > CURRENT_LIMIT) {
        target_current_q = CURRENT_LIMIT;
    } else if (target_current_q < -CURRENT_LIMIT) {
        target_current_q = -CURRENT_LIMIT;
    }

    // 设置目标Q轴电流
    foc_ctrl.target_q = target_current_q;

    // 调用电流环控制
    foc_current_control();

    // 如果需要直接输出PWM（跳过电流环），可以取消下面的注释并注释掉 foc_current_control() 调用
    /************************** 1. 反Park变换（DQ→αβ） **************************/
    // float32_t sin_theta, cos_theta;
    // // 使用角度制输入
    // float32_t angle_deg;
    // arm_scale_f32(&foc_ctrl.angle, RAD_TO_DEG, &angle_deg, 1);
    // arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    // // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    // float32_t u_alpha, u_beta;
    // // 反Park变换公式
    // u_alpha = foc_ctrl.u_d * cos_theta - foc_ctrl.u_q * sin_theta;
    // u_beta = foc_ctrl.u_d * sin_theta + foc_ctrl.u_q * cos_theta;

    // /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // // 扇区判断
    // int32_t current_sector = svpwm_sector_calc(u_alpha, u_beta);

    // // 计算基本矢量与零矢量作用时间
    // svpwm_calc_times(current_sector, u_alpha, u_beta, foc_datav.vbus, &T1, &T2, &T0);

    // float32_t T0_half = T0 / 2.0f;

    // // 按扇区计算三相导通时间
    // switch (current_sector)
    // {
    //     case 1:  // 扇区1
    //         Ta = T0_half + T1 + T2;
    //         Tb = T0_half + T2;
    //         Tc = T0_half;
    //         break;
    //     case 2:  // 扇区2
    //         Ta = T0_half + T2;
    //         Tb = T0_half + T1 + T2;
    //         Tc = T0_half;
    //         break;
    //     case 3:  // 扇区3
    //         Ta = T0_half;
    //         Tb = T0_half + T1 + T2;
    //         Tc = T0_half + T2;
    //         break;
    //     case 4:  // 扇区4
    //         Ta = T0_half;
    //         Tb = T0_half + T2;
    //         Tc = T0_half + T1 + T2;
    //         break;
    //     case 5:  // 扇区5
    //         Ta = T0_half + T2;
    //         Tb = T0_half;
    //         Tc = T0_half + T1 + T2;
    //         break;
    //     case 6:  // 扇区6
    //         Ta = T0_half + T1 + T2;
    //         Tb = T0_half;
    //         Tc = T0_half + T2;
    //         break;
    //     default:  // 异常扇区
    //         Ta = T0_half;
    //         Tb = T0_half;
    //         Tc = T0_half;
    //         break;
    // }
    // // 导通时间→占空比(此处需要将占空比除以2，因为定时器是中心对齐模式,算出来的Ta,Tb,Tc是两个PWM周期的时间下的有效时间)
    // duty_a = Ta * HALF_PWM_FREQ;
    // duty_b = Tb * HALF_PWM_FREQ;
    // duty_c = Tc * HALF_PWM_FREQ;

    // /************************** 4. 占空比→PWM定时器比较值 **************************/
    // uint16_t pwm_a = (uint16_t)(duty_a * PWM_PERIOD);
    // uint16_t pwm_b = (uint16_t)(duty_b * PWM_PERIOD);
    // uint16_t pwm_c = (uint16_t)(duty_c * PWM_PERIOD);

    // // 输出PWM到定时器
    // bsp_pwm_set_duty(pwm_a, pwm_b, pwm_c);
}
