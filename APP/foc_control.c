#include "foc_control.h"

// 采集电压及母线电压结构体
extern foc_data_v foc_datav;

float32_t duty_a;
float32_t duty_b;
float32_t duty_c;

// FOC控制变量
foc_control_t foc_ctrl;

// 任务句柄
TaskHandle_t xFOCControlTaskHandle = NULL;

void foc_control_init(void);
void foc_open_loop_control(void);
void foc_control_set_params(float32_t* u_d_ptr, float32_t* u_q_ptr, float32_t* angle_ptr, float32_t* speed_ptr, float32_t* target_speed_ptr);

/**
 * @brief FOC控制任务
 */
void vFOCControlTask(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const float xFrequency = 10000.0f / CONTROL_LOOP_FREQ; // 控制频率

  // 初始化xLastWakeTime变量
  xLastWakeTime = xTaskGetTickCount();

  float32_t new_target_speed = 5.0f;
  foc_control_set_params(NULL, NULL, NULL, NULL, &new_target_speed);

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
    float32_t speed_factor = _2PI / 60.0f * MOTOR_POLE_PAIRS;
    arm_scale_f32(&foc_ctrl.target_speed, speed_factor, &foc_ctrl.speed, 1);
    foc_ctrl.speed_rpm = foc_ctrl.target_speed;

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    float32_t increment;
    arm_mult_f32(&foc_ctrl.speed, &ctrl_period, &increment, 1);
    arm_add_f32(&angle_accum, &increment, &angle_accum, 1);
    angle_accum = angle_normalize(angle_accum);  // 归一化到0~2π
    foc_ctrl.angle = angle_accum;

    /************************** 2. DQ轴电压设定 **************************/
    foc_ctrl.u_d = 0.0f;           // D轴电压=0
    foc_ctrl.u_q = 1.0f;           // Q轴电压

    /************************** 3. 反Park变换（DQ→αβ） **************************/
    float32_t sin_theta, cos_theta;
    // 使用角度制输入
    float32_t angle_deg;
    arm_scale_f32(&foc_ctrl.angle, 180.0f / PI, &angle_deg, 1);
    arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

    // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    float32_t u_alpha, u_beta;
    arm_inv_park_f32(foc_ctrl.u_d, foc_ctrl.u_q, &u_alpha, &u_beta, sin_theta, cos_theta);

    /************************** 4. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    int32_t current_sector = svpwm_sector_calc(u_alpha, u_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(current_sector, u_alpha, u_beta, foc_datav.vbus, &T1, &T2, &T0);

    float32_t T7_half = T0 / 2.0f;

    // 按扇区计算三相导通时间
    switch (current_sector)
    {
        case 1:  // 扇区1：基本矢量V4(100)→V6(110)，导通顺序：A上→B上→A下→B下3
            arm_add_f32(&T7_half, &T1, &Ta, 1);
            arm_add_f32(&Ta, &T2, &Ta, 1);
            arm_add_f32(&T7_half, &T2, &Tb, 1);
            Tc = T7_half;
            break;
        case 2:  // 扇区2：基本矢量V6(110)→V2(010)，导通顺序：B上→A下→B下→A上1
            arm_add_f32(&T7_half, &T2, &Ta, 1);
            arm_add_f32(&T7_half, &T1, &Tb, 1);
            arm_add_f32(&Tb, &T2, &Tb, 1);
            Tc = T7_half;
            break;
        case 3:  // 扇区3：基本矢量V2(010)→V3(011)，导通顺序：B上→C上→B下→C下5
            Ta = T7_half;
            arm_add_f32(&T7_half, &T1, &Tb, 1);
            arm_add_f32(&Tb, &T2, &Tb, 1);
            arm_add_f32(&T7_half, &T2, &Tc, 1);
            break;
        case 4:  // 扇区4：基本矢量V3(011)→V1(001)，导通顺序：C上→B下→C下→B上4
            Ta = T7_half;
            arm_add_f32(&T7_half, &T2, &Tb, 1);
            arm_add_f32(&T7_half, &T1, &Tc, 1);
            arm_add_f32(&Tc, &T2, &Tc, 1);
            break;
        case 5:  // 扇区5：基本矢量V1(001)→V5(101)，导通顺序：C上→A上→C下→A下6
            arm_add_f32(&T7_half, &T2, &Ta, 1);
            Tb = T7_half;
            arm_add_f32(&T7_half, &T1, &Tc, 1);
            arm_add_f32(&Tc, &T2, &Tc, 1);
            break;
        case 6:  // 扇区6：基本矢量V5(101)→V4(100)，导通顺序：A上→C下→A下→C上2
            arm_add_f32(&T7_half, &T1, &Ta, 1);
            arm_add_f32(&Ta, &T2, &Ta, 1);
            Tb = T7_half;
            arm_add_f32(&T7_half, &T2, &Tc, 1);
            break;
        default:  // 异常扇区：全用零矢量（避免电机失控）
            Ta = T7_half;
            Tb = T7_half;
            Tc = T7_half;
            break;
    }

    // 导通时间→占空比（占空比 = 导通时间 / PWM周期）
    float32_t freq = PWM_FREQ;
    arm_mult_f32(&Ta, &freq, &duty_a, 1);
    arm_mult_f32(&Tb, &freq, &duty_b, 1);
    arm_mult_f32(&Tc, &freq, &duty_c, 1);

    /************************** 5. 占空比限幅（避免PWM输出异常） **************************/
    // 限制占空比在0~1范围（超出会导致桥臂直通或PWM无输出）
    duty_a = (duty_a > 1.0f) ? 1.0f : ((duty_a < 0.0f) ? 0.0f : duty_a);
    duty_b = (duty_b > 1.0f) ? 1.0f : ((duty_b < 0.0f) ? 0.0f : duty_b);
    duty_c = (duty_c > 1.0f) ? 1.0f : ((duty_c < 0.0f) ? 0.0f : duty_c);

    /************************** 6. 占空比→PWM定时器比较值 **************************/
    // PWM_PERIOD为定时器自动重装值（例：16位定时器→65535）
    uint16_t pwm_a = (uint16_t)(duty_a * PWM_PERIOD);
    uint16_t pwm_b = (uint16_t)(duty_b * PWM_PERIOD);
    uint16_t pwm_c = (uint16_t)(duty_c * PWM_PERIOD);

    // 输出PWM到定时器（需根据硬件实现底层驱动）
    // bsp_pwm_set_duty(pwm_a, pwm_b, pwm_c);
}
