#include "foc_encoder.h"

hall_get_t hall_data;

// HALL传感器电角度查找表 (单位: 弧度) - 基于实测标定
const float hall_elec_angle_precise[8] = {
    [0b000] = 0.0f,          // 000 无效
    [0b001] = 1.44f,         // 001 ~82.5°
    [0b010] = -2.61f,        // 010 ~210° (3.67 - 2π = -2.61)
    [0b011] = 2.62f,         // 011 ~150°
    [0b100] = -0.28f,        // 100 ~344°/0° (6.00 - 2π = -0.28)
    [0b101] = 0.63f,         // 101 ~36°
    [0b110] = -1.57f,        // 110 ~270° (4.71 - 2π = -1.57)
    [0b111] = 0.0f           // 111 无效
};

// 霍尔序列
const uint8_t hall_sequence_forward[6] = {5, 1, 3, 2, 6, 4};
const uint8_t hall_sequence_reverse[6] = {5, 4, 6, 2, 3, 1};

// PLL结构体
typedef struct {
    float angle;          // 估计角度
    float speed;          // 估计角速度 (rad/s)
    float speed_filt;     // 估计角速度滤波值 (rad/s)
    float integral;       // 速度积分项
    float kp;
    float ki;
} pll_estimator_t;

static pll_estimator_t g_pll = {0};

/**
 * @brief 读取HALL传感器状态
 */
inline static uint8_t hall_sensor_read(void)
{
    uint8_t hall_state = 0;
    
    if (GPIO_ReadInputDataBit(HALL_PORT, HALL_A_PIN) == Bit_SET) {
        hall_state |= 0x01;
    }
    if (GPIO_ReadInputDataBit(HALL_PORT, HALL_B_PIN) == Bit_SET) {
        hall_state |= 0x02;
    }
    if (GPIO_ReadInputDataBit(HALL_PORT, HALL_C_PIN) == Bit_SET) {
        hall_state |= 0x04;
    }
    
    return hall_state;
}

/**
 * @brief 获取HALL对应的电角度
 */
inline static float hall_get_electrical_angle(uint8_t hall_state)
{
    return (hall_state < 8) ? hall_elec_angle_precise[hall_state] : 0.0f;
}

/**
 * @brief 初始化PLL
 */
static void pll_init(float kp, float ki)
{
    g_pll.angle = 0.0f;
    g_pll.speed = 0.0f;
    g_pll.integral = 0.0f;
    g_pll.kp = kp;
    g_pll.ki = ki;
}

/**
 * @brief PLL更新
 * @param hall_angle 霍尔扇区角度
 * @param dt 时间间隔 (s)
 */
static inline void pll_update(float hall_angle, float dt)
{
    static ButterworthLPF_t omega_flux_filt;
    static uint8_t filt_inited = 0;
    
    // 初始化滤波器
    if (!filt_inited) {
        butterworth_init(&omega_flux_filt, 10.0f, PWM_FREQ);
        filt_inited = 1;
    }

    // 计算角度误差（最短路径）
    float error = angle_normalize_pi(hall_angle - g_pll.angle);
    
    // PI控制器
    float p_term = g_pll.kp * error;
    g_pll.integral += g_pll.ki * error * dt;
    
    // 积分限幅（防止饱和）
    if (g_pll.integral > MAX_PLL_SPEED) g_pll.integral = MAX_PLL_SPEED;
    if (g_pll.integral < -MAX_PLL_SPEED) g_pll.integral = -MAX_PLL_SPEED;
    
    // 角速度 = P项 + I项
    g_pll.speed = p_term + g_pll.integral;

    g_pll.speed_filt = butterworth_filter(&omega_flux_filt, g_pll.speed);
    
    // 更新角度
    g_pll.angle += g_pll.speed * dt;
    
    // 归一化角度
    g_pll.angle = angle_normalize(g_pll.angle);
}

/**
 * @brief 编码器初始化
 */
void foc_encoder_init(void)
{
    pll_init(PLL_KP_DEFAULT, PLL_KI_DEFAULT);
}

/**
 * @brief 更新霍尔PLL角度和速度 - 主函数
 */
void hall_update_PLL(hall_get_t* hall_data)
{
    uint8_t current_state = hall_sensor_read();
    
    // 时间间隔
    float dt = PWM_PERIOD_S;
    
    // 获取霍尔扇区角度
    float hall_angle = hall_get_electrical_angle(current_state);
    
    // 更新PLL
    pll_update(hall_angle, dt);
    
    // 输出数据
    hall_data->angle = g_pll.angle;
    hall_data->elec_speed = g_pll.speed_filt;
    hall_data->speed = g_pll.speed_filt * MOTOR_POLE_PAIRS;
}
