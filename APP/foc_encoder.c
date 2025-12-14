#include "foc_encoder.h"

// HALL传感器电角度查找表 (单位: 弧度) - 精确值
const float hall_elec_angle_precise[8] = {
    0.0f,                    // 000-无效
    0.0723f,                   // 001-90°（π/2）
    2.3811f,                 // 010-210°（7π/6）
    1.1206f,                 // 011-150°（5π/6）
    4.3164f,                // 100-330°（11π/6）
    5.2653f,                   // 101-30°（π/6）
    3.1484f,                 // 110-270°（3π/2）
    0.0f                     // 111-无效
};

const uint8_t hall_sequence_forward[6] = {5, 1, 3, 2, 6, 4};    // 正转序列

// 添加低通滤波系数
#define BEMF_LPF_ALPHA 0.2f  // 反电动势滤波系数
#define SPEED_LPF_ALPHA 0.01f // 反电动势速度滤波系数，较小的值意味着更强的滤波效果
#define LOWPASS_ALPHA     0.1f // 速度一阶低通滤波参数 (α值越小，滤波效果越强，响应越慢)

// 滑动平均值滤波变量
#define SPEED_FILTER_WINDOW_SIZE 8  // 滑动窗口大小
static float speed_filter_buffer[SPEED_FILTER_WINDOW_SIZE] = {0};
static uint8_t speed_filter_index = 0;

// PLL相关结构体和变量
typedef struct {
    float angle;          // 估计角度
    float speed;          // 估计速度
    float integral;       // 积分项
    float kp;            // 比例增益
    float ki;            // 积分增益
} pll_angle_estimator_t;

pll_angle_estimator_t pll_estimator;
static void pll_angle_estimator_init(float kp, float ki);

/**
 * @brief 编码器初始化函数
 */
void foc_encoder_init(void)
{
    // 初始化PLL角度估算器
    pll_angle_estimator_init(300.0f, 2000.0f); // 根据实际系统调整参数
}

/**
 * @brief 读取HALL传感器状态
 * @return 3位HALL传感器状态值，格式为 [0 0 0 0 0 C B A]
 */
inline static uint8_t hall_sensor_read(void)
{
    uint8_t hall_state = 0;
    
    // 读取HALL传感器三个引脚的状态
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
 * @brief 根据HALL传感器状态获取电角度(弧度)
 * @param hall_state HALL传感器状态 (0-7)
 * @return 电角度(弧度)
 */
inline static float hall_get_electrical_angle_rad(uint8_t hall_state)
{
    if (hall_state < 8)
    {
        return hall_elec_angle_precise[hall_state];
    }
    else
    {
        return 0.0f; // 无效状态返回默认值
    }
}

/**
 * @brief 一阶低通滤波处理
 * @param input 新的输入值
 * @param last_output 上一次的输出值
 * @param alpha 滤波系数 (0 < alpha < 1)
 * @return 滤波后的值
 */
inline static float lowpass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1 - alpha) * last_output;
}

/**
 * @brief 滑动平均值滤波函数
 * @param new_value 新的输入值
 * @return 滤波后的值
 */
inline static float moving_average_filter(float new_value)
{
    // 添加新值到缓冲区
    speed_filter_buffer[speed_filter_index] = new_value;
    
    // 更新索引
    speed_filter_index = (speed_filter_index + 1) % SPEED_FILTER_WINDOW_SIZE;
    
    // 计算平均值
    float result;
    arm_mean_f32(speed_filter_buffer, SPEED_FILTER_WINDOW_SIZE, &result);
    
    // 返回平均值
    return result;
}

/**
 * @brief 初始化PLL角度估算器
 * @param kp 比例增益
 * @param ki 积分增益
 */
static void pll_angle_estimator_init(float kp, float ki)
{
    pll_estimator.angle = 0.0f;
    pll_estimator.speed = 0.0f;
    pll_estimator.integral = 0.0f;
    pll_estimator.kp = kp;
    pll_estimator.ki = ki;
}

/**
 * @brief 使用PLL计算平滑角度
 * @param hall_angle 基于霍尔传感器的角度
 * @param dt 时间间隔
 * @return 平滑的角度值
 */
static inline void pll_angle_update(float hall_angle, float dt, pll_angle_estimator_t* pll_estimator)
{
    // 计算角度误差（处理角度环绕问题）
    float error = hall_angle - pll_estimator->angle;
    
    // 处理角度环绕（-π到π范围内）
    while (error > _PI) error -= _2PI;
    while (error < -_PI) error += _2PI;
    
    // PI控制器
    float p_term = pll_estimator->kp * error;
    pll_estimator->integral += pll_estimator->ki * error * dt;
    float pi_output = p_term + pll_estimator->integral;
    
    // 更新角度估计
    pll_estimator->angle += pi_output * dt;

    pll_estimator->speed = pll_estimator->angle;
    
    // 积分过零重置
    if(pll_estimator->angle >= _2PI || pll_estimator->angle <= -_2PI)
    {
        pll_estimator->integral = 0.0f;
    }
    // 角度归一化到0~2π范围
    pll_estimator->angle = fmodf(pll_estimator->angle, _2PI);

    if (pll_estimator->angle < 0.0f)
    {
        pll_estimator->angle += _2PI;
    }
}

/**
 * @brief 更新霍尔电角度与速度
 * @param 时间戳 (10微秒)
 * @return 平滑的角度值
 */
inline void hall_update_PLL(hall_get* hall_data)
{
    uint32_t current_time = bsp_get_micros();
    static uint32_t last_hall_time_local = 0;

    // 获取时间间隔 (修正时间单位转换系数)
    float time_interval = (current_time - last_hall_time_local) * 0.00001f;
    last_hall_time_local = current_time;

    uint8_t current_hall_state = hall_sensor_read();
    float hall_sector_angle = hall_get_electrical_angle_rad(current_hall_state);
    
    pll_angle_update(hall_sector_angle, time_interval, &pll_estimator);
    
    // 更新输出电角度与速度
    hall_data->angle = pll_estimator.angle;
    hall_data->speed = pll_estimator.speed;
}

/**
 * @brief 计算转速函数
 * @return 电机转速 (RPM)
 */
float calculate_motor_speed_rpm(void)
{
    static uint32_t last_hall_time = 0;
    static uint8_t last_hall_state = 0;
    static float filtered_speed = 0.0f;
    static float last_raw_rpm = 0.0f;
    
    uint32_t current_time = bsp_get_micros();
    uint8_t current_hall_state = hall_sensor_read();
    
    // 如果霍尔状态发生变化且等于序列中的第一个状态
    if (current_hall_state != last_hall_state && current_hall_state == hall_sequence_forward[0])
    {
        // 计算完整一圈的时间间隔（从上一次经过第一个状态到现在）
        float time_interval = (current_time - last_hall_time) * 0.00001f; // 转换为秒
        
        // 计算转速 RPM = 1(圈) / 极对数 / 时间间隔
        float raw_rpm = 60.0f / MOTOR_POLE_PAIRS / time_interval;
        // 应用滑动平均值滤波
        filtered_speed = moving_average_filter(raw_rpm);
        last_raw_rpm = raw_rpm;
        
        last_hall_time = current_time;
    }
    else if(current_time - last_hall_time > 10000) // 如果超过100毫秒没有检测到霍尔状态变化，则认为电机停止
    {
        filtered_speed = lowpass_filter(0.0f, last_raw_rpm, LOWPASS_ALPHA); // 快速衰减速度至0
    }
    last_hall_state = current_hall_state;
    return filtered_speed;
}
