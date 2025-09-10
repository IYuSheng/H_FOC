#include "foc_encoder.h"

// 一阶低通滤波参数 (α值越小，滤波效果越强，响应越慢)
#define LOWPASS_ALPHA     0.01f

// HALL传感器电角度查找表 (单位: 弧度) - 精确值
const float hall_elec_angle_precise[8] = {
    0.0f,                    // 000-无效
    _PI/2 + 0.5236f,                   // 001-90°（π/2）
    7*_PI/6 + 0.5236f,                 // 010-210°（7π/6）
    5*_PI/6 + 0.5236f,                 // 011-150°（5π/6）
    11*_PI/6 + 0.5236f - 0.0001f,                // 100-330°（11π/6）
    _PI/6 + 0.5236f,                   // 101-30°（π/6）
    3*_PI/2 + 0.5236f,                 // 110-270°（3π/2）
    0.0f                     // 111-无效
};

// 电机转速和位置相关变量
static float hall_total_mechanical_angle = 0.0f; // 累计机械角度(弧度)
static uint32_t last_hall_time = 0;              // 上次HALL状态变化时间
static uint8_t last_hall_state = 0;              // 上次HALL状态
static float current_speed = 0.0f;               // 当前转速 (rad/s)
static float filtered_speed = 0.0f;              // 滤波后转速 (rad/s)
static float current_mechanical_angle = 0.0f;    // 当前机械角度 (rad)
static int8_t rotation_direction = 0;            // 旋转方向: 1=正转, -1=反转, 0=未知
const uint8_t hall_sequence_forward[6] = {5, 1, 3, 2, 6, 4};    // 正转序列

// 添加反电动势计算相关变量
static float32_t bemf_a = 0.0f, bemf_b = 0.0f, bemf_c = 0.0f;  // 三相反电动势
static float32_t last_ia = 0.0f, last_ib = 0.0f, last_ic = 0.0f;  // 上次电流值
static float32_t bemf_alpha = 0.0f, bemf_beta = 0.0f;  // αβ坐标系下的反电动势
static float32_t bemf_angle = 0.0f;  // 计算出的电角度

// 添加速度计算相关变量
static float32_t last_bemf_angle = 0.0f;  // 上次电角度
static float32_t bemf_speed_rpm = 0.0f;   // 机械速度(rpm)
static float32_t last_bemf_speed_rpm = 0.0f;  // 上次机械速度(rpm)

// 添加低通滤波系数
#define BEMF_LPF_ALPHA 0.2f  // 反电动势滤波系数
#define SPEED_LPF_ALPHA 0.01f // 反电动势速度滤波系数，较小的值意味着更强的滤波效果

/**
 * @brief 编码器初始化函数
 */
void foc_encoder_init(void)
{
    // 初始化相关变量
    hall_total_mechanical_angle = 0.0f;
    last_hall_time = 0;
    last_hall_state = 0;
    current_speed = 0.0f;
    filtered_speed = 0.0f;
    current_mechanical_angle = 0.0f;
}

/**
 * @brief 读取HALL传感器状态
 * @return 3位HALL传感器状态值，格式为 [0 0 0 0 0 C B A]
 */
uint8_t hall_sensor_read(void)
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
float hall_get_electrical_angle_rad(uint8_t hall_state)
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
 * @brief 根据当前和上一个HALL状态判断旋转方向
 * @param current_state 当前HALL状态
 * @param last_state 上一个HALL状态
 * @return 旋转方向: 1=正转, -1=反转, 0=未知
 */
inline static int8_t hall_get_rotation_direction(uint8_t current_state, uint8_t last_state)
{
    // 查找当前和上一个状态在序列中的位置
    int8_t current_index = -1, last_index = -1;
    
    // 在正转序列中查找位置
    for (int i = 0; i < 6; i++)
    {
        if (hall_sequence_forward[i] == current_state)  current_index = i;
        if (hall_sequence_forward[i] == last_state)     last_index = i;
    }
    
    // 计算索引差值
    int8_t index_diff = current_index - last_index;
        
    // 处理循环情况
    if (index_diff == 1 || index_diff == -5)
    {
       return 1; // 正转
    }
    else if (index_diff == -1 || index_diff == 5)
    {
        return -1; // 反转
    }
    else
    {
        return 0; // 未知
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
 * @brief 更新HALL传感器状态并计算速度和位置
 * @param current_time 当前时间戳(100us)
 * @return 电角度(弧度)
 */
float hall_update_position_and_speed(uint32_t current_time)
{
    static float last_mechanical_angle = 0.0f;
    uint8_t current_hall_state = hall_sensor_read();
    float electrical_angle = 0.0f;

    // 获取时间间隔
    float time_interval = (current_time - last_hall_time) * 0.0001f;
    
    // 如果HALL状态发生变化且时间间隔有效
    if (current_hall_state != last_hall_state && time_interval > 0.00001f)  // 避免除以极小值
    {
        // 判断旋转方向 (1:正转/反转:-1)
        rotation_direction = hall_get_rotation_direction(current_hall_state, last_hall_state);
        
        // 获取当前电角度
        electrical_angle = hall_get_electrical_angle_rad(current_hall_state);
        
        // 计算机械角度 (电角度 / 极对数)
        current_mechanical_angle = electrical_angle / MOTOR_POLE_PAIRS;
        
        // 根据旋转方向更新累计电角度
        hall_total_mechanical_angle += rotation_direction * _60_angle / MOTOR_POLE_PAIRS; // 每个HALL状态变化代表60度(机械角度)

        // 计算本次角度变化量
        float angle_diff = hall_total_mechanical_angle - last_mechanical_angle;

        // 计算当前机械原始速度 (rad/s)
        current_speed = angle_diff / time_interval;

        // 对原始机械速度进行一阶低通滤波
        filtered_speed = lowpass_filter(current_speed, filtered_speed, LOWPASS_ALPHA);

        // 更新上次状态和时间
        last_hall_time = bsp_get_micros();
        last_mechanical_angle = hall_total_mechanical_angle;
        last_hall_state = current_hall_state;
    }
    else if (current_time - last_hall_time > 1000)  // 超过100ms无变化，认为静止
    {
        electrical_angle = hall_get_electrical_angle_rad(current_hall_state);
        filtered_speed = lowpass_filter(0.0f, filtered_speed, LOWPASS_ALPHA * 2); // 快速衰减速度
    }
    else    // 状态未变化且间隔时间短情况
    {
        // 状态未变化但时间较短，使用插值计算更精确的电角度
        // 获取当前和下一个霍尔状态的电角度
        float current_sector_angle = hall_get_electrical_angle_rad(last_hall_state);
        float next_sector_angle = 0.0f;
        
        // 根据旋转方向计算下一个扇区角度
        if (rotation_direction == 1) { // 正转
            for (int i = 0; i < 6; i++) {
                if (hall_sequence_forward[i] == last_hall_state) {
                    next_sector_angle = hall_get_electrical_angle_rad(hall_sequence_forward[(i + 1) % 6]);
                    // 新增：正转时，若下一扇区角度 < 当前扇区角度，说明跨越2π，下扇区角度+2π
                    if (next_sector_angle < current_sector_angle) {
                        next_sector_angle += 2 * _PI;
                    }
                    break;
                }
            }
        } else if (rotation_direction == -1) { // 反转
            for (int i = 0; i < 6; i++) {
                if (hall_sequence_forward[i] == last_hall_state) {
                    next_sector_angle = hall_get_electrical_angle_rad(hall_sequence_forward[(i + 5) % 6]);
                    // 新增：反转时，若下一扇区角度 > 当前扇区角度，说明跨越0，下扇区角度-2π
                    if (next_sector_angle > current_sector_angle) {
                        next_sector_angle -= 2 * _PI;
                    }
                    break;
                }
            }
        } else {
            // 旋转方向未知，使用当前扇区角度
            electrical_angle = hall_get_electrical_angle_rad(current_hall_state);
        }
        
        // 如果能确定下一个扇区角度，则进行插值
        if (next_sector_angle != 0.0f && last_hall_time != 0) {
            // 计算在当前扇区的时间比例
            float sector_time_elapsed = time_interval; // 转换为秒
            float expected_sector_time = fabsf((next_sector_angle - current_sector_angle) / (filtered_speed * MOTOR_POLE_PAIRS));
            
            // 处理角度跨越2π的情况
            if (fabsf(next_sector_angle - current_sector_angle) > _PI) {
                if (next_sector_angle > current_sector_angle) {
                    current_sector_angle += 2 * _PI;
                } else {
                    next_sector_angle += 2 * _PI;
                }
            }
            
            // 时间比例（0-1之间）
            float time_ratio = sector_time_elapsed / expected_sector_time;
            time_ratio = fminf(1.0f, fmaxf(0.0f, time_ratio)); // 限制在0-1之间
            
            // 线性插值计算电角度
            electrical_angle = current_sector_angle + time_ratio * (next_sector_angle - current_sector_angle);
            
            // 角度归一化到0-2π范围
            electrical_angle = fmodf(electrical_angle, 2 * _PI);
        } else {
            // 无法插值，使用查表值
            electrical_angle = hall_get_electrical_angle_rad(current_hall_state);
        }
    }
    
    return electrical_angle;
}

/**
 * @brief 获取当前转速(滤波后)
 * @return 转速 (rad/s)
 */
float hall_get_speed_rad_per_sec(void)
{
    return filtered_speed;
}

/**
 * @brief 获取当前转速(RPM)(滤波后)
 * @return 转速 (RPM)
 */
float hall_get_speed_rpm(void)
{
    // rad/s 转换为 RPM: RPM = (rad/s) * 60 / (2*PI)
    return filtered_speed * 30.0f / _PI;
}

/**
 * @brief 获取当前机械角度
 * @return 累计机械角度 (弧度)
 */
float hall_get_mechanical_angle(void)
{
    return current_mechanical_angle;
}

/**
 * @brief 获取累计机械角度
 * @return 累计机械角度 (弧度)
 */
float hall_get_total_mechanical_angle(void)
{
    return hall_total_mechanical_angle;
}

/**
 * @brief 获取累计转数
 * @return 累计转数
 */
float hall_get_total_rotations(void)
{
    // 一圈为2*PI弧度
    return hall_total_mechanical_angle / (2.0f * _PI);
}

/**
 * @brief 计算反电动势并估算电角度
 * @param ia A相电流
 * @param ib B相电流
 * @param ic C相电流
 * @param va A相电压
 * @param vb B相电压
 * @param vc C相电压
 * @param dt 控制周期(s)
 * @return 估算的电角度(弧度)
 */
float32_t bsp_adc_calculate_bemf_angle(float32_t ia, float32_t ib, float32_t ic,
                                       float32_t va, float32_t vb, float32_t vc,
                                       float32_t dt)
{
    static float32_t last_bemf_a = 0.0f, last_bemf_b = 0.0f, last_bemf_c = 0.0f;
    static float32_t last_bemf_alpha = 0.0f, last_bemf_beta = 0.0f;
    static uint32_t last_call_time = 0;  // 上次调用时间戳
    static uint8_t first_call = 1;       // 首次调用标志
    
    // 电机相电阻和电感(需要根据实际电机参数调整)
    const float32_t R = MOTOR_RESISTANCE;   // 相电阻(欧姆)
    const float32_t L = MOTOR_INDUCTANCE; // 相电感(亨利)
    
    // 使用时间戳计算精确的时间间隔
    float32_t precise_dt = dt;  // 默认使用传入的dt值
    uint32_t current_time = bsp_get_micros();  // 获取当前时间戳
    
    if (!first_call && last_call_time != 0) {
        // 计算精确的时间间隔（单位：秒）
        if (current_time >= last_call_time) {
            precise_dt = (current_time - last_call_time) * 0.0001f;  // 转换为秒
        } else {
            // 处理计数器溢出（32位无符号整数最大值为0xFFFFFFFF）
            precise_dt = ((0xFFFFFFFF - last_call_time) + current_time) * 0.0001f;
        }
    } else {
        first_call = 0;
    }
    
    // 保存当前时间戳供下次使用
    last_call_time = current_time;
    
    // 计算三相反电动势: bemf = v - i*R - L*di/dt
    float32_t dia_dt = (ia - last_ia) / precise_dt;
    float32_t dib_dt = (ib - last_ib) / precise_dt;
    float32_t dic_dt = (ic - last_ic) / precise_dt;
    
    float32_t bemf_a_raw = va - ia * R - L * dia_dt;
    float32_t bemf_b_raw = vb - ib * R - L * dib_dt;
    float32_t bemf_c_raw = vc - ic * R - L * dic_dt;
    
    // 低通滤波反电动势
    bemf_a = BEMF_LPF_ALPHA * bemf_a_raw + (1.0f - BEMF_LPF_ALPHA) * last_bemf_a;
    bemf_b = BEMF_LPF_ALPHA * bemf_b_raw + (1.0f - BEMF_LPF_ALPHA) * last_bemf_b;
    bemf_c = BEMF_LPF_ALPHA * bemf_c_raw + (1.0f - BEMF_LPF_ALPHA) * last_bemf_c;
    
    // 保存当前值供下次使用
    last_ia = ia;
    last_ib = ib;
    last_ic = ic;
    last_bemf_a = bemf_a;
    last_bemf_b = bemf_b;
    last_bemf_c = bemf_c;
    
    // Clarke变换到αβ坐标系
    // α = (2*a - b - c)/3
    // β = (b - c)/√3
    bemf_alpha = (2.0f * bemf_a - bemf_b - bemf_c) / 3.0f;
    bemf_beta = (bemf_b - bemf_c) * _1_SQRT3;
    
    // 低通滤波αβ分量
    bemf_alpha = BEMF_LPF_ALPHA * bemf_alpha + (1.0f - BEMF_LPF_ALPHA) * last_bemf_alpha;
    bemf_beta = BEMF_LPF_ALPHA * bemf_beta + (1.0f - BEMF_LPF_ALPHA) * last_bemf_beta;
    last_bemf_alpha = bemf_alpha;
    last_bemf_beta = bemf_beta;
    
    // 计算电角度(使用atan2确保正确象限)
    bemf_angle = atan2f(bemf_beta, bemf_alpha);
    
    // 角度归一化到[0, 2π]
    if (bemf_angle < 0.0f) {
        bemf_angle += 2.0f * _PI;
    }
    
    // 计算速度：通过电角度变化率估算速度
    float32_t delta_angle = bemf_angle - last_bemf_angle;
    
    // 处理角度跨越2π的情况
    if (delta_angle > _PI) {
        delta_angle -= 2.0f * _PI;
    } else if (delta_angle < -_PI) {
        delta_angle += 2.0f * _PI;
    }
    
    // 计算电角速度(rad/s)
    float32_t bemf_speed_elec = delta_angle / precise_dt;
    
    // 转换为机械速度RPM
    float32_t bemf_speed_rpm_raw = bemf_speed_elec * RAD_TO_DEG * 60.0f / (360.0f * MOTOR_POLE_PAIRS);
    
    // 对速度进行低通滤波
    bemf_speed_rpm = SPEED_LPF_ALPHA * bemf_speed_rpm_raw + (1.0f - SPEED_LPF_ALPHA) * last_bemf_speed_rpm;
    
    // 保存当前角度和速度供下次使用
    last_bemf_angle = bemf_angle;
    last_bemf_speed_rpm = bemf_speed_rpm;
    
    return bemf_angle;
}

/**
 * @brief 获取计算的反电动势
 * @param alpha α轴反电动势指针
 * @param beta β轴反电动势指针
 */
void bsp_adc_get_bemf(float32_t* alpha, float32_t* beta)
{
    if (alpha) *alpha = bemf_alpha;
    if (beta) *beta = bemf_beta;
}

/**
 * @brief 获取基于反电动势估算的机械转速(RPM)
 * @return 机械转速(RPM)
 */
float32_t bsp_adc_get_bemf_speed_rpm(void)
{
    return bemf_speed_rpm;
}

