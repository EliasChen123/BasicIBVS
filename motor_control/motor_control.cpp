#include "motor_control.hpp"

namespace RSR
{

MotorControl::MotorControl(SerialPortInterface::Ptr ptr)
    : m_serial_interface_ptr(ptr),
    m_enable_data(m_serial_interface_ptr->FRAME_DATA_LENGTH, 0x01),
    m_disable_data(m_serial_interface_ptr->FRAME_DATA_LENGTH, 0x01),
    m_setzero_data(m_serial_interface_ptr->FRAME_DATA_LENGTH, 0x01)
{
    int cout = 0;
    for (size_t i = 0; i < 8; i++)
    {
        m_enable_data[i] = m_enable_motor_data[i];
        m_disable_data[i] = m_disable_motor_data[i];
        m_setzero_data[i] = m_setzero_motor_data[i];
        cout = 8;
        m_enable_data[cout + i] = m_enable_motor_data[i];
        m_disable_data[cout + i] = m_disable_motor_data[i];
        m_setzero_data[cout + i] = m_setzero_motor_data[i];
        cout = 16;
        m_enable_data[cout + i] = m_enable_motor_data[i];
        m_disable_data[cout + i] = m_disable_motor_data[i];
        m_setzero_data[cout + i] = m_setzero_motor_data[i];
        cout = 24;
        m_enable_data[cout + i] = m_enable_motor_data[i];
        m_disable_data[cout + i] = m_disable_motor_data[i];
        m_setzero_data[cout + i] = m_setzero_motor_data[i];
        cout = 32;
        m_enable_data[cout + i] = m_enable_motor_data[i];
        m_disable_data[cout + i] = m_disable_motor_data[i];
        m_setzero_data[cout + i] = m_setzero_motor_data[i];
        cout = 40;
        m_enable_data[cout + i] = m_enable_motor_data[i];
        m_disable_data[cout + i] = m_disable_motor_data[i];
        m_setzero_data[cout + i] = m_setzero_motor_data[i];
    }
}

MotorControl::~MotorControl()
{

}

int MotorControl::floatToUint(float value, float min, float max, int bits)
{
    float span = max - min;
    return static_cast<int>((value - min) * ((1 << bits) - 1) / span);
}

float MotorControl::uintToFloat(int value, float min, float max, int bits)
{
    float span = max - min;
    return value * span / ((1 << bits) - 1) + min;
}

void MotorControl::MitCtrl(float pos_, float vel_, float kp_, float kd_, float torq_, uint8_t data[8])
{
    int pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = floatToUint(pos_, P_MIN, P_MAX, 16);
    vel_tmp = floatToUint(vel_, V_MIN, V_MAX, 12);
    kp_tmp = floatToUint(kp_, KP_MIN, KP_MAX, 12);
    kd_tmp = floatToUint(kd_, KD_MIN, KD_MAX, 12);
    tor_tmp = floatToUint(torq_, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = (((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = (((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
    data[7] = tor_tmp;
}

void MotorControl::usbDataToMotorState(const std::vector<uint8_t> &data, std::vector<float> &motorState_)
{
    if (data.size() < 8)
    {
        throw std::invalid_argument("Insufficient data length for motor state extraction.");
    }

    motorState_.resize(9);

    int m1Int = (data[1] << 8) | data[2];
    int m2Int = (data[3] << 4) | (data[4] >> 4);
    int m3Int = ((data[4]&0xF)<<8)|data[5];

    motorState_[0] = uintToFloat(m1Int, P_MIN, P_MAX, 16);
    motorState_[1] = uintToFloat(m2Int, V_MIN, V_MAX, 12);
    motorState_[2] = uintToFloat(m3Int, T_MIN, T_MAX, 12);

    int offset = 8;
    m1Int = (data[offset + 1] << 8) | data[offset + 2];
    m2Int = (data[offset + 3] << 4) | (data[offset + 4] >> 4);
    m3Int = ((data[offset + 4]&0xF)<<8)|data[offset + 5];
    motorState_[3] = uintToFloat(m1Int, P_MIN, P_MAX, 16);
    motorState_[4] = uintToFloat(m2Int, V_MIN, V_MAX, 12);
    motorState_[5] = uintToFloat(m3Int, T_MIN, T_MAX, 12);

    offset = 16;
    m1Int = (data[offset + 1] << 8) | data[offset + 2];
    m2Int = (data[offset + 3] << 4) | (data[offset + 4] >> 4);
    m3Int = ((data[offset + 4]&0xF)<<8)|data[offset + 5];
    motorState_[6] = uintToFloat(m1Int, P_MIN, P_MAX, 16);
    motorState_[7] = uintToFloat(m2Int, V_MIN, V_MAX, 12);
    motorState_[8] = uintToFloat(m3Int, T_MIN, T_MAX, 12);

}

void MotorControl::EnableMotors()
{
    m_serial_interface_ptr->PackAndSend(m_enable_data);
}

void MotorControl::DisableMotors()
{
    m_serial_interface_ptr->PackAndSend(m_disable_data);
}

void MotorControl::SetZeroMotors()
{
    m_serial_interface_ptr->PackAndSend(m_setzero_data);
}

void MotorControl::ControlMotors(float pos[6], float vel[6], float kp[6], float kd[6], float tau[6])
{
    // uint8_t enableData[8] = {0};
    int motorIndex = 0;
    uint8_t motorDate_1[8] = {0};
    uint8_t motorDate_2[8] = {0};
    uint8_t motorDate_3[8] = {0};
    uint8_t motorDate_4[8] = {0};
    uint8_t motorDate_5[8] = {0};
    uint8_t motorDate_6[8] = {0};

    float desir_motor_pos_protect_max[MOTOR_NUM] = { 1.5,  1.45,  2.30};
    float desir_motor_pos_protect_min[MOTOR_NUM] = {-1.5, -0.22, -0.3};

    float desir_motor_vel_protect_max[MOTOR_NUM] = { 1.57,  2.57,  2.57};
    float desir_motor_vel_protect_min[MOTOR_NUM] = {-1.57, -2.57, -2.57};

    float desir_motor_tau_protect_max[MOTOR_NUM] = { 5,  5,  5};
    float desir_motor_tau_protect_min[MOTOR_NUM] = {-5, -5, -5};

    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        if(pos[i]>desir_motor_pos_protect_max[i]){
            std::cout << "joint" << i << " desir pos out range:" << " " << pos[i]<<std::endl;
            pos[i] = desir_motor_pos_protect_max[i];
        }
        if (pos[i] < desir_motor_pos_protect_min[i])
        {
            std::cout << "joint" << i << " desir pos out range:" << " " << pos[i] << std::endl;
            pos[i] = desir_motor_pos_protect_min[i];
        }

        if (vel[i] > desir_motor_vel_protect_max[i])
        {
            std::cout << "joint" << i << " desir vel out range:" << " " << vel[i] << std::endl;
            vel[i] = desir_motor_vel_protect_max[i];
        }
        if (vel[i] < desir_motor_vel_protect_min[i])
        {
            std::cout << "joint" << i << " desir vel out range:" << " " << vel[i] << std::endl;
            vel[i] = desir_motor_vel_protect_min[i];
        }

        if (tau[i] > desir_motor_tau_protect_max[i])
        {
            std::cout << "joint" << i << " desir tau out range:" << " " << tau[i] << std::endl;
            tau[i] = desir_motor_tau_protect_max[i];
        }
        if (tau[i] < desir_motor_tau_protect_min[i])
        {
            std::cout << "joint" << i << " desir tau out range:" << " " << tau[i] << std::endl;
            tau[i] = desir_motor_tau_protect_min[i];
        }

    }

    motorIndex = 0;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_1);
    motorIndex = 1;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_2);
    motorIndex = 2;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_3);
    motorIndex = 3;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_4);
    motorIndex = 4;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_5);
    motorIndex = 5;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_6);

    std::vector<uint8_t> data;
    data.resize(m_serial_interface_ptr->FRAME_DATA_LENGTH);
    std::vector<uint8_t> frame;

    for (int i = 0; i < 8; i++)
    {
        motorIndex = 0;
        data[motorIndex + i] = motorDate_1[i]; // 电机1
        motorIndex = 8;
        data[motorIndex + i] = motorDate_2[i]; // 电机2
        motorIndex = 16;
        data[motorIndex + i] = motorDate_3[i]; // 电机3
        motorIndex = 24;
        data[motorIndex + i] = motorDate_4[i]; // 电机4
        motorIndex = 32;
        data[motorIndex + i] = motorDate_5[i]; // 电机5
        motorIndex = 40;
        data[motorIndex + i] = motorDate_6[i]; // 电机6
    }

    m_serial_interface_ptr->PackAndSend(data);
}

void MotorControl::updatMotorState(std::vector<float> motorState)
{
    m_current_motor_pos[0] = motorState[0];
    m_current_motor_pos[1] = motorState[3];
    m_current_motor_pos[2] = motorState[6];

    m_current_motor_vel[0] = motorState[1];
    m_current_motor_vel[1] = motorState[4];
    m_current_motor_vel[2] = motorState[7];

    m_current_motor_tau[0] = motorState[2];
    m_current_motor_tau[1] = motorState[5];
    m_current_motor_tau[2] = motorState[8];
}

void MotorControl::real_time_motor_protection(float vel_[MOTOR_NUM], float tau_[MOTOR_NUM])
{
    static int tau_dengerous_cout[MOTOR_NUM] = {0};
    static int vel_dengerous_cout[MOTOR_NUM] = {0};

    int vel_max_dengerous_cout = 10;
    int tau_max_dengerous_cout = 10;

    int bad_motor[3] = {0};

    static bool out_flag = false;

    if(out_flag) return;

    float current_motor_pos_protect_max[MOTOR_NUM] = { 1.5,  1.45,  1.57};
    float current_motor_pos_protect_min[MOTOR_NUM] = {-1.5, -0.22, -0.27};

    float current_motor_vel_protect_max[MOTOR_NUM] = { 8,  8,  8};
    float current_motor_vel_protect_min[MOTOR_NUM] = {-8, -8, -8};

    float current_motor_tau_protect_max[MOTOR_NUM] = { 2.8,  4.8,  4.67};
    float current_motor_tau_protect_min[MOTOR_NUM] = {-0.2, -0.2, -0.17};

    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        if (abs(vel_[i]) > current_motor_vel_protect_max[i])
        {
            vel_dengerous_cout[i]++;
            std::cout << "joint vel " << i << " out range must Ctr + c kill programe!!!!" << std::endl;
            std::cout << "vel_[ " << i << "]= " <<  vel_[i] << std::endl;
            

            if (vel_dengerous_cout[i] >= vel_max_dengerous_cout)
            {
                //  dengerous_cout = 0;
                for (size_t i = 0; i < 10; i++)
                {
                    DisableMotors();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                bad_motor[i] = 1;
                out_flag = true;
            }
        }else{
            //标志位置零
            vel_dengerous_cout[i] = 0;

        }

        if (abs(tau_[i]) > current_motor_tau_protect_max[i])
        {
            tau_dengerous_cout[i]++;
            std::cout << "joint tau " << i << " dengerous must Ctr + c kill programe!!!!" << std::endl;

            if (tau_dengerous_cout[i] >= tau_max_dengerous_cout)
            {
                // dengerous_cout = 0;
                //加循环，失能电机
                for (size_t i = 0; i < 10; i++)
                {
                    DisableMotors();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                bad_motor[i] = 1;
                out_flag = true;
            }
        }
        else
        {
            // 标志位置零
            tau_dengerous_cout[i] = 0;
        }

    }
}

void MotorControl::KalmanFilter(float process_noise, float measurement_noise, float estimation_error, float initial_value, float limit) {
    Q = process_noise;
    R = measurement_noise;
    P = estimation_error;
    X = initial_value;
    threshold = limit;
}

float MotorControl::update(float measurement) {
    // 预测步骤
    P = P + Q;

    // 计算卡尔曼增益
    K = P / (P + R);

    // 更新状态估计
    X = X + K * (measurement - X);

    // 更新状态协方差
    P = (1 - K) * P;

    // std::cout << "threshold: " << threshold<< std::endl;

    //阈值滤波
    if (std::abs(X) < threshold)
    {
        X = 0.0;
    }

    return X;
}

void MotorControl::ButterworthFilter(float cutoff_frequency, float sampling_rate) {
    float wc = tan(M_PI * cutoff_frequency / sampling_rate);
    float k1 = sqrt(2.0) * wc;
    float k2 = wc * wc;
    a0 = k2 / (1.0 + k1 + k2);
    a1 = 2.0 * a0;
    a2 = a0;
    b1 = 2.0 * (k2 - 1.0) / (1.0 + k1 + k2);
    b2 = (1.0 - k1 + k2) / (1.0 + k1 + k2);
}

float MotorControl::Butt_update(float new_value) {
    float result = a0 * new_value + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
    x2 = x1;
    x1 = new_value;
    y2 = y1;
    y1 = result;
    return result;
}

void MotorControl::compensate_static_friction_through_vel(float filtered_motor_vel[MOTOR_NUM], float motor_tor[MOTOR_NUM])
{
    float static_friction_in_gravity_max[MOTOR_NUM] = { 0.08,  0.2,   0.15};
    float static_friction_in_gravity_min[MOTOR_NUM] = {-0.08, -0.04, -0.05};

    for (size_t i = 0; i < 3; i++)
    {
        if (filtered_motor_vel[i] > 0)
            motor_tor[i] += static_friction_in_gravity_max[i];
        if (filtered_motor_vel[i] < 0)
            motor_tor[i] += static_friction_in_gravity_min[i];
    }
}

} // namespace RSR




