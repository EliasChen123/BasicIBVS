#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "../commu/serial_port_interface.hpp"

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <math.h>
#include <memory>

#define MOTOR_NUM 3

namespace RSR
{
    
class MotorControl {
public:
    typedef std::shared_ptr<MotorControl> Ptr;
    explicit MotorControl(SerialPortInterface::Ptr ptr);

    ~MotorControl();

    void EnableMotors();

    void DisableMotors();

    void SetZeroMotors();

    void ControlMotors(float pos[6], float vel[6], float kp[6], float kd[6], float tau[6]);

    void updatMotorState(std::vector<float> motorState);

    void real_time_motor_protection(float vel_[MOTOR_NUM], float tau_[MOTOR_NUM]);
    
    void KalmanFilter(float process_noise, float measurement_noise, float estimation_error, float initial_value, float limit);
      
    float update(float measurement);

    void ButterworthFilter(float cutoff_frequency, float sampling_rate);

    float Butt_update(float new_value);

    void compensate_static_friction_through_vel(float filtered_motor_vel[MOTOR_NUM], float motor_tor[MOTOR_NUM]);

    int floatToUint(float value, float min, float max, int bits);

    float uintToFloat(int value, float min, float max, int bits);

    void MitCtrl( float pos, float vel, float kp, float kd, float torq, uint8_t data[8]);

    void usbDataToMotorState(const std::vector<uint8_t>& data, std::vector<float>& motorState);

public:
    float m_current_motor_pos[MOTOR_NUM] = {0};
    float m_current_motor_vel[MOTOR_NUM] = {0};
    float m_current_motor_tau[MOTOR_NUM] = {0};

private:
    SerialPortInterface::Ptr    m_serial_interface_ptr;

    uint8_t                 m_enable_motor_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    uint8_t                 m_disable_motor_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    uint8_t                 m_setzero_motor_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

    std::vector<uint8_t>    m_enable_data; 
    std::vector<uint8_t>    m_disable_data; 
    std::vector<uint8_t>    m_setzero_data; 

private:

    static constexpr float P_MIN = -12.5f;
    static constexpr float P_MAX = 12.5f;

    static constexpr float V_MIN = -30.0f;
    static constexpr float V_MAX = 30.0f;

    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;

    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;

    static constexpr float T_MIN = -10.0f;
    static constexpr float T_MAX = 10.0f;

    float Q = 1e-6; // 过程噪声协方差
    float R = 1e-2; // 测量噪声协方差
    float P = 1; // 估计误差协方差
    float K; // 卡尔曼增益
    float X = 0; // 状态估计
    float threshold= 0.03;//阈值

    float a0, a1, a2, b1, b2;
    float x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;


private:



};

} // namespace RSR

#endif // MOTOR_CONTROL_H
