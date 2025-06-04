#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>
#include <vector>
#include <memory>
#include <queue>
#include <iostream>
#include <algorithm>
#include <cstdint>
#include <fstream>

#include "../motor_control/motor_control.hpp"
#include "../data_pool/data_pool.hpp"

#define ARM_DOF 3

namespace RSR
{
    
class Robot
{
public:
    typedef std::shared_ptr<Robot> Ptr;

    Robot(MotorControl::Ptr ptr, RSR::DataPool::Ptr dataPtr);
    ~Robot();

    void initialize();

    void impdence();

    void autoServo();

    void Ik_3Dof(float end_effect_pos[3], float q[3]);

    void fk(float q[3], float X[3]);

    bool now2aim(float sta_pos[3], float end_pos[3], float now_pos[3]);

    void Closed_Arm_Modle_decoup(float tau[3]);

    void joint2motor(   float q[ARM_DOF],float dq[ARM_DOF], 
                        float ddq[ARM_DOF],  float tau_joint[ARM_DOF],  
                        float motor_pos[ARM_DOF],float motor_vel[ARM_DOF], 
                        float motor_acc[ARM_DOF],  float tau_motor[ARM_DOF]);

    void end_impedance_control(float desir_end_pos[3], float desir_end_vel[3], 
        float current_joint_pos[3], float current_joint_vel[3], 
        float joint_tor[3], float c, float k, float *e_state,
        float delta[3]);

    void joint2end_force(float q[3], float joint_force[3], float end_force[3]);

    void Dof_3_Arm_iAV_Sca(float Joint[3], float V[3], float dV[3], float AV[6]);

    bool updatRobotJoint();

public:
    float m_joint_control_tau[ARM_DOF] = {0};
    float m_joint_gravity_tau[ARM_DOF] = {0};

    float m_desir_end_efect_pos[ARM_DOF] = {0};
    float m_desir_end_efect_vel[ARM_DOF] = {0};
    float m_desir_end_efect_tau[ARM_DOF] = {0};
    float m_desir_end_efect_acc[ARM_DOF] = {0};

    float m_desir_joint_pos[3] = {0};
    float m_desir_joint_vel[3] = {0};
    float m_desir_joint_acc[3] = {0};
    float m_desir_joint_tau[3] = {0};
    float m_desir_motor_pos[3] = {0};
    float m_desir_motor_vel[3] = {0};
    float m_desir_motor_tau[3] = {0};

    float m_link_lenth[2] = {0.184, 0.186};
    float m_link_mass[2] = {0.25, 0.15};

    float m_current_joint_pos[ARM_DOF] = {0};
    float m_current_joint_vel[ARM_DOF] = {0};
    float m_current_joint_acc[ARM_DOF] = {0};
    float m_current_joint_tau[ARM_DOF] = {0};
    float m_motor_direction[ARM_DOF] = {1, 1, -1};

private:
    std::vector<float>      m_motor_state;
    float                   m_last_pos[3];
    std::queue<float>       m_fy_queue;
    std::queue<float>       m_fx_queue;
    MotorControl::Ptr       m_motor_ptr;
    DataPool::Ptr           m_data_pool_ptr;

    std::vector<double>     m_record_pos_0;
    std::vector<double>     m_record_pos_1;
    std::vector<double>     m_record_pos_2;

    std::vector<double>     m_record_vel_0;
    std::vector<double>     m_record_vel_1;
    std::vector<double>     m_record_vel_2;

    std::vector<double>     m_record_tau_0;
    std::vector<double>     m_record_tau_1;
    std::vector<double>     m_record_tau_2;

    std::ofstream           m_record_data_file;
};

} // namespace RSR

#endif // ROBOT_H
