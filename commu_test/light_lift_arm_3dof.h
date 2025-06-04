#ifndef LlArm3dof_H
#define LlArm3dof_H

#include "math.h"
#include <vector>
#include <queue>
#include <iostream>
#include <algorithm>
#include <cstdint>
#include "light_lift_arm_3dof.h"

#define ARM_DOF 3

class LlArm3dof
{
public:
    LlArm3dof();
    ~LlArm3dof();

    float desir_joint_pos[3] = {0};
    float desir_joint_vel[3] = {0};
    float desir_joint_acc[3] = {0};
    float desir_joint_tau[3] = {0};
    float desir_motor_pos[3] = {0};
    float desir_motor_vel[3] = {0};
    float desir_motor_tau[3] = {0};
    float desir_end_efect_pos_default[ARM_DOF] = {0.21, 0, 0};
    float desir_end_efect_pos[ARM_DOF] = {0};
    float desir_end_efect_vel[ARM_DOF] = {0};
    float desir_end_efect_tau[ARM_DOF] = {0};
    float desir_end_efect_acc[ARM_DOF] = {0};
     
    float desir_grp_default = 100;
    float desir_grp = desir_grp_default;//0-230=闭合-张开
    float desir_joint_pos_default [3] = {0.21, 0, 0};
    bool is_load = false;
    bool init_load = false;

    float current_joint_pos[ARM_DOF] = {0};
    float current_joint_vel[ARM_DOF] = {0};
    float current_joint_acc[ARM_DOF] = {0};
    float current_joint_tau[ARM_DOF] = {0};
    float current_motor_pos[ARM_DOF] = {0};
    float current_motor_vel[ARM_DOF] = {0};
    float current_motor_tau[ARM_DOF] = {0};
    float current_motor_acc[ARM_DOF] = {0};
    float current_end_efect_pos[ARM_DOF] = {0};
    float current_end_efect_vel[ARM_DOF] = {0};
    float current_end_efect_tau[ARM_DOF] = {0};

    float end_force_value = -1.0;
    float end_force_orientation[3] = {0, 0, 1};//x轴正方向,只能是正数

    float joint_control_tau[ARM_DOF] = {0};
    float joint_gravity_tau[ARM_DOF] = {0};
    float joint_STSMC_tau[ARM_DOF] = {0};
    float joint_impedance_tau[ARM_DOF] = {0};
    float joint_pid_tau[ARM_DOF] = {0};
    float joint_admittance_tau[ARM_DOF] = {0};
    float motor_direction[ARM_DOF] = {1, 1, -1};

    std::vector<float> motorState;

    float link_lenth[2] = {0.184, 0.186};//link2 和 link3
    //增加了加爪质量，从0.08到0.2
    
    // float link_mass_default[2] = {0.25, 0.20};  //link2 和 link3//填充15%
    // float link_mass_default[2] = {0.3, 0.15};  //link2 和 link3//填充50%
    float link_mass_default[2] = {0.25, 0.15};
    float link_mass[2] = {link_mass_default[0], link_mass_default[1]};  //link2 和 link3

    float end2grp = 0.0;//0.12;//末端到夹爪的距离

    /***
     * 转动惯量就不使用类里的变量了，想改就到函数里改
     * */

    void moveToPosition(int x, int y, int z);
    void setSpeed(float speed);
    void stop();

    void get_jacob0();

    void get_jonit_vel_acc();

    void Dof_3_Arm_iAV_Sca(float Joint[3], float V[3], float dV[3], float AV[6]);

    void get_jacb(float q[3], float jacobian[3][3]);

    void end2joint_force(float q[3], float end_force[3], float joint_force[3]);

    void joint2end_force(float q[3], float joint_force[3], float end_force[3]);

    void fk(float q[3], float X[3]);

    void end_impedance_control(float desir_end_pos[3], float desir_end_vel[3], 
                               float current_joint_pos[3], float current_joint_vel[3], 
                               float joint_tor[3], float c, float k, float *e_state,
                               float delta[3]);

    void spherical(float desir_point[3], float desir_theta, float desir_fa, float current_point[3], float out_point[3]);

    void spherical_force(float desir_theta, float desir_fa, float force_r, float out_point[3]);

    int now2desir(float now_end_pos[3], float dlt_end_pos[3], float cur_pos[3], float cur_vel[3], float cur_acc[3], float next, int is_run);

    void Ik_3Dof(float end_effect_pos[3], float q[3]);
    void Closed_Arm_Modle_decoup(float q[3], float dq[3], float ddq[3], float tao_[3]);
    void Closed_Arm_Modle_teach(float q[3], float dq[3], float ddq[3], float tao_[3]);

    void PID_impdence(float xyz[3], float end_vel[3], float end_acc[3], float motor_pos_bk[3], float motor_vel_bk[3], float c, float k, float tao_imp[3], float motor_pos[3], float motor_vel[3]);

    void admittance(float reference_point[3], float motor_pos_bk[3], float motor_vel_bk[3], float end_force_orien[3], float end_force_value, float end_acc[3], float motor_pos[3], float motor_vel[3], float tao_dan[3]);

    void soft_start();

    void updatArmState(float motor_pos[3], float motor_vel[3], float motor_tau[3]);

    void joint2motor(float q[ARM_DOF],float dq[ARM_DOF], float ddq[ARM_DOF],  float tau_joint[ARM_DOF],  float motor_pos[ARM_DOF],float motor_vel[ARM_DOF], float motor_acc[ARM_DOF],  float tau_motor[ARM_DOF]);

    bool now2aim(float sta_pos[3], float end_pos[3], float now_pos[3], float steps, int delay);

private:
    int currentX, currentY, currentZ;
    float currentSpeed;
    float m_last_pos[3];

    std::queue<float> m_fy_queue;
    std::queue<float> m_fx_queue;
};

#endif // ARMCONTROL_H
