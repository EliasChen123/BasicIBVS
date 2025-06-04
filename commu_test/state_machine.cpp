#include <iostream>
#include <iomanip>
#include "state_machine.h"

    

ArmState::ArmState() : currentState(State::AutoServo) {}

int signum(float x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/*测试函数*/
void ts_static_fraction(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{

    float kp1 = 45;
    float kp2 = 45;
    float kp3 = 45;
    float kd1 = 0.23;
    float kd2 = 0.23;
    float kd3 = 0.13;

    // while (true)
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        // arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

        // 关节的控制力矩 = 重力补偿力矩 + 阻尼力矩

        arm.joint_control_tau[0] = arm.joint_gravity_tau[0];

        arm.joint_control_tau[1] = arm.joint_gravity_tau[1];

        arm.joint_control_tau[2] = arm.joint_gravity_tau[2];

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // 直接在电机空间运行pid

        motor.motor_control_tau[2] += arm.desir_end_efect_pos[1];

        motor.kp[0] = 0.0;
        motor.kp[1] = 0.0;
        motor.kp[2] = 0.0;
        motor.kd[0] = 0.0;
        motor.kd[1] = 0.0;
        motor.kd[2] = 0.0;
        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // std::cout << "manualServo-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "manualServo-current_end_pos  : " << arm.current_end_efect_pos[0] << " " << arm.current_end_efect_pos[1] << " " << arm.current_end_efect_pos[2]<< std::endl;
    }
}

void ArmState::modeTransition(State newState)
{
    switch (newState)
    {
    case State::AutoServo:
        std::cout << "Transitioning to AutoServo state.\n";
        break;
    case State::ManualServo:
        std::cout << "Transitioning to ManualServo state.\n";
        break;
    case State::Impdence:
        std::cout << "Transitioning to Impdence state.\n";
        break;
    case State::Admittance:
        std::cout << "Transitioning to Admittance state.\n";
        break;
    case State::JointAutoServo:
        std::cout << "Transitioning to JointAutoServo state.\n";
        break;
    case State::Gravity:
        std::cout << "Transitioning to Gravity state.\n";
        break;
    case State::Zero:
        std::cout << "Transitioning to Zero state.\n";
        break;
    case State::Teach:
        std::cout << "Transitioning to Teach state.\n";
        break;
    }

    // 更新当前状态
    currentState = newState;
}

void ArmState::run(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{

    static bool initialized = false;
    static bool joint_initialized = false;

    if (currentState == State::Zero)
        initialized = true; // 校零不用到期望位型

    if (!initialized)
    {
        std::cout << "Initializing Robot Arm.\n";
        if ((currentState == State::JointAutoServo))
        { // 关节空间位型初始化
            joint_initialize(port, arm, motor);
            initialized = true;
        }
        else
        { // 工作空间位型初始化
            initialize(port, arm, motor);
            initialized = true;
        }
    }

    switch (currentState)
    {
    case State::AutoServo:
        autoServo(port, arm, motor);
        break;

    case State::ManualServo:
        // manualServo(port, arm, motor);
        // manualServo_CTSMC(port, arm, motor);
        manualServo_STSMC(port, arm, motor);
        break;

    case State::Impdence:
        impdence(port, arm, motor);
        break;

    case State::Admittance:
        admittance(port, arm, motor);
        break;

    case State::JointAutoServo:
        joint_auto_servo(port, arm, motor);
        break;

    case State::Gravity:
        // std::cout << "Robot Arm has in Gravity state.\n";
        gravity(port, arm, motor);
        break;
    case State::Zero:
        std::cout << "Robot Arm has in Zero state.\n";
        motor.SetZeroMotors(port);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        motor.SetZeroMotors(port);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "ok.\n";
        break;
    case State::Teach:
        teach(port, arm, motor);
        break;
    }
}

void ArmState::autoServo(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{

    // while (true)
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        float est_joint_load[3] = {15 * (arm.current_joint_tau[0] - arm.joint_gravity_tau[0]), 15 * (arm.current_joint_tau[1] - arm.joint_gravity_tau[1]), 15 * (arm.current_joint_tau[2] - arm.joint_gravity_tau[2])};

        arm.joint2end_force(arm.current_joint_pos, est_joint_load, arm.current_end_efect_tau);

        arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

        // std::cout << "autoServo-desir_joint_pos: " << arm.desir_joint_pos[0] << " " << arm.desir_joint_pos[1] << " " << arm.desir_joint_pos[2]<< std::endl;
       
        float AV[6] = {0};

        arm.Dof_3_Arm_iAV_Sca(arm.desir_joint_pos, arm.desir_end_efect_vel, arm.desir_end_efect_acc, AV);

        for (size_t i = 0; i < 3; i++)
        {
            arm.desir_joint_vel[i] = AV[i];
            arm.desir_joint_acc[i] = AV[3 + i];
        }

        arm.joint_control_tau[0] = 1.0 * arm.joint_gravity_tau[0];

        arm.joint_control_tau[1] = 1.0 * arm.joint_gravity_tau[1];

        arm.joint_control_tau[2] = 1.0 * arm.joint_gravity_tau[2];

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // 用期望的电机速度，提前补偿静摩擦
        motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);

        // motor.ControlMotors(port,
        //                     motor.desir_motor_pos,
        //                     motor.desir_motor_vel,
        //                     motor.kp,
        //                     motor.kd,
        //                     motor.motor_control_tau);

        motor.ControlMotors_g(port,
                              motor.desir_motor_pos,
                              motor.desir_motor_vel,
                              motor.kp,
                              motor.kd,
                              motor.motor_control_tau,
                              arm.desir_grp);

        float force = sqrt(arm.current_end_efect_tau[0] * arm.current_end_efect_tau[0] + arm.current_end_efect_tau[1] * arm.current_end_efect_tau[1] + arm.current_end_efect_tau[2] * arm.current_end_efect_tau[2]);

        // float estimate_end_load = force / 0.00981;
        // std::cout << "autoServo-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "autoServo-current_end_pos  : " << arm.current_end_efect_pos[0] << " " << arm.current_end_efect_pos[1] << " " << arm.current_end_efect_pos[2]<< std::endl;
        // std::cout << "autoServo-current_motor_tau: " << motor.current_motor_tau[0] << " " << motor.current_motor_tau[1] << " " << motor.current_motor_tau[2]<< std::endl;
        // std::cout << std::endl;
        // std::cout << "autoServo-estimate_end_load: " << arm.current_end_efect_tau[2] << std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}
/// @brief 电机pid为零，仅用力矩控制，手动实现pid
/// @param port
/// @param arm
/// @param motor
void ArmState::manualServo(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{

    float kp1 = 25;
    float kp2 = 25;
    float kp3 = 22;
    float kd1 = 0.93;
    float kd2 = 0.93;
    float kd3 = 0.73;

    // while (true)
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

        float AV[6] = {0};

        arm.Dof_3_Arm_iAV_Sca(arm.desir_joint_pos, arm.desir_end_efect_vel, arm.desir_end_efect_acc, AV);

        for (size_t i = 0; i < 3; i++)
        {
            arm.desir_joint_vel[i] = AV[i];
            arm.desir_joint_acc[i] = AV[3 + i];
        }

        // 关节的控制力矩 = 重力补偿力矩 + 阻尼力矩

        arm.joint_control_tau[0] = arm.joint_gravity_tau[0];

        arm.joint_control_tau[1] = arm.joint_gravity_tau[1];

        arm.joint_control_tau[2] = arm.joint_gravity_tau[2];

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // 直接在电机空间运行pid

        motor.motor_pid_tau[0] = kp1 * (motor.desir_motor_pos[0] - motor.current_motor_pos[0]) + kd1 * (motor.desir_motor_vel[0] - motor.current_motor_vel[0]);

        motor.motor_pid_tau[1] = kp2 * (motor.desir_motor_pos[1] - motor.current_motor_pos[1]) + kd2 * (motor.desir_motor_vel[1] - motor.current_motor_vel[1]);

        motor.motor_pid_tau[2] = kp3 * (motor.desir_motor_pos[2] - motor.current_motor_pos[2]) + kd3 * (motor.desir_motor_vel[2] - motor.current_motor_vel[2]);

        for (size_t i = 0; i < 3; i++)
        {
            motor.motor_control_tau[i] += motor.motor_pid_tau[i];
        }

        // float est_joint_load[3] = {(arm.current_joint_tau[0] - arm.joint_gravity_tau[0]), (arm.current_joint_tau[1] - arm.joint_gravity_tau[1]), (arm.current_joint_tau[2] - arm.joint_gravity_tau[2])};

        arm.joint2end_force(arm.current_joint_pos, motor.motor_pid_tau, arm.current_end_efect_tau);

        // 用期望的电机速度，提前补偿静摩擦
        motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);

        motor.kp[0] = 0.0;
        motor.kp[1] = 0.0;
        motor.kp[2] = 0.0;
        motor.kd[0] = 0.0;
        motor.kd[1] = 0.0;
        motor.kd[2] = 0.0;
        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        std::cout << "manualServo-estimate_end_load: " << arm.current_end_efect_tau[2] << std::endl;
        // std::cout << "manualServo-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "manualServo-current_end_pos  : " << arm.current_end_efect_pos[0] << " " << arm.current_end_efect_pos[1] << " " << arm.current_end_efect_pos[2]<< std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}
/// @brief 电机pid为零，仅用力矩控制，但是使用超螺旋滑模算法
/// @param port
/// @param arm
/// @param motor
void ArmState::manualServo_STSMC(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{

    // while (true)
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

        float JointState_r[6] = {0};
        float JointState_D[9] = {0};
        float JointState[9] = {0};
        float estimateT[3] = {0};
        float ControlT[3] = {0}; // 控制输入
        float s[3] = {0};
        float e[3] = {0};
        float de[3] = {0};
        float K[3] = {0};
        float S_K[3] = {0};
        float F_K1[3] = {0};
        float F_K2[3] = {0};
        float V[3] = {0};
        float Yd[3] = {0};
        float S_state[3] = {0};
        float Control_his[5] = {0};
        float detlta;
        float A1 = 0;
        float A2 = 0;
        float B1 = 0;
        float B2 = 0;
        float C1 = 0;
        float C2 = 0;

        K[0] = 90;
        S_K[0] = 0.1; // 适当的增大会增加精度
        F_K1[0] = 0.1;
        F_K2[0] = 0.1;

        K[1] = 120;
        S_K[1] = 0.2; // 但是会导致运动的波动，负载越大波动越严重
        F_K1[1] = 0.15;
        F_K2[1] = 0.1;

        K[2] = 80;     // 增大精确，与抖动无关
        S_K[2] = 0.1;  // 增大会增加控制力矩的振动频率，会更快恢复平稳状态
        F_K1[2] = 0.1; // 增大后显著抖动
        F_K2[2] = 0.1; // 调小了丝滑，增大抖动

        /* 误 差 */
        for (size_t i = 0; i < 3; i++)
        {
            e[i] = arm.desir_joint_pos[i] - arm.current_joint_pos[i];

            de[i] = arm.desir_joint_vel[i] - arm.current_joint_vel[i];

            s[i] = 3.0 * de[i] + K[i] * e[i]; // 擅自在速度误差上加了增益，也算有点效果

            /*更 新 滑 模 状 态*/
            S_state[i] += S_K[i] * signum(s[i]) * 0.001;
        }

        float desir_joint_vel_acc[6] = {0};

        arm.Dof_3_Arm_iAV_Sca(arm.desir_joint_pos, arm.desir_end_efect_vel, arm.desir_end_efect_acc, desir_joint_vel_acc);

        arm.desir_joint_vel[0] = desir_joint_vel_acc[0];
        arm.desir_joint_vel[1] = desir_joint_vel_acc[1];
        arm.desir_joint_vel[2] = desir_joint_vel_acc[2];

        arm.desir_joint_acc[3] = desir_joint_vel_acc[3];
        arm.desir_joint_acc[4] = desir_joint_vel_acc[4];
        arm.desir_joint_acc[5] = desir_joint_vel_acc[5];

     
        // float estimateT[3] = {0};
        // float ControlT[3] = {0};
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.desir_joint_acc,
                                    estimateT);

        ControlT[0] = estimateT[0] + F_K1[0] * s[0] + F_K2[0] * sqrt(abs(s[0])) * tanh(s[0]) + S_state[0];

        ControlT[1] = estimateT[1] + F_K1[1] * s[1] + F_K2[1] * sqrt(abs(s[1])) * tanh(s[1]) + S_state[1];

        ControlT[2] = estimateT[2] + F_K1[2] * s[2] + F_K2[2] * sqrt(abs(s[2])) * tanh(s[2]) + S_state[2];

        detlta = 0.63;
        if (ControlT[0] > (arm.joint_gravity_tau[0] + detlta))
            ControlT[0] = arm.joint_gravity_tau[0] + detlta;
        if (ControlT[0] < (arm.joint_gravity_tau[0] - detlta))
            ControlT[0] = arm.joint_gravity_tau[0] - detlta;

        detlta = 1.69;
        if (ControlT[1] > (arm.joint_gravity_tau[1] + detlta))
            ControlT[1] = arm.joint_gravity_tau[1] + detlta;
        if (ControlT[1] < (arm.joint_gravity_tau[1] - detlta))
            ControlT[1] = arm.joint_gravity_tau[1] - detlta;

        detlta = 1.6;
        if (ControlT[2] > (arm.joint_gravity_tau[2] + detlta))
            ControlT[2] = arm.joint_gravity_tau[2] + detlta;
        if (ControlT[2] < (arm.joint_gravity_tau[2] - detlta))
            ControlT[2] = arm.joint_gravity_tau[2] - detlta;

        // 关节的控制力矩 = 重力补偿力矩 + 阻尼力矩

        arm.joint_control_tau[0] = ControlT[0];

        arm.joint_control_tau[1] = 1 * ControlT[1];

        arm.joint_control_tau[2] = 1 * ControlT[2];

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        motor.kp[0] = 0.0;
        motor.kp[1] = 0.0;
        motor.kp[2] = 0.0;
        motor.kd[0] = 0.0;
        motor.kd[1] = 0.0;
        motor.kd[2] = 0.0;

        // 用期望的电机速度，提前补偿静摩擦
        motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // std::cout << "manualServo_STSMC-motor_control_tau: " << motor.motor_control_tau[0] << " " << motor.motor_control_tau[1] << " " << motor.motor_control_tau[2] << std::endl;
        std::cout << "manualServo_STSMC-desir_joint_vel: " << arm.desir_joint_vel[0] << " " << arm.desir_joint_vel[1] << " " << arm.desir_joint_vel[2] << std::endl;

        // 超螺旋控制运算主程序
    }
}
/// @brief 基于计算力矩法的滑模控制
/// @param port
/// @param arm
/// @param motor
void ArmState::manualServo_CTSMC(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{
    auto parsedData = port.getReceivedData();

    motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

    motor.updatMotorState(motor.motorState); // 更新电机实时状态

    arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

    arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

    // 实时重力补偿
    arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                arm.current_joint_vel,
                                arm.current_joint_acc,
                                arm.joint_gravity_tau);

    arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

    float JointState_r[6] = {0};
    float JointState_D[9] = {0};
    float JointState[9] = {0};
    float estimateT[3] = {0};
    float ControlT[3] = {0}; // 控制输入
    float s[3] = {0};
    float e[3] = {0};
    float de[3] = {0};
    float K[3] = {0};
    float V[3] = {0};
    float Yd[3] = {0};
    float detlta;

    K[0] = 50;

    K[1] = 74;

    K[2] = 40;


    float desir_joint_vel_acc[6] = {0};

    arm.Dof_3_Arm_iAV_Sca(arm.desir_joint_pos, arm.desir_end_efect_vel, arm.desir_end_efect_acc, desir_joint_vel_acc);

    for (size_t i = 0; i < 3; i++)
    {
      arm.desir_joint_vel[i] = desir_joint_vel_acc[i];
    }
    


    /* 误 差 */
    for (size_t i = 0; i < 3; i++)
    {
        e[i] = arm.desir_joint_pos[i] - arm.current_joint_pos[i];

        de[i] = arm.desir_joint_vel[i] - arm.current_joint_vel[i];

        s[i] = 1.2 * de[i] + K[i] * e[i]; // 擅自在速度误差上加了增益，也算有点效果
    }


    /*计算力矩算法实现*/
    // Yd[0] = (27 + 0) * tanh(s[0] / 4);
    // Yd[1] = (75 + 4) * tanh(s[1]);
    // Yd[2] = (50 + 4) * tanh(s[2]);

    Yd[0] = (90 + 0) * tanh(s[0] / 4);
    Yd[1] = (250 + 4) * tanh(s[1] / 10);
    Yd[2] = (190 + 4) * tanh(s[2] / 6);
    /********************************************/

    arm.desir_joint_acc[0] = desir_joint_vel_acc[3] + K[0] * de[0] + Yd[0];
    arm.desir_joint_acc[1] = desir_joint_vel_acc[4] + K[1] * de[1] + Yd[1];
    arm.desir_joint_acc[2] = desir_joint_vel_acc[5] + K[2] * de[2] + Yd[2];

   
    arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                arm.current_joint_vel,
                                arm.desir_joint_acc,
                                estimateT);

    // estimateT = Closed_Arm_Modle_Opt(JointState);

    ControlT[0] = arm.joint_gravity_tau[0] + estimateT[0];

    ControlT[1] = arm.joint_gravity_tau[1] + estimateT[1];

    ControlT[2] = arm.joint_gravity_tau[2] + estimateT[2];

    detlta = 1.43;
    if (ControlT[0] > (arm.joint_gravity_tau[0] + detlta))
        ControlT[0] = arm.joint_gravity_tau[0] + detlta;
    if (ControlT[0] < (arm.joint_gravity_tau[0] - detlta))
        ControlT[0] = arm.joint_gravity_tau[0] - detlta;

    detlta = 1.89;
    if (ControlT[1] > (arm.joint_gravity_tau[1] + detlta))
        ControlT[1] = arm.joint_gravity_tau[1] + detlta;
    if (ControlT[1] < (arm.joint_gravity_tau[1] - detlta))
        ControlT[1] = arm.joint_gravity_tau[1] - detlta;

    detlta = 1.8;
    if (ControlT[2] > (arm.joint_gravity_tau[2] + detlta))
        ControlT[2] = arm.joint_gravity_tau[2] + detlta;
    if (ControlT[2] < (arm.joint_gravity_tau[2] - detlta))
        ControlT[2] = arm.joint_gravity_tau[2] - detlta;

    arm.joint_control_tau[0] = ControlT[0];

    arm.joint_control_tau[1] = 1 * ControlT[1];

    arm.joint_control_tau[2] = 1 * ControlT[2];

    // 由于关节和电机的方向有可能不一致，所以要转换一下

    arm.joint2motor(arm.desir_joint_pos,
                    arm.desir_joint_vel,
                    arm.desir_joint_acc,
                    arm.joint_control_tau,
                    motor.desir_motor_pos,
                    motor.desir_motor_vel,
                    motor.desir_motor_acc,
                    motor.motor_control_tau);

    motor.kp[0] = 0.0;
    motor.kp[1] = 0.0;
    motor.kp[2] = 0.0;
    motor.kd[0] = 0.0;
    motor.kd[1] = 0.0;
    motor.kd[2] = 0.0;

    // 用期望的电机速度，提前补偿静摩擦
    // motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);

    // motor.ControlMotors(port,
    //                     motor.desir_motor_pos,
    //                     motor.desir_motor_vel,
    //                     motor.kp,
    //                     motor.kd,
    //                     motor.motor_control_tau);

    motor.ControlMotors_g(port,
                          motor.desir_motor_pos,
                          motor.desir_motor_vel,
                          motor.kp,
                          motor.kd,
                          motor.motor_control_tau,
                          arm.desir_grp);

    std::cout << "manualServo_STSMC-motor_control_tau: " << motor.motor_control_tau[0] << " " << motor.motor_control_tau[1] << " " << motor.motor_control_tau[2] << std::endl;
    
    // ... existing code
    
     
}

/**
 *
 * 使用纯力矩控制，实现阻抗，
 * 比较丝滑，不容易抖动。
 * 但是，非阻抗的轴向，控制精度不如用电机内部pid的高
 * 这个模式用来估计末端负载非常合适
 * 仅有此模式增加了更新惯性参数（连杆2的质量）功能
 */
void ArmState::impdence(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{
    // while (true)
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

        arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

        float end_force[3] = {0};
        // 根据期望的末端速度和位置和当前的末端位置和速度
        // 计算关节的阻尼力矩
        static int cnt = 20;
        static bool first = true;
        static float last_pos[3];
        if(first){
            last_pos[0] = arm.current_end_efect_pos[0];
            last_pos[1] = arm.current_end_efect_pos[1];
            last_pos[2] = arm.current_end_efect_pos[2];
            first = false;
        }

        if(--cnt == 0){
            float delta[3];
            delta[0] = arm.current_end_efect_pos[0] - last_pos[0];
            delta[1] = arm.current_end_efect_pos[1] - last_pos[1];
            delta[2] = arm.current_end_efect_pos[2] - last_pos[2];

            arm.end_impedance_control(arm.desir_end_efect_pos,
                arm.desir_end_efect_vel,
                arm.current_joint_pos,
                arm.current_joint_vel,
                arm.joint_impedance_tau,
                //4.0, 60,
                  4, 120,
                end_force,
                delta);

            cnt = 20;

            last_pos[0] = arm.current_end_efect_pos[0];
            last_pos[1] = arm.current_end_efect_pos[1];
            last_pos[2] = arm.current_end_efect_pos[2];

            arm.desir_end_efect_pos[0] = arm.current_end_efect_pos[0];
            arm.desir_end_efect_pos[1] = arm.current_end_efect_pos[1];
            arm.desir_end_efect_pos[2] = arm.current_end_efect_pos[2];

        }

        if (arm.is_load)
        {
            // 有负载，则修改动力学参数
            arm.link_mass[1] = arm.link_mass_default[1] + 0.0015 * (end_force[2] / 0.00981);
             
            std::cout << "m3 ,end load, z aix force  : " << arm.link_mass[1] << " " << end_force[2] / 0.00981 << " " << end_force[2] << std::endl;
        }

        if (arm.init_load)
        {
            // 无负载，恢复连杆质量
            arm.link_mass[1] = arm.link_mass_default[1];

            std::cout << "m3 ,end load, z aix force  : " << arm.link_mass[1] << " " << end_force[2] / 0.00981 << " " << end_force[2] << std::endl;
        }

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        // 关节的控制力矩 = 重力补偿力矩 + 阻尼力矩
        arm.joint_control_tau[0] = arm.joint_gravity_tau[0] + 1.0 * arm.joint_impedance_tau[0];

        arm.joint_control_tau[1] = arm.joint_gravity_tau[1] + 1.0 * arm.joint_impedance_tau[1];

        arm.joint_control_tau[2] = arm.joint_gravity_tau[2] + 1.0 * arm.joint_impedance_tau[2];

        // 根据当前关节角速度，对静摩擦力进行补偿
        //  float static_fr_max[3] = { 0.08,  0.25,  0.15};
        //  float static_fr_min[3] = {-0.08, -0.25, -0.15};
        float static_fr_max[3] = { 0.1,  0.25,  0.1};
        float static_fr_min[3] = {-0.1, -0.25, -0.15};

        float cut = 0.0;//在他的临界点附近会抖

        if(std::abs(arm.current_joint_vel[0]) < 0.1){
            arm.joint_control_tau[0] *= 1.2;
        }
        if(std::abs(arm.current_joint_vel[0]) > 0.3){
            if (arm.current_joint_vel[0] > cut)
            arm.joint_control_tau[0] += static_fr_max[0];
            if (arm.current_joint_vel[0] < -cut)
            arm.joint_control_tau[0] += static_fr_min[0];
        }
        
        if(std::abs(arm.current_joint_vel[1]) < 0.1){
            arm.joint_control_tau[1] *= 1.2;
        }
        if(std::abs(arm.current_joint_vel[1]) > 0.3){
            if (arm.current_joint_vel[1] > cut)
            arm.joint_control_tau[1] += static_fr_max[1];
            if (arm.current_joint_vel[1] < -cut)
            arm.joint_control_tau[1] += static_fr_min[1];
        }

        if(std::abs(arm.current_joint_vel[2]) < 0.1){
            arm.joint_control_tau[2] *= 1.4;
        }
        if(std::abs(arm.current_joint_vel[2]) > 0.3){
            if (arm.current_joint_vel[2] > cut)
            arm.joint_control_tau[2] += static_fr_max[2];
            if (arm.current_joint_vel[2] < -cut)
            arm.joint_control_tau[2] += static_fr_min[2];
        }

        // for (size_t i = 0; i < 3; i++)
        // {
        //     if (arm.current_joint_vel[i] > cut)
        //     arm.joint_control_tau[i] += static_fr_max[i];
        //     if (arm.current_joint_vel[i] < -cut)
        //     arm.joint_control_tau[i] += static_fr_min[i];
            
        // }
        // std::cout << arm.current_joint_vel[2] << std::endl;
        // 由于关节和电机的方向有可能不一致，所以要转换一下
        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // 电机反馈回来的力矩是pid的控制力矩+重力补偿力矩

        motor.kp[0] = 0;
        motor.kd[0] = 0.0;

        motor.kp[1] = 0;
        motor.kd[1] = 0.0;

        motor.kp[2] = 0;
        motor.kd[2] = 0.0;

        // motor.ControlMotors(port,
        //                     motor.desir_motor_pos,
        //                     motor.desir_motor_vel,
        //                     motor.kp,
        //                     motor.kd,
        //                     motor.motor_control_tau);
 
        motor.ControlMotors_g(port,
                              motor.desir_motor_pos,
                              motor.desir_motor_vel,
                              motor.kp,
                              motor.kd,
                              motor.motor_control_tau,
                              arm.desir_grp);

        // std::cout << "impdence-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "impdence-end_force  : " << arm.link_mass[1] << " " << end_force[1] << " " << end_force[2]<< std::endl;

        // float ts_tau1 = motor.current_motor_tau[2]/q3_e;
        // std::cout << "impdence-desir_grp: " << arm.desir_grp << " " << 0 << " " << motor.current_motor_tau[2]<< std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}

// 不能对速度滤波，再输入电机
void ArmState::admittance(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{
    static float ts_time = 0;
    float kp1 = 95;
    float kp2 = 95;
    float kp3 = 55;
    float kd1 = 0.93;
    float kd2 = 0.93;
    float kd3 = 0.53;

    // while (true)
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        arm.admittance(arm.desir_end_efect_pos,
                       arm.current_joint_pos,
                       arm.current_joint_vel,
                       arm.end_force_orientation,
                       arm.end_force_value,
                       arm.desir_end_efect_acc,
                       arm.desir_joint_pos,
                       arm.desir_joint_vel,
                       arm.joint_admittance_tau);

        for (size_t i = 0; i < 3; i++)
        {
            if (abs(arm.current_joint_vel[i]) <= 0.01)
                arm.joint_admittance_tau[i] += signum(arm.joint_admittance_tau[i]) * 0.1;
        }

        arm.joint_control_tau[0] = arm.joint_admittance_tau[0] + arm.joint_gravity_tau[0];

        arm.joint_control_tau[1] = arm.joint_admittance_tau[1] + arm.joint_gravity_tau[1];

        arm.joint_control_tau[2] = arm.joint_admittance_tau[2] + arm.joint_gravity_tau[2];

        // 由于关节和电机的方向有可能不一致，所以要转换一下
        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // motor.compensate_static_friction_through_vel(motor.current_motor_vel, motor.motor_control_tau);
        motor.kp[0] = 20.0;
        motor.kd[0] = 2.01;
    
        motor.kp[1] = 110.0;
        motor.kd[1] = 2.01;
    
        motor.kp[2] = 90.0;
        motor.kd[2] = 1.51;



        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.current_motor_vel, // 使用实时速度是最好的！！！！
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);
        // std::cout << "admittance-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "admittance-desir_motor_pos  : " << motor.desir_motor_pos[0] << " " << motor.desir_motor_pos[1] << " " << motor.desir_motor_pos[2]<< std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}

void ArmState::gravity(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{
    // 配置滤波参数
    motor.KalmanFilter(1e-6, 1e-2, 1, 0, 0.03);

    // while (true)
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        // 卡尔曼滤波，处理电机速度值
        for (size_t i = 0; i < 3; i++)
        {
            motor.current_motor_vel[i] = motor.update(motor.current_motor_vel[i]);
        }

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

        arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        arm.joint_control_tau[0] = arm.joint_gravity_tau[0];

        arm.joint_control_tau[1] = arm.joint_gravity_tau[1];

        arm.joint_control_tau[2] = arm.joint_gravity_tau[2];

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        motor.compensate_static_friction_through_vel(motor.current_motor_vel, motor.motor_control_tau);

        float gravity_kp[3] = {0.0, 0.0, 0.0};
        float gravity_kd[3] = {0.0, 0.0, 0.0};

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            gravity_kp,
                            gravity_kd,
                            motor.motor_control_tau);

        // std::cout << "gravity: " << motor.motor_control_tau[0] << " " << motor.motor_control_tau[1] << " " << motor.motor_control_tau[2]<< std::endl;
        // std::cout << "arm.current_joint_pos: " << arm.current_joint_pos[0] << " " << arm.current_joint_pos[1] << " " << arm.current_joint_pos[2] << std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}

void ArmState::initialize(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{

    float ts_time = 0;
    /**
     * 初始姿态
     */

    // arm.desir_end_efect_pos[0] = 0.21 + 0.03 * sin(3 * ts_time / 1000.0);

    // arm.desir_end_efect_pos[1] = 0.0 + 0.03 * cos(3 * ts_time / 1000.0);

    arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

    std::cout << "获取当前关节位置>>>" << std::endl;

    /* 获取当前关节位置 */
    for (size_t i = 0; i < 50; i++)
    {
        auto parsedData_ = port.getReceivedData();

        motor.usbDataToMotorState(parsedData_, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

        for (size_t i = 0; i < 3; i++)
        {
            motor.kp[i] = 0.0;
            motor.kd[i] = 0.0;
            motor.tau[i] = 0.0;
        }
        motor.ControlMotors(port, motor.desir_motor_pos, motor.desir_motor_vel, motor.kp, motor.kd, motor.tau);

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 测试**********************************************************************************************************************
    float current_joint_pos_temp[3] = {arm.current_joint_pos[0], arm.current_joint_pos[1], arm.current_joint_pos[2]};
    float desir_joint_pos_temp[3] = {arm.desir_joint_pos[0], arm.desir_joint_pos[1], arm.desir_joint_pos[2]};
    // 测试**********************************************************************************************************************

    motor.kp[0] = 20.0;
    motor.kd[0] = 2.01;

    motor.kp[1] = 50.0;
    motor.kd[1] = 2.01;

    motor.kp[2] = 40.0;
    motor.kd[2] = 1.51;


    // motor.kp[0] = 10.0;
    // motor.kd[0] = 2.01;

    // motor.kp[1] = 40.0;
    // motor.kd[1] = 2.01;

    // motor.kp[2] = 30.0;
    // motor.kd[2] = 1.71;


    

    /* 前往期望位置 */
    while (!arm.now2aim(current_joint_pos_temp, desir_joint_pos_temp, arm.desir_joint_pos, 0, 50))
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        arm.joint_control_tau[0] = arm.joint_gravity_tau[0];

        arm.joint_control_tau[1] = arm.joint_gravity_tau[1];

        arm.joint_control_tau[2] = arm.joint_gravity_tau[2];

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // std::cout << "init arm: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2];

        // std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "到达目标位置>>>" << std::endl;

}

void ArmState::joint_initialize(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{

    float ts_time = 0;

    std::cout << "arm.获取当前关节位置: " << std::endl;

    /* 获取当前关节位置 */
    for (size_t i = 0; i < 50; i++)
    {
        auto parsedData_ = port.getReceivedData();

        motor.usbDataToMotorState(parsedData_, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        for (size_t i = 0; i < 3; i++)
        {
            motor.kp[i] = 0.0;
            motor.kd[i] = 0.0;
            motor.tau[i] = 0.0;
        }
        motor.ControlMotors(port, motor.desir_motor_pos, motor.desir_motor_vel, motor.kp, motor.kd, motor.tau);

        std::cout << "current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2] << std::endl;
        std::cout << "desir_joint_pos  : " << arm.desir_joint_pos[0] << " " << arm.desir_joint_pos[1] << " " << arm.desir_joint_pos[2] << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 测试**********************************************************************************************************************
    float current_joint_pos_temp[3] = {arm.current_joint_pos[0], arm.current_joint_pos[1], arm.current_joint_pos[2]};
    float desir_joint_pos_temp[3] = {arm.desir_joint_pos[0], arm.desir_joint_pos[1], arm.desir_joint_pos[2]};
    // 测试**********************************************************************************************************************

    motor.kp[0] = 30.0;
    motor.kd[0] = 2.01;

    motor.kp[1] = 80.0;
    motor.kd[1] = 2.51;

    motor.kp[2] = 60.0;
    motor.kd[2] = 1.71;


    /* 前往期望位置 */
    while (!arm.now2aim(current_joint_pos_temp, desir_joint_pos_temp, arm.desir_joint_pos, 0, 50))
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        // 实时重力补偿
        arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
                                    arm.current_joint_vel,
                                    arm.current_joint_acc,
                                    arm.joint_gravity_tau);

        arm.joint_control_tau[0] = arm.joint_gravity_tau[0];

        arm.joint_control_tau[1] = arm.joint_gravity_tau[1];

        arm.joint_control_tau[2] = arm.joint_gravity_tau[2];

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // std::cout << "init arm: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2];

        // std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "到达目标位置 " << std::endl;
}

void ArmState::joint_auto_servo(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{
    // while (true)
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        arm.fk(arm.current_joint_pos, arm.current_end_efect_pos); // 更新末端实时位置

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);
    }
}


/// @brief 示教
/// @param port 
/// @param arm 
/// @param motor 
void ArmState::teach(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor)
{
    // static bool recording_flag = false; //时间到了录制结束
    // static bool reset = false;          //时间到了复位结束
    // static bool playback_flag = false;  //重播到最后一个元素重播结束

    static int recording_count = 0; 
    static int reset_count  = 0;          
    static int playback_count  = 0;  

    float reset_steps = 1000; //复位步数
    // float motor1_reset_step = 0; //复位步长
    // float motor2_reset_step = 0; //复位步长
    // float motor3_reset_step = 0; //复位步长
    /**
     * 0: 录制
     * 1：复位
     * 2：回放
     */
    static uint8_t state = 0;

    switch (state)
    {
    case 0:
        {
            recording_count++;
            if(recording_count < (motor.record_count))
            {
                gravity(port, arm, motor);

                motor.motor1_record_pos[recording_count] = motor.current_motor_pos[0];
                motor.motor2_record_pos[recording_count] = motor.current_motor_pos[1];
                motor.motor3_record_pos[recording_count] = motor.current_motor_pos[2];

                std::cout << "arm.current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2] << std::endl;


                std::cout << "motor.motor_record_pos: " << motor.motor1_record_pos[recording_count] << " " << motor.motor2_record_pos[recording_count] << " " << motor.motor3_record_pos[recording_count] << std::endl;

                // gravity(port, arm, motor);

                std::cout << "录制>>>" << std::endl;
            }
            else{
                recording_count = 0;
                state = 1;
            }
        }
        break;

    case 1:
        {
            //当前位置 motor1_record_pos[motor.record_count-1]
            //期望位置 motor1_record_pos[0]
            //距离
            //步长

            reset_count++;

            if (reset_count < reset_steps)
            {
                float motor1_distance = motor.motor1_record_pos[1] - motor.motor1_record_pos[motor.record_count-1];
                float motor2_distance = motor.motor2_record_pos[1] - motor.motor2_record_pos[motor.record_count-1] ;
                float motor3_distance = motor.motor3_record_pos[1] - motor.motor3_record_pos[motor.record_count-1] ;

                // std::cout << "motor.motor_record_pos0: " << motor.motor1_record_pos[0] << " " << motor.motor2_record_pos[0]  << " " << motor.motor3_record_pos[0]  << std::endl;
                // std::cout << "motor.motor_record_pos : " << motor.motor1_record_pos[motor.record_count-1] << " " << motor.motor2_record_pos[motor.record_count-1]  << " " << motor.motor3_record_pos[motor.record_count-1]  << std::endl;


                float motor1_reset_step = motor1_distance / reset_steps;
                float motor2_reset_step = motor2_distance / reset_steps;
                float motor3_reset_step = motor3_distance / reset_steps;

                motor.desir_motor_pos[0] = motor.motor1_record_pos[motor.record_count - 1] + motor1_reset_step * reset_count;
                motor.desir_motor_pos[1] = motor.motor2_record_pos[motor.record_count - 1] + motor2_reset_step * reset_count;
                motor.desir_motor_pos[2] = motor.motor3_record_pos[motor.record_count - 1] + motor3_reset_step * reset_count;

                for (size_t i = 0; i < 3; i++)
                {
                    motor.motor_control_tau[i] = 0;
                    motor.desir_motor_vel[i] = 0;
                }
                
             
                // std::cout << "motor_distance: " << motor1_distance << " " << motor2_distance << " " << motor3_distance << std::endl;
                // std::cout << "motor_reset_step: " << motor1_reset_step << " " << motor2_reset_step << " " << motor3_reset_step << std::endl;


                motor.ControlMotors(port,
                                    motor.desir_motor_pos,
                                    motor.desir_motor_vel,
                                    motor.kp,
                                    motor.kd,
                                    motor.motor_control_tau);

                std::cout << "复位>>>" << std::endl;
            }
            else
            {
                reset_count = 0;
                state = 2;
            }
        }
        break;

    case 2:
        {
            playback_count++;
            if(playback_count < (motor.record_count))
            {
                motor.desir_motor_pos[0] = motor.motor1_record_pos[playback_count];
                motor.desir_motor_pos[1] = motor.motor2_record_pos[playback_count];
                motor.desir_motor_pos[2] = motor.motor3_record_pos[playback_count];

                for (size_t i = 0; i < 3; i++)
                {
                    motor.motor_control_tau[i] = 0;
                    motor.desir_motor_vel[i] = 0;
                }

                motor.ControlMotors(port,
                                    motor.desir_motor_pos,
                                    motor.desir_motor_vel,
                                    motor.kp,
                                    motor.kd,
                                    motor.motor_control_tau);

                std::cout << "回放>>>" << std::endl;
            }
            else
            {
                playback_count = 0;
                state = 1;
            }
        }
        break;

    default:
        break;
    }

 

}
