#ifndef MODESTSTEMSCHINE_H
#define MODESTSTEMSCHINE_H

#include <iostream>
#include "serial_port.h"
#include "light_lift_arm_3dof.h"
#include "motor_control.h"


enum class State {
    AutoServo,      //电机内部mit
    ManualServo,    //手动通过纯力矩控制电机
    Impdence,       //阻抗
    Admittance,      //导纳
    Gravity,        //重力补偿
    JointAutoServo, //关节自动控制,注意要初始化关节位置
    Zero ,        //设置零点
    Teach        //示教
};

class ArmState {
public:
    ArmState();

    void modeTransition(State newState);
    void run(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void autoServo(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void manualServo(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void manualServo_STSMC(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void manualServo_CTSMC(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void impdence(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void admittance(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void gravity(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void initialize(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void joint_initialize(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void joint_auto_servo(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);
    void teach(serialport::SerialPortWrapper &port, LlArm3dof &arm, MotorControl &motor);



private:
    State currentState;
};




#endif // ARMCONTROL_H

