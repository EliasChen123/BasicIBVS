
#include <rtt/os/main.h>
#include <rtt/Logger.hpp>
#include <rtt/Activity.hpp>
#include <rtt/base/ActivityInterface.hpp>
#include <rtt/base/RunnableInterface.hpp>

#include <kdl/frames.hpp>
#include <kdl/utilities/utility.h>

#include <memory>
#include <unistd.h>
#include <csignal>
#include <stdlib.h>

/*
#include "commu_test/serial_port.h"
#include "commu_test/motor_control.h"
#include "commu_test/state_machine.h"*/


#include "commu/serial_port_interface.hpp"
#include "motor_control/motor_control.hpp"
#include "thread/SerialRunnable.hpp"
#include "thread/RobotRunnable.hpp"
#include "data_pool/data_pool.hpp"
#include "robot/robot.hpp"

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#define ROBOT_MOVE_INTERP 0.001
#define RECEIVER_INTERP 0.001

using namespace LibSerial;
using namespace std::chrono_literals;

RSR::SerialPort::Ptr serialPtr = 
		std::make_shared<RSR::SerialPort>("/dev/ttyACM0", LibSerial::BaudRate::BAUD_921600);

RSR::SerialPortInterface::Ptr serialInterfacePtr = 
		std::make_shared<RSR::SerialPortInterface>(serialPtr);		

RSR::MotorControl::Ptr motorPtr = 
		std::make_shared<RSR::MotorControl>(serialInterfacePtr);

RSR::DataPool::Ptr dataPoolPtr = 
        std::make_shared<RSR::DataPool>();

RSR::Robot::Ptr robotPtr = 
        std::make_shared<RSR::Robot>(motorPtr, dataPoolPtr);

void signalHandler(int signum)
{
    std::cout << "\n >>>>>>接收到SIGINT(" << signum << ")，准备终止程序。" << std::endl;
    motorPtr->DisableMotors();
    sleep(1);
	serialPtr->close();
    std::exit(signum); // 程序正常退出，将触发atexit的调用
}

int ORO_main(int argc, char **argv)
{

	std::signal(SIGINT, signalHandler);

	motorPtr->EnableMotors();

	sleep(1);

	motorPtr->EnableMotors();

	sleep(1);

	//////////////////////////////////////
	// uart thread
	std::unique_ptr<RTT::base::RunnableInterface> uart_runner = std::make_unique<RSR::SerialRunnable>(serialInterfacePtr, dataPoolPtr);
	std::unique_ptr<RTT::base::ActivityInterface> uart_act =
			std::make_unique<RTT::Activity>(ORO_SCHED_RT, RTT::os::HighestPriority - 3, 0.001, uart_runner.get(), "uart");
	uart_act->start();

    robotPtr->initialize();

    std::unique_ptr<RTT::base::RunnableInterface> robot_runner = std::make_unique<RSR::RobotRunnable>(robotPtr);
	std::unique_ptr<RTT::base::ActivityInterface> robot_act =
			std::make_unique<RTT::Activity>(ORO_SCHED_RT, RTT::os::HighestPriority - 3, 0.001, robot_runner.get(), "robot");
    robot_act->start();

	while(1){

		sleep(1);
	}
}





/*
float get_duration_realtime(float time_step, float max_time)
{

    static float loop_count = 0;
    static float realtime = 0;

    realtime = (loop_count)*time_step;

    loop_count++;

    if (realtime > max_time)
    {
        loop_count = 0;
        realtime = 0;
    }

    return realtime;
}

int ORO_main(int argc, char **argv)
{

	std::signal(SIGINT, signalHandler);


	serialport::SerialPortWrapper serial;

	MotorControl motorControl(serial);

	ArmState control_mode;

	LlArm3dof arm_3dof;

	motorControl.EnableMotors(serial);

    std::cerr << "ENABLE MOTORS>>>>>>>>>>>>>>>>>>>>>>>." << std::endl;

    sleep(1);

    motorControl.EnableMotors(serial);

    // 启动接收线程
    serial.startReceiving();

	while (1)
	{

		float realtime = get_duration_realtime(0.001, 3.1415926 * 2);

        // 实时更新期望位置
        realtime = realtime * 0.0;

        float rate = 2, amplitude = 0.03;

        // 外部话题控制，要把内部轨迹更新关闭

        arm_3dof.desir_end_efect_pos[0] = 0.21 + amplitude * sin(rate * realtime);

        arm_3dof.desir_end_efect_pos[1] = 0.0 + amplitude * cos(rate * realtime);

        arm_3dof.desir_end_efect_pos[2] = 0.0 + amplitude * cos(rate * realtime);

        arm_3dof.desir_end_efect_vel[0] = amplitude * rate * cos(rate * realtime);

        arm_3dof.desir_end_efect_vel[1] = -amplitude * rate * sin(rate * realtime);

        arm_3dof.desir_end_efect_vel[2] = -amplitude * rate * sin(rate * realtime);

        arm_3dof.desir_end_efect_acc[0] = -amplitude * rate * rate * sin(rate * realtime);

        arm_3dof.desir_end_efect_acc[1] = -amplitude * rate * rate * cos(rate * realtime);

        arm_3dof.desir_end_efect_acc[2] = -amplitude * rate * rate * cos(rate * realtime);

        try
        {
            //     AutoServo,      //电机内部mit
            //     ManualServo,    //手动通过纯力矩控制电机
            //     Impdence,       //阻抗
            //     Admittance,     //导纳
            //     Gravity,        //重力补偿
            //     JointAutoServo, //关节自动控制,注意要初始化关节位置
            //     Zero            //设置零点
            //     Teach           //示教

            control_mode.modeTransition(State::Impdence);

            control_mode.run(serial, arm_3dof, motorControl); // 循环运行
        }
        catch (const std::exception &e)
        {
            std::cerr << "程序异常：" << e.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

		
        // std::cout << "hi" << std::endl;
		// usleep(500);
	}

	return 0;
}
	*/