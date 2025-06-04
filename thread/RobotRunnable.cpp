
#include "RobotRunnable.hpp"


namespace RSR
{
    
    RobotRunnable::RobotRunnable(Robot::Ptr Ptr)
        :m_robot_ptr(Ptr)
    {

    }

    RobotRunnable::~RobotRunnable(){

    }

    bool RobotRunnable::initialize(){
        
        return true;
    }

    float RobotRunnable::get_duration_realtime(float time_step, float max_time) {

        static float loop_count = 0;
        static float realtime = 0;
    
        realtime = (loop_count) * time_step;
    
        loop_count++;
    
        if (realtime > max_time)
        {
            loop_count = 0;
            realtime = 0;
        }
    
        return realtime;
    }

    void RobotRunnable::step(){
        float realtime = get_duration_realtime(0.001, 3.1415926 * 2);
        m_time_cnt++;
        float rate = 2 , amplitude = 0.03 ;
        float amplitude_x = 0.03;
        float amplitude_y = 0.03;
        float amplitude_z = 0.03;

        //外部话题控制，要把内部轨迹更新关闭

        m_robot_ptr->m_desir_end_efect_pos[0] = 0.22 + amplitude_x * sin(rate * realtime);
        m_robot_ptr->m_desir_end_efect_pos[1] = 0.0 + amplitude_y * cos(rate * realtime);
        m_robot_ptr->m_desir_end_efect_pos[2] = 0.03 + amplitude_z * cos(rate * realtime);

        // float joint[3];
        // joint[0] = ;
        // joint[1] = ;
        // joint[2] = ;
        // float pos[3];
        // m_robot_ptr->fk(joint, pos);
        // m_robot_ptr->m_desir_end_efect_pos[0] = pos[0];
        // m_robot_ptr->m_desir_end_efect_pos[1] = pos[1];
        // m_robot_ptr->m_desir_end_efect_pos[2] = pos[2];




        // m_robot_ptr->m_desir_end_efect_vel[0] = amplitude_x * rate *cos(rate * realtime);
        // m_robot_ptr->m_desir_end_efect_vel[1] = -amplitude_y * rate *sin(rate * realtime);
        // m_robot_ptr->m_desir_end_efect_vel[2] = -amplitude_z * rate *sin(rate * realtime);
        // m_robot_ptr->m_desir_end_efect_acc[0] = -amplitude_x * rate *rate *sin(rate * realtime);
        // m_robot_ptr->m_desir_end_efect_acc[1] = -amplitude_y * rate *rate *cos(rate * realtime);
        // m_robot_ptr->m_desir_end_efect_acc[2] = -amplitude_z * rate *rate *cos(rate * realtime);

        m_robot_ptr->m_desir_end_efect_vel[0] = 0.1;
        m_robot_ptr->m_desir_end_efect_vel[1] = -0.1;
        m_robot_ptr->m_desir_end_efect_vel[2] = -0.1;
        m_robot_ptr->m_desir_end_efect_acc[0] = -0.02;
        m_robot_ptr->m_desir_end_efect_acc[1] = -0.02;
        m_robot_ptr->m_desir_end_efect_acc[2] = -0.02;

        //关节控制的期望位置
        m_robot_ptr->m_desir_joint_pos[1] = 0.8 + 0.6 * sin(1 * realtime);
        m_robot_ptr->m_desir_joint_pos[2] = -1.5 ;

        

        // m_robot_ptr->impdence();

        m_robot_ptr->autoServo();

    }

    void RobotRunnable::finalize(){

    }

    void RobotRunnable::loop(){

    }

} // namespace Tiromu
