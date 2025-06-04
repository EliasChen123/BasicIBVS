#include "robot.hpp"
#include <cmath>
#include <iomanip>

namespace RSR
{

Robot::Robot(MotorControl::Ptr ptr, RSR::DataPool::Ptr dataPtr)
    :m_motor_ptr(ptr),
    m_data_pool_ptr(dataPtr),
    m_record_data_file("out.txt", std::ios::trunc)
{
    m_motor_state.reserve(ARM_DOF*3);
}

Robot::~Robot() {}

void Robot::initialize()
{
    /*init communication*/
    float kp[3] = {0.0,0.0,0.0};
    float kd[3] = {0.0,0.0,0.0};
    float tau[3] = {0.0,0.0,0.0};
    for (size_t i = 0; i < 50; i++)
    {
        float pos[3] = {0.0,0.0,0.0};
        float vel[3] = {0.0,0.0,0.0};
        m_motor_ptr->ControlMotors(pos, vel, kp, kd, tau);

        bool ok = updatRobotJoint(); 
        if(!ok)
        {            
            continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    auto parsedData = m_data_pool_ptr->GetUsbParsedData();
    if(parsedData.empty())
    {
        std::cerr << "commu failed, exit" << std::endl;
        exit(0);
    }

    if(!parsedData.empty())
    {
        std::cout << "uart data flow ok" << std::endl;
    }

    /*run to init pos*/
    float target_pos[3] = {0.22, 0.03, 0.06};
    float target_joint[3];
    Ik_3Dof(target_pos, target_joint);

    float start[3] = {m_current_joint_pos[0], m_current_joint_pos[1], m_current_joint_pos[2]};
    float end[3] = {target_joint[0], target_joint[1], target_joint[2]};

    float interp_pos[3] = {0.0,0.0,0.0};
    float interp_vel[3] = {0.0,0.0,0.0};
    float interp_acc[3] = {0.0,0.0,0.0};

    float motor_pos[3] = {0.0,0.0,0.0};
    float motor_vel[3] = {0.0,0.0,0.0};
    float motor_acc[3] = {0.0,0.0,0.0};
    float motor_tau[3] = {0.0,0.0,0.0};

    while (!now2aim(start, end, interp_pos))
    {

        updatRobotJoint(); 

        float gravity[3];
        Closed_Arm_Modle_decoup(gravity);

        joint2motor(interp_pos,
                    interp_vel,
                    interp_acc,
                    gravity,
                    motor_pos,
                    motor_vel,
                    motor_acc,
                    motor_tau);

        kp[0] = 20.0; kp[1] = 50.0; kp[2] = 40.0; 
        kd[0] = 2.01; kd[1] = 2.01; kd[2] = 1.51; 
        
        m_motor_ptr->ControlMotors( motor_pos,
                                    motor_vel,
                                    kp,
                                    kd,
                                    motor_tau);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));        

    }

    std::cout << "到达目标位置>>>" << std::endl;

}

void Robot::impdence()
{

    bool ok = updatRobotJoint(); 
    if(!ok)
    {    
        std::cout << "impdence updatRobotJoint failed!" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));        
        return;
        
    }

    float end_pos[3] = {0.0,0.0,0.0};
    fk(m_current_joint_pos, end_pos); 

    Ik_3Dof(m_desir_end_efect_pos, m_desir_joint_pos);

    float end_force[3] = {0};

    static int cnt = 20;
    static bool first = true;
    static float last_pos[3];
    if(first){
        last_pos[0] = end_pos[0];
        last_pos[1] = end_pos[1];
        last_pos[2] = end_pos[2];
        first = false;
    }

    float joint_impedance_tau[3] = {0.0,0.0,0.0};

    if(--cnt == 0){
        float delta[3];
        delta[0] = end_pos[0] - last_pos[0];
        delta[1] = end_pos[1] - last_pos[1];
        delta[2] = end_pos[2] - last_pos[2];

        end_impedance_control(m_desir_end_efect_pos,
            m_desir_end_efect_vel,
            m_current_joint_pos,
            m_current_joint_vel,
            joint_impedance_tau,
                4, 120,
            end_force,
            delta);

        cnt = 20;

        last_pos[0] = end_pos[0];
        last_pos[1] = end_pos[1];
        last_pos[2] = end_pos[2];

        m_desir_end_efect_pos[0] = end_pos[0];
        m_desir_end_efect_pos[1] = end_pos[1];
        m_desir_end_efect_pos[2] = end_pos[2];

    }

    float gravity[3] = {0.0,0.0,0.0};
    Closed_Arm_Modle_decoup(gravity);

    float joint_tau[3] = {};

    joint_tau[0] = gravity[0] + 1.0 * joint_impedance_tau[0];
    joint_tau[1] = gravity[1] + 1.0 * joint_impedance_tau[1];
    joint_tau[2] = gravity[2] + 1.0 * joint_impedance_tau[2];

    float static_fr_max[3] = { 0.1,  0.15,  0.05};
    float static_fr_min[3] = {-0.1, -0.15, -0.05};

    float cut = 0.0;

    if(std::abs(m_current_joint_vel[0]) < 0.1){
        joint_tau[0] *= 1.2;
    }
    if(std::abs(m_current_joint_vel[0]) > 0.3){
        if (m_current_joint_vel[0] > cut)
        joint_tau[0] += static_fr_max[0];
        if (m_current_joint_vel[0] < -cut)
        joint_tau[0] += static_fr_min[0];
    }
    
    if(std::abs(m_current_joint_vel[1]) < 0.1){
        joint_tau[1] *= 1.2;
    }
    if(std::abs(m_current_joint_vel[1]) > 0.3){
        if (m_current_joint_vel[1] > cut)
        joint_tau[1] += static_fr_max[1];
        if (m_current_joint_vel[1] < -cut)
        joint_tau[1] += static_fr_min[1];
    }

    if(std::abs(m_current_joint_vel[2]) < 0.1){
        joint_tau[2] *= 1.2;
    }
    if(std::abs(m_current_joint_vel[2]) > 0.3){
        if (m_current_joint_vel[2] > cut)
        joint_tau[2] += static_fr_max[2];
        if (m_current_joint_vel[2] < -cut)
        joint_tau[2] += static_fr_min[2];
    }


    float joint_pos[3] = {0.0,0.0,0.0};
    float joint_vel[3] = {0.0,0.0,0.0};
    float joint_acc[3] = {0.0,0.0,0.0};

    float motor_pos[3] = {0.0,0.0,0.0};
    float motor_vel[3] = {0.0,0.0,0.0};
    float motor_acc[3] = {0.0,0.0,0.0};
    float motor_tau[3] = {0.0,0.0,0.0};

    joint2motor(joint_pos,
                joint_vel,
                joint_acc,
                joint_tau,
                motor_pos,
                motor_vel,
                motor_acc,
                motor_tau);

    float kp[3] = {0.0,0.0,0.0};
    float kd[3] = {0.0,0.0,0.0};
    float tau[3] = {0.0,0.0,0.0};

    m_motor_ptr->ControlMotors( motor_pos,
                                motor_vel,
                                kp,
                                kd,
                                motor_tau);

}

void Robot::autoServo()
{
    bool ok = updatRobotJoint(); 
    if(!ok)
    {    
        std::cout << "autoServo updatRobotJoint failed!" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));        
        return;
        
    }

    float end_pos[3] = {0.0,0.0,0.0};
    fk(m_current_joint_pos, end_pos);

    float gravity[3] = {0.0,0.0,0.0};
    Closed_Arm_Modle_decoup(gravity);

    float est_joint_load[3] = { 15 * (m_current_joint_tau[0] - gravity[0]),
                                15 * (m_current_joint_tau[1] - gravity[1]),
                                15 * (m_current_joint_tau[2] - gravity[2])};

    float end_effect_tau[3] = {0.0, 0.0, 0.0};
    joint2end_force(m_current_joint_pos, est_joint_load, end_effect_tau);

    Ik_3Dof(m_desir_end_efect_pos, m_desir_joint_pos);
    
    float AV[6] = {0};

    Dof_3_Arm_iAV_Sca(m_desir_joint_pos, m_desir_end_efect_vel, m_desir_end_efect_acc, AV);

    for (size_t i = 0; i < 3; i++)
    {
        m_desir_joint_vel[i] = AV[i];
        m_desir_joint_acc[i] = AV[3 + i];
    }

    float joint_tau[3] = {0.0, 0.0, 0.0};
    joint_tau[0] = 1.0 * gravity[0];
    joint_tau[1] = 1.0 * gravity[1];
    joint_tau[2] = 1.0 * gravity[2];

    float motor_pos[3] = {0.0, 0.0, 0.0};
    float motor_vel[3] = {0.0, 0.0, 0.0};
    float motor_acc[3] = {0.0, 0.0, 0.0};
    float motor_tau[3] = {0.0, 0.0, 0.0};

    joint2motor(m_desir_joint_pos,
                    m_desir_joint_vel,
                    m_desir_joint_acc,
                    joint_tau,
                    motor_pos,
                    motor_vel,
                    motor_acc,
                    motor_tau);

    // 用期望的电机速度，提前补偿静摩擦
    m_motor_ptr->compensate_static_friction_through_vel(motor_vel, motor_tau);

    float kp[3] = {20.0, 50.0, 40.0};
    float kd[3] = {2.01, 2.01, 1.51};

    m_motor_ptr->ControlMotors( motor_pos,
                                motor_vel,
                                kp,
                                kd,
                                motor_tau);


    float force = sqrt(end_effect_tau[0] * end_effect_tau[0] +
        end_effect_tau[1] * end_effect_tau[1] + 
        end_effect_tau[2] * end_effect_tau[2]);

    // float estimate_end_load = force / 0.00981;
    // std::cout << "autoServo-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
    // std::cout << "autoServo-current_end_pos  : " << arm.current_end_efect_pos[0] << " " << arm.current_end_efect_pos[1] << " " << arm.current_end_efect_pos[2]<< std::endl;
    // std::cout << "autoServo-current_motor_tau: " << motor.current_motor_tau[0] << " " << motor.current_motor_tau[1] << " " << motor.current_motor_tau[2]<< std::endl;
    // std::cout << std::endl;
    // std::cout << "autoServo-estimate_end_load: " << arm.current_end_efect_tau[2] << std::endl;

    // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
}

void Robot::Ik_3Dof(float end_effect_pos[3], float q[3])
{
    float FuY, r, a, b, c, elbow, m1, m2, m3, a1, a2, a3, detlta, q21, q22;
    float q1, q2, q3;
    float x = end_effect_pos[0];
    float y = end_effect_pos[1];
    float z = end_effect_pos[2];

    a2 = m_link_lenth[0];
    a3 = m_link_lenth[1];

    /******************************************************/
    // 把加爪的位置转换为机械臂末端的位置
    q1 = atan2(y, x);
    x = end_effect_pos[0];
    y = end_effect_pos[1];
    z = end_effect_pos[2];
    /******************************************************/

    // std::cout << "autoServo-desir_joint_xyz: " << x << " " << y << " " << z<< std::endl;
       
    float arm_lenth = 0.35;
    if (sqrt(x * x + y * y + z * z) >= arm_lenth)
    {
        std::cout << "c n m 超出工作空间！！" << std::endl;
        return;
    }

    if (abs(z) <= 0.0001)
    {

        float r2 = x * x + y * y;
        r = sqrt(r2);
        q2 = acos((a2 * a2 + r2 - a3 * a3) / (2 * r * a2));
        q3 = acos((a2 * a2 + a3 * a3 - r2) / (2 * a3 * a2)) - 3.1415926; // ����
        // q1 = atan2(y, x);

        // std::cout << "q123z: " << q1<< " " << q2 << " " << q3<< " "<< x << " " << y << " " << z<< std::endl;
    }
    else
    {
        /*���˶�ѧ*/
        elbow = -1;
        FuY = 1;
        if (y < 0)
        {
            FuY = -1;
        }
        else
        {
            FuY = 1;
        }
        r = x * x + y * y + z * z;
        q3 = elbow * acos((r - (a2 * a2 + a3 * a3)) / (2 * a2 * a3)); // Console.WriteLine($"q3 = {q3}--��{i}��");

        // q1 = atan2(y, x); // Console.WriteLine($"q1 = {q1}--��{i}��");

        b = -a3 * cos(q1) * sin(q3);
        a = a2 * cos(q1) + a3 * cos(q1) * cos(q3);
        c = x;
        if (a == 0)
        {
            q21 = 3.1415926 / 2.0;
        }
        else
        {
            q21 = atan2(b, a);
        }
        if (c == 0)
        {
            q22 = FuY * 3.1415926 / 2.0;
        }
        else
        {
            q22 = atan2(sqrt(a * a + b * b - c * c), c);
        }
        q2 = q21 + q22; // Console.WriteLine($"q2 = {q2}--��{i}��");

        if (z < 0)
        q2 = q21 - q22;

        // std::cout << "q123 : " << q1<< " " << q2 << " " << q3<< " "<< x << " " << y << " " << z<< std::endl;
   
    // 判断 q2 是否为 NaN 并打印信息
    if (std::isnan(q2))
    {
        std::cerr << "Error: q2 is NaN" << std::endl;

        std::cout << "q123 : " << q1<< " " << q2 << " " << q3<< std::endl;

        std::cout << "xyz: " << x << " " << y << " " << z<< std::endl;

        std::cout << "q123 : " << a<< " " << b << " " << c<< " "<< q21 << " " << q22 << " " << 0<< std::endl;
   
        std::cout << "q22: " << a * a  << " " << b * b << " " << c * c<< std::endl;
    }



    }


    q[0] = q1;
    q[1] = q2;
    q[2] = q3;
}

void Robot::fk(float q[3], float X[3])
{
    float a1 = 0, a2 = 0.184, a3 = 0.186;
    a2 = m_link_lenth[0];
    a3 = m_link_lenth[1];

    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];

    X[0] = cos(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    X[1] = sin(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    X[2] = a2 * sin(q2) + a3 * sin(q2 + q3);
}

bool Robot::now2aim(float sta_pos[3], float end_pos[3], float now_pos[3])
{
    static float e[3] = {(end_pos[0] - sta_pos[0]), (end_pos[1] - sta_pos[1]), (end_pos[2] - sta_pos[2])};

    static int couter = 0;

    static float start_pos[3] = {sta_pos[0], sta_pos[1], sta_pos[2]};

    static float inter_step = 0;

    float time_couter = (float)couter / 1000.0;

    inter_step += 0.5 * 0.001 * sin(time_couter);

    for (size_t i = 0; i < 3; i++)
    {
        now_pos[i] = start_pos[i] + e[i] * inter_step;
    }

    if (couter >= 3140)
    {
        couter = 0;
        return true;
    }
    else
    {
        couter++;
        return false;
    }

}

void Robot::Closed_Arm_Modle_decoup(float tau[3])
{
    float Joint_Torque[9];
    float tao3, tao2, tao1, n11x, n11y, n11z, n22y, n22x, n33x, n33y, n33z;
    float q1, qd1, qdd1, q2, qd2, qdd2, q3, qd3, qdd3; // �ؽ�״̬
    float a1, a2, a3, m1, m2, m3;
    float Ixx1, Iyy1, Izz1, Ixx2, Izz2, Iyy2, Izz3, Ixx3, Iyy3;
    float M1_Q1, C1_Q1Q3, C1_Q1Q2, G2, C2_Q1Q1, C2_Q2Q2, C2_Q2Q3, C2_Q3Q3, M2_Q3, M2_Q2, G3, C3_Q1Q1, C3_Q2Q2, M3_Q3, M3_Q2;
    q1 = m_current_joint_pos[0];
    qd1 = m_current_joint_vel[0];
    qdd1 = m_current_joint_acc[0];

    q2 = m_current_joint_pos[1];
    qd2 = m_current_joint_vel[1];
    qdd2 = m_current_joint_acc[1];

    q3 = m_current_joint_pos[2];
    qd3 = m_current_joint_vel[2];
    qdd3 = m_current_joint_acc[2];

    a1 = 0.0;
    //   a2 = 0.184;
    //   a3 = 0.186;
    a2 = m_link_lenth[0];
    a3 = m_link_lenth[1];

    m1 = 0.0;
    m2 = 0.1;
    m3 = 0.055;

    m2 = m_link_mass[0];
    m3 = m_link_mass[1];

    //惯性张量很小，控制效果却很好
    Izz1 = 0.000035;
    Izz2 = 0.000067;
    Izz3 = 0.000048;
    Iyy1 = 0.000025;
    Iyy2 = 0.00006;
    Iyy3 = 0.000048;
    Ixx1 = 0.000025;
    Ixx2 = 0.000010;
    Ixx3 = 0.00005;

    G3 = a3 * m3 * cos(q2 + q3) * 9.81;
    C3_Q1Q1 = a3 * m3 * ((0.5 * a2 * sin(q3) + 0.5 * a3 * sin(2.0 * (q2 + q3)) + 0.5 * a2 * sin(2.0 * q2 + q3))) + 0.5 * Iyy3 * sin(2.0 * (q2 + q3));
    C3_Q2Q2 = a3 * m3 * a2 * sin(q3);
    M3_Q3 = (a3 * a3 * m3 + Izz3);
    M3_Q2 = (a3 * m3 * (a2 * cos(q3) + a3) + Izz3);
    tao3 = M3_Q2 * qdd2 + M3_Q3 * qdd3 + C3_Q1Q1 * qd1 * qd1 + C3_Q2Q2 * qd2 * qd2 + G3;

    G2 = tao3 + a2 * cos(q2) * (9.81 * m2 + 9.81 * m3);
    C2_Q1Q1 = a2 * (a2 * sin(2.0 * q2) * 0.5 * (m2 + m3) + a3 * m3 * (-0.5 * sin(q3) + 0.5 * sin(2.0 * q2 + q3))) + 0.5 * sin(2.0 * q2) * Iyy2;
    C2_Q2Q2 = -a2 * a3 * m3 * sin(q3);
    C2_Q2Q3 = -2.0 * a2 * a3 * m3 * sin(q3);
    C2_Q3Q3 = -a2 * a3 * m3 * sin(q3);
    M2_Q3 = a2 * a3 * m3 * cos(q3);
    M2_Q2 = (a2 * (a2 * m2 + a2 * m3 + a3 * m3 * cos(q3)) + Izz2);
    tao2 = M2_Q2 * qdd2 + M2_Q3 * qdd3 + C2_Q1Q1 * qd1 * qd1 + C2_Q2Q2 * qd2 * qd2 + C2_Q2Q3 * qd2 * qd3 + C2_Q3Q3 * qd3 * qd3 + G2;

    M1_Q1 = 0.500 * (a3 * a3 * m3 + a2 * a2 * (m2 + m3) + a2 * a2 * (m2 + m3) * cos(2.0 * q2) + a3 * m3 * (4.0 * a2 * cos(q2) * cos(q2 + q3) + a3 * cos(2.0 * (q2 + q3))) + 2.0 * sin(q2) * sin(q2) * Ixx2 + 2.0 * sin(q2 + q3) * sin(q2 + q3) * Ixx3 + 2.0 * Iyy1 + 2.0 * cos(q2) * cos(q2) * Iyy2 + 2.0 * cos(q2 + q3) * cos(q2 + q3) * Iyy3);
    C1_Q1Q3 = -(2.0 * sin(q2 + q3) * (a3 * m3 * (a2 * cos(q2) + a3 * cos(q2 + q3)) + cos(q2 + q3) * (-Ixx3 + Iyy3)));
    C1_Q1Q2 = -(a2 * a2 * (m2 + m3) * sin(2.0 * q2) + a3 * m3 * (a3 * sin(2.0 * (q2 + q3)) + 2.0 * a2 * sin(2.0 * q2 + q3)) + sin(2.0 * q2) * (-Ixx2 + Iyy2) + sin(2.0 * (q2 + q3)) * (-Ixx3 + Iyy3));
    tao1 = M1_Q1 * qdd1 + C1_Q1Q3 * qd1 * qd3 + C1_Q1Q2 * qd1 * qd2;

    tau[0] = tao1;
    tau[1] = tao2;
    tau[2] = tao3;
}

void Robot::joint2motor(float q[ARM_DOF], float dq[ARM_DOF], float ddq[ARM_DOF], float tau_joint[ARM_DOF], float motor_pos[ARM_DOF], float motor_vel[ARM_DOF], float motor_acc[ARM_DOF], float tau_motor[ARM_DOF])
{
    for (size_t i = 0; i < 3; i++)
    {
        motor_pos[i] = m_motor_direction[i] * q[i];
        motor_vel[i] = m_motor_direction[i] * dq[i];
        motor_acc[i] = m_motor_direction[i] * ddq[i];
        tau_motor[i] = m_motor_direction[i] * tau_joint[i];
    }
}

/**
 * 输入：
 *      期望的末端位置
 *      期望的末端速度
 *      当前的关节位置
 *      当前的关节速度
 *      刚度系数
 *      阻尼系数
 * 输出：
 *      当前的关节力矩
 * 
*/
void Robot::end_impedance_control(float desir_end_pos[3], float desir_end_vel[3], 
    float current_joint_pos[3], float current_joint_vel[3], 
    float joint_tor[3], float c, float k, float *e_state, float* delta)
{
float e[3], de[3], K[3], V[3];
float J11, J12, J13, J21, J22, J23, J31, J32, J33, X[3], dX[3], X_desired[3];
float m;
float Fx = 0;
float Fy = 0;
float Fz = 0;
float a1 = 0, a2 = 0.184, a3 = 0.186;

a2 = m_link_lenth[0];
a3 = m_link_lenth[1];

X_desired[0] = desir_end_pos[0];
X_desired[1] = desir_end_pos[1];
X_desired[2] = desir_end_pos[2];

float q1 = current_joint_pos[0];
float q2 = current_joint_pos[1];
float q3 = current_joint_pos[2];

J11 = -sin(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
J21 = cos(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
J31 = 0;
J12 = -cos(q1) * (a2 * sin(q2) + a3 * sin(q2 + q3));
J22 = -sin(q1) * (a2 * sin(q2) + a3 * sin(q2 + q3));
J32 = a2 * cos(q2) + a3 * cos(q2 + q3);
J13 = -a3 * cos(q1) * sin(q3 + q2);
J23 = -a3 * sin(q1) * sin(q3 + q2);
J33 = a3 * cos(q3 + q2);
//当前的末端位置
X[0] = cos(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
X[1] = sin(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
X[2] = a2 * sin(q2) + a3 * sin(q2 + q3);
// 当前的末端速度
dX[0] = current_joint_vel[0] * J11 + current_joint_vel[1] * J12 + current_joint_vel[2] * J13;
dX[1] = current_joint_vel[0] * J21 + current_joint_vel[1] * J22 + current_joint_vel[2] * J23;
dX[2] = current_joint_vel[0] * J31 + current_joint_vel[1] * J32 + current_joint_vel[2] * J33;

// 末端误差
e[0] = X_desired[0] - X[0];
e[1] = X_desired[1] - X[1];
e[2] = X_desired[2] - X[2];
de[0] = desir_end_vel[0] - dX[0];
de[1] = desir_end_vel[1] - dX[1];
de[2] = desir_end_vel[2] - dX[2];

if((abs(e[0])+abs(e[1])+abs(e[2]))>0.007) *e_state=1;
else *e_state=0;

m = 0;

float Fxt = 2*c * de[0] + 9*k * e[0] - 0.45; 
float Fyt = 2*c * de[1] + 9*k * e[1]; 
float Fzt = 2*c * de[2] + 9*k * e[2]; 


if(std::abs(Fyt) > 0.1){
m_fy_queue.push(Fyt);
}

if(m_fy_queue.size() < 10) 
{
Fy = 0;
}
else{
std::queue<float> temp = m_fy_queue;

std::vector<float> v;
while(!temp.empty()){
v.push_back(temp.front());
temp.pop();
}

std::sort(v.begin(), v.end());

if(std::abs(v[2]) > 0.5){

Fy = v[2];
// std::cout << v[2] << std::endl;
}

m_fy_queue.pop();
}

if(std::abs(Fxt) > 0.1){
m_fx_queue.push(Fxt);
}

if(m_fx_queue.size() < 10) 
{
Fx = 0;
}
else{
std::queue<float> temp = m_fx_queue;

std::vector<float> v;
while(!temp.empty()){
v.push_back(temp.front());
temp.pop();
}

std::sort(v.begin(), v.end());

if(std::abs(v[2]) > 0.5){
// if(v[2] > 0){
//     Fx = 6;
// }
// else
// {
//     Fx = -6;
// }
Fx = v[2];
// std::cout << v[2] << std::endl;
}

m_fx_queue.pop();
}

Fx = 0;
Fy = 0;
Fz = 0;

int max_range = 7;
int min_range = -7;
if(Fx > max_range)
{
Fx = max_range;
}
else if (Fx < min_range)
{
Fx = min_range;
}

if(Fy > max_range)
{
Fy = 5;
}
else if (Fy < min_range)
{
Fy = -5;
}

if(Fz > max_range)
{
Fz = max_range;
}
else if (Fz < min_range)
{
Fz = min_range;
}


e_state[0] = Fx;
e_state[1] = Fy;
e_state[2] = Fz;


//映射到关节
joint_tor[0] = J11 * Fx + J21 * Fy + J31 * Fz;
joint_tor[1] = J12 * Fx + J22 * Fy + J32 * Fz;
joint_tor[2] = J13 * Fx + J23 * Fy + J33 * Fz;

// joint_tor[2] = -joint_tor[2];
}

/**
 * 输入：关节力矩
 * 输出：末端力矩
 * */ 
void Robot::joint2end_force(float q[3], float joint_force[3], float end_force[3])
{
    float J11, J12, J13, J21, J22, J23, J31, J32, J33, a1, a2, a3;
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    a1 = 0;
    //   a2 = 0.184;
    //   a3 = 0.186;

    a2 = m_link_lenth[0];
    a3 = m_link_lenth[1];

    // joint_force[2] = -joint_force[2];

    J11 = -sin(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    J21 = cos(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    J31 = 0;

    J12 = -cos(q1) * (a2 * sin(q2) + a3 * sin(q2 + q3));
    J22 = -sin(q1) * (a2 * sin(q2) + a3 * sin(q2 + q3));
    J32 = a2 * cos(q2) + a3 * cos(q2 + q3);

    J13 = -a3 * cos(q1) * sin(q3 + q2);
    J23 = -a3 * sin(q1) * sin(q3 + q2);
    J33 = a3 * cos(q3 + q2);

    end_force[0] = joint_force[0] * J11 + joint_force[1] * J12 + joint_force[2] * J13;
    end_force[1] = joint_force[0] * J21 + joint_force[1] * J22 + joint_force[2] * J23;
    end_force[2] = joint_force[0] * J31 + joint_force[1] * J32 + joint_force[2] * J33;
}

/**
 * 输入： 
 *      关节位置
 *      末端速度
 *      末端加速度
 * 输出：
 *      关节速度
 *      关节加速度
*/
void Robot::Dof_3_Arm_iAV_Sca(float Joint[3], float V[3], float dV[3], float AV[6])
{
    float l2, l3, qdd1, qdd2, qdd3, qd1, qd2, qd3, j21, j22, j23, j31, j32, j33;
    float q[3] = {0.0, 0.0, 0.0};
    float end_vol[3] = {0.0, 0.0, 0.0};
    float A_wan[3] = {0.0, 0.0, 0.0};
    float jb1[3] = {0.0, 0.0, 0.0};
    float jb2[3] = {0.0, 0.0, 0.0};
    float jb3[3] = {0.0, 0.0, 0.0};
    float nj1[3] = {0.0, 0.0, 0.0};
    float nj2[3] = {0.0, 0.0, 0.0};
    float nj3[3] = {0.0, 0.0, 0.0};
    float omega01[3] = {0.0, 0.0, 0.0};
    float omega02[3] = {0.0, 0.0, 0.0};
    float v02[3] = {0.0, 0.0, 0.0};
    float v33[3] = {0.0, 0.0, 0.0};
    float v03[3] = {0.0, 0.0, 0.0};
    float db1[3] = {0.0, 0.0, 0.0};
    float db2[3] = {0.0, 0.0, 0.0};
    float db3[3] = {0.0, 0.0, 0.0};
    float ddxs[3] = {0.0, 0.0, 0.0};
    float outputArg1[6] = {0.0, 0.0, 0.0};

    l2 = m_link_lenth[0];
    l3 = m_link_lenth[1];

    q[0] = Joint[0];
    q[1] = Joint[1];
    q[2] = Joint[2];
    for (int i = 0; i < 3; i++)
    {
        end_vol[i] = V[i];
        A_wan[i] = dV[i];
    }

    jb1[0] = (-1) * l2 * cos(q[1]) * sin(q[0]) + (-1) * l3 * cos(q[1]) * cos(q[2]) * sin(q[0]) + l3 * sin(q[0]) * sin(q[1]) * sin(q[2]);
    jb1[1] = l2 * cos(q[0]) * cos(q[1]) + l3 * cos(q[0]) * cos(q[1]) * cos(q[2]) + (-1) * l3 * cos(q[0]) * sin(q[1]) * sin(q[2]);
    jb1[2] = 0;

    jb2[0] = (-1) * l2 * cos(q[0]) * sin(q[1]) + (-1) * l3 * cos(q[0]) * cos(q[2]) * sin(q[1]) + (-1) * l3 * cos(q[0]) * cos(q[1]) * sin(q[2]);

    jb2[1] = (-1) * l2 * sin(q[0]) * sin(q[1]) + (-1) * l3 * cos(q[2]) * sin(q[0]) * sin(q[1]) + (-1) * l3 * cos(q[1]) * sin(q[0]) * sin(q[2]);
    jb2[2] = l2 * cos(q[0]) * cos(q[0]) * cos(q[1]) + l3 * cos(q[0]) * cos(q[0]) * cos(q[1]) * cos(q[2]) + l2 * cos(q[1]) * sin(q[0]) * sin(q[0]) + l3 * cos(q[1]) * cos(q[2]) * sin(q[0]) * sin(q[0]) + (-1) * l3 * cos(q[0]) * cos(q[0]) * sin(q[1]) * sin(q[2]) + (-1) * l3 * sin(q[0]) * sin(q[0]) * sin(q[1]) * sin(q[2]);

    jb3[0] = (-1) * l3 * cos(q[0]) * cos(q[2]) * sin(q[1]) + (-1) * l3 * cos(q[0]) * cos(q[1]) * sin(q[2]);
    jb3[1] = (-1) * l3 * cos(q[2]) * sin(q[0]) * sin(q[1]) + (-1) * l3 * cos(q[1]) * sin(q[0]) * sin(q[2]);
    jb3[2] = l3 * cos(q[0]) * cos(q[0]) * cos(q[1]) * cos(q[2]) + l3 * cos(q[1]) * cos(q[2]) * sin(q[0]) * sin(q[0]) + (-1) * l3 * cos(q[0]) * cos(q[0]) * sin(q[1]) * sin(q[2]) + (-1) * l3 * sin(q[0]) * sin(q[0]) * sin(q[1]) * sin(q[2]);

    j21 = jb1[0] * jb1[0] + jb1[1] * jb1[1] + jb1[2] * jb1[2];
    j22 = jb1[0] * jb2[0] + jb1[1] * jb2[1] + jb1[2] * jb2[2];
    j23 = jb1[0] * jb3[0] + jb1[1] * jb3[1] + jb1[2] * jb3[2];
    j31 = jb1[0] * jb2[0] + jb1[1] * jb2[1] + jb1[2] * jb2[2];
    j32 = jb2[0] * jb2[0] + jb2[1] * jb2[1] + jb2[2] * jb2[2];
    j33 = jb2[0] * jb3[0] + jb2[1] * jb3[1] + jb2[2] * jb3[2];

    nj2[0] = j22 * jb1[0] + (-1) * j21 * jb2[0];
    nj2[1] = j22 * jb1[1] + (-1) * j21 * jb2[1];
    nj2[2] = j22 * jb1[2] + (-1) * j21 * jb2[2];

    nj3[0] = (-1) * j23 * j32 * jb1[0] + j22 * j33 * jb1[0] + j23 * j31 * jb2[0] + (-1) * j21 * j33 * jb2[0] + (-1) * j22 * j31 * jb3[0] + j21 * j32 * jb3[0];

    nj3[1] = (-1) * j23 * j32 * jb1[1] + j22 * j33 * jb1[1] + j23 * j31 * jb2[1] + (-1) * j21 * j33 * jb2[1] + (-1) * j22 * j31 * jb3[1] + j21 * j32 * jb3[1];

    nj3[2] = (-1) * j23 * j32 * jb1[2] + j22 * j33 * jb1[2] + j23 * j31 * jb2[2] + (-1) * j21 * j33 * jb2[2] + (-1) * j22 * j31 * jb3[2] + j21 * j32 * jb3[2];

    qd3 = (nj3[0] * end_vol[0] + nj3[1] * end_vol[1] + nj3[2] * end_vol[2]) * pow((jb3[0] * nj3[0] + jb3[1] * nj3[1] + jb3[2] * nj3[2]), -1);

    qd2 = (nj2[0] * ((-1) * jb3[0] * qd3 + end_vol[0]) + nj2[1] * ((-1) * jb3[1] * qd3 + end_vol[1]) + nj2[2] * ((-1) * jb3[2] * qd3 + end_vol[2])) / (jb2[0] * nj2[0] + jb2[1] * nj2[1] + jb2[2] * nj2[2]);

    qd1 = (jb1[0] * ((-1) * jb2[0] * qd2 + (-1) * jb3[0] * qd3 + end_vol[0]) + jb1[1] * ((-1) * jb2[1] * qd2 + (-1) * jb3[1] * qd3 + end_vol[1]) + jb1[2] * ((-1) * jb2[2] * qd2 + (-1) * jb3[2] * qd3 + end_vol[2])) / (jb1[0] * jb1[0] + jb1[1] * jb1[1] + jb1[2] * jb1[2]);

    omega01[0] = 0;
    omega01[1] = 0;
    omega01[2] = qd1;

    omega02[0] = qd2 * sin(q[0]);
    omega02[1] = (-1) * qd2 * cos(q[0]);
    omega02[2] = qd1 * cos(q[1]) * cos(q[1]) + qd1 * sin(q[1]) * sin(q[1]);

    v02[0] = (-1) * l2 * qd1 * cos(q[1]) * sin(q[0]) + (-1) * l2 * qd2 * cos(q[0]) * sin(q[1]);

    v02[1] = l2 * qd1 * cos(q[0]) * cos(q[1]) + (-1) * l2 * qd2 * sin(q[0]) * sin(q[1]);

    v02[2] = l2 * qd2 * cos(q[1]);

    v33[0] = l2 * qd2 * sin(q[2]);
    v33[1] = l3 * qd2 + l3 * qd3 + l2 * qd2 * cos(q[2]);
    v33[2] = (-1) * l2 * qd1 * cos(q[1]) + (-1) * l3 * qd1 * cos(q[1]) * cos(q[2]) + l3 * qd1 * sin(q[1]) * sin(q[2]);

    v03[0] = v33[2] * sin(q[0]) + v33[1] * ((-1) * cos(q[0]) * cos(q[2]) * sin(q[1]) + (-1) * cos(q[0]) * cos(q[1]) * sin(q[2])) + v33[0] * (cos(q[0]) * cos(q[1]) * cos(q[2]) + (-1) * cos(q[0]) * sin(q[1]) * sin(q[2]));
    v03[1] = (-1) * v33[2] * cos(q[0]) + v33[1] * ((-1) * cos(q[2]) * sin(q[0]) * sin(q[1]) + (-1) * cos(q[1]) * sin(q[0]) * sin(q[2])) + v33[0] * (cos(q[1]) * cos(q[2]) * sin(q[0]) + (-1) * sin(q[0]) * sin(q[1]) * sin(q[2]));
    v03[2] = v33[0] * (cos(q[2]) * sin(q[1]) + cos(q[1]) * sin(q[2])) + v33[1] * (cos(q[1]) * cos(q[2]) + (-1) * sin(q[1]) * sin(q[2]));

    db1[0] = (-1) * v03[1];
    db1[1] = v03[0];
    db1[2] = 0;

    db2[0] = (-1) * v03[2] * cos(q[0]) + l2 * qd1 * sin(q[0]) * sin(q[1]) + l3 * qd1 * cos(q[2]) * sin(q[0]) * sin(q[1]) + l3 * qd1 * cos(q[1]) * sin(q[0]) * sin(q[2]);
    db2[1] = (-1) * v03[2] * sin(q[0]) + (-1) * l2 * qd1 * cos(q[0]) * sin(q[1]) + (-1) * l3 * qd1 * cos(q[0]) * cos(q[2]) * sin(q[1]) + (-1) * l3 * qd1 * cos(q[0]) * cos(q[1]) * sin(q[2]);
    db2[2] = v03[0] * cos(q[0]) + v03[1] * sin(q[0]);

    db3[0] = v02[2] * cos(q[0]) + (-1) * v03[2] * cos(q[0]) + l3 * omega02[0] * cos(q[0]) * cos(q[1]) * cos(q[2]) * sin(q[0]) + l3 * omega02[1] * cos(q[1]) * cos(q[2]) * sin(q[0]) * sin(q[0]) + l3 * omega02[2] * cos(q[2]) * sin(q[0]) * sin(q[1]) + l3 * omega02[2] * cos(q[1]) * sin(q[0]) * sin(q[2]) + (-1) * l3 * omega02[0] * cos(q[0]) * sin(q[0]) * sin(q[1]) * sin(q[2]) + (-1) * l3 * omega02[1] * sin(q[0]) * sin(q[0]) * sin(q[1]) * sin(q[2]);
    db3[1] = (-1) * l3 * omega02[0] * cos(q[0]) * cos(q[0]) * cos(q[1]) * cos(q[2]) + v02[2] * sin(q[0]) + (-1) * v03[2] * sin(q[0]) + (-1) * l3 * omega02[1] * cos(q[0]) * cos(q[1]) * cos(q[2]) * sin(q[0]) + (-1) * l3 * omega02[2] * cos(q[0]) * cos(q[2]) * sin(q[1]) + (-1) * l3 * omega02[2] * cos(q[0]) * cos(q[1]) * sin(q[2]) + l3 * omega02[0] * cos(q[0]) * cos(q[0]) * sin(q[1]) * sin(q[2]) + l3 * omega02[1] * cos(q[0]) * sin(q[0]) * sin(q[1]) * sin(q[2]);

    db3[2] = (-1) * v02[0] * cos(q[0]) + v03[0] * cos(q[0]) + (-1) * v02[1] * sin(q[0]) + v03[1] * sin(q[0]);

    ddxs[0] = A_wan[0] + qd1 * v03[1] + (-1) * qd2 * ((-1) * v03[2] * cos(q[0]) + l2 * qd1 * sin(q[0]) * sin(q[1]) + l3 * qd1 * cos(q[2]) * sin(q[0]) * sin(q[1]) + l3 * qd1 * cos(q[1]) * sin(q[0]) * sin(q[2])) + (-1) * qd3 * (v02[2] * cos(q[0]) + (-1) * v03[2] * cos(q[0]) + l3 * omega02[0] * cos(q[0]) * cos(q[1]) * cos(q[2]) * sin(q[0]) + l3 * omega02[1] * cos(q[1]) * cos(q[2]) * sin(q[0]) * sin(q[0]) + l3 * omega02[2] * cos(q[2]) * sin(q[0]) * sin(q[1]) + l3 * omega02[2] * cos(q[1]) * sin(q[0]) * sin(q[2]) + (-1) * l3 * omega02[0] * cos(q[0]) * sin(q[0]) * sin(q[1]) * sin(q[2]) + (-1) * l3 * omega02[1] * sin(q[0]) * sin(q[0]) * sin(q[1]) * sin(q[2]));
    ddxs[1] = A_wan[1] + (-1) * qd1 * v03[0] + (-1) * qd2 * ((-1) * v03[2] * sin(q[0]) + (-1) * l2 * qd1 * cos(q[0]) * sin(q[1]) + (-1) * l3 * qd1 * cos(q[0]) * cos(q[2]) * sin(q[1]) + (-1) * l3 * qd1 * cos(q[0]) * cos(q[1]) * sin(q[2])) + (-1) * qd3 * ((-1) * l3 * omega02[0] * cos(q[0]) * cos(q[0]) * cos(q[1]) * cos(q[2]) + v02[2] * sin(q[0]) + (-1) * v03[2] * sin(q[0]) + (-1) * l3 * omega02[1] * cos(q[0]) * cos(q[1]) * cos(q[2]) * sin(q[0]) + (-1) * l3 * omega02[2] * cos(q[0]) * cos(q[2]) * sin(q[1]) + (-1) * l3 * omega02[2] * cos(q[0]) * cos(q[1]) * sin(q[2]) + l3 * omega02[0] * cos(q[0]) * cos(q[0]) * sin(q[1]) * sin(q[2]) + l3 * omega02[1] * cos(q[0]) * sin(q[0]) * sin(q[1]) * sin(q[2]));
    ddxs[2] = A_wan[2] + (-1) * qd2 * (v03[0] * cos(q[0]) + v03[1] * sin(q[0])) + (-1) * qd3 * ((-1) * v02[0] * cos(q[0]) + v03[0] * cos(q[0]) + (-1) * v02[1] * sin(q[0]) + v03[1] * sin(q[0]));

    qdd3 = (ddxs[0] * nj3[0] + ddxs[1] * nj3[1] + ddxs[2] * nj3[2]) / (jb3[0] * nj3[0] + jb3[1] * nj3[1] + jb3[2] * nj3[2]);

    qdd2 = (nj2[0] * (ddxs[0] + (-1) * jb3[0] * qdd3) + nj2[1] * (ddxs[1] + (-1) * jb3[1] * qdd3) + nj2[2] * (ddxs[2] + (-1) * jb3[2] * qdd3)) / (jb2[0] * nj2[0] + jb2[1] * nj2[1] + jb2[2] * nj2[2]);

    qdd1 = (jb1[0] * (ddxs[0] + (-1) * jb2[0] * qdd2 + (-1) * jb3[0] * qdd3) + jb1[1] * (ddxs[1] + (-1) * jb2[1] * qdd2 + (-1) * jb3[1] * qdd3) + jb1[2] * (ddxs[2] + (-1) * jb2[2] * qdd2 + (-1) * jb3[2] * qdd3)) / (jb1[0] * jb1[0] + jb1[1] * jb1[1] + jb1[2] * jb1[2]);

    AV[0] = qd1;
    AV[1] = qd2;
    AV[2] = qd3;
    AV[3] = qdd1;
    AV[4] = qdd2;
    AV[5] = qdd3;
}

bool Robot::updatRobotJoint()
{
    auto parsedData = m_data_pool_ptr->GetUsbParsedData();

    if(parsedData.empty()){
        return false;
    } 

    std::vector<float> motorState;
    m_motor_ptr->usbDataToMotorState(parsedData, motorState); 
    m_motor_ptr->updatMotorState(motorState);

    m_current_joint_pos[0] = m_motor_direction[0] * m_motor_ptr->m_current_motor_pos[0];
    m_current_joint_pos[1] = m_motor_direction[1] * m_motor_ptr->m_current_motor_pos[1];
    m_current_joint_pos[2] = m_motor_direction[2] * m_motor_ptr->m_current_motor_pos[2];


    // std::cout << m_current_joint_pos[0] * 57.3 << ", "
    // << m_current_joint_pos[1] * 57.3  << ", "
    // << m_current_joint_pos[2] * 57.3  << std::endl;

    m_record_pos_0.push_back(m_current_joint_pos[0]);
    m_record_pos_1.push_back(m_current_joint_pos[1]);
    m_record_pos_2.push_back(m_current_joint_pos[2]);

    m_current_joint_vel[0] = m_motor_direction[0] * m_motor_ptr->m_current_motor_vel[0];
    m_current_joint_vel[1] = m_motor_direction[1] * m_motor_ptr->m_current_motor_vel[1];
    m_current_joint_vel[2] = m_motor_direction[2] * m_motor_ptr->m_current_motor_vel[2];

    m_record_vel_0.push_back(m_current_joint_vel[0]);
    m_record_vel_1.push_back(m_current_joint_vel[1]);
    m_record_vel_2.push_back(m_current_joint_vel[2]);

    m_current_joint_tau[0] = m_motor_direction[0] * m_motor_ptr->m_current_motor_tau[0];
    m_current_joint_tau[1] = m_motor_direction[1] * m_motor_ptr->m_current_motor_tau[1];
    m_current_joint_tau[2] = m_motor_direction[2] * m_motor_ptr->m_current_motor_tau[2];
    
    m_record_tau_0.push_back(m_current_joint_tau[0]);
    m_record_tau_1.push_back(m_current_joint_tau[1]);
    m_record_tau_2.push_back(m_current_joint_tau[2]);

    if(m_record_pos_0.size() >= 1000){
        for(int i = 0; i < 1000; ++i){
            m_record_data_file  << m_record_pos_0[i]
                                << "," << m_record_pos_1[i]
                                << "," << m_record_pos_2[i] 

                                << "," << m_record_vel_0[i] 
                                << "," << m_record_vel_1[i] 
                                << "," << m_record_vel_2[i] 

                                << "," << m_record_tau_0[i] 
                                << "," << m_record_tau_1[i] 
                                << "," << m_record_tau_2[i] 
                                << std::endl;
        }
        m_record_pos_0.clear();
        m_record_pos_1.clear();
        m_record_pos_2.clear();

        m_record_vel_0.clear();
        m_record_vel_1.clear();
        m_record_vel_2.clear();

        m_record_tau_0.clear();
        m_record_tau_1.clear();
        m_record_tau_2.clear();
    }

    m_motor_ptr->real_time_motor_protection(m_current_joint_vel, m_current_joint_tau);

    return true;
}


} // namespace RSR