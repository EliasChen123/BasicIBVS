#include "light_lift_arm_3dof.h"
#include <cmath>
#include <iomanip>

LlArm3dof::LlArm3dof() : currentX(0), currentY(0), currentZ(0), currentSpeed(0.0) {

    motorState.reserve(ARM_DOF*3);
}

LlArm3dof::~LlArm3dof() {}

void LlArm3dof::moveToPosition(int x, int y, int z)
{
    // Add logic to move arm to specified position
    currentX = x;
    currentY = y;
    currentZ = z;
}

void LlArm3dof::setSpeed(float speed)
{
    // Set the movement speed of the arm
    currentSpeed = speed;
}

void LlArm3dof::stop()
{
    // Stop the arm's movement
    // Reset position or maintain current position
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
void LlArm3dof::Dof_3_Arm_iAV_Sca(float Joint[3], float V[3], float dV[3], float AV[6])
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
    // float[] q = new float[3];

    //   l2 = 0.184;
    //   l3 = 0.186;

    l2 = link_lenth[0];
    l3 = link_lenth[1];

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

/***********************/
/**
 * 输入：
 *  关节位置
 * 
*/
void LlArm3dof::get_jacb(float q[3], float jacobian[3][3])
{
    float J11, J12, J13, J21, J22, J23, J31, J32, J33, a1, a2, a3;
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    a1 = 0;
    //   a2 = 0.184;
    //   a3 = 0.186;

    a2 = link_lenth[0];
    a3 = link_lenth[1];

    J11 = -sin(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    J21 = cos(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    J31 = 0;

    J12 = -cos(q1) * (a2 * sin(q2) + a3 * sin(q2 + q3));
    J22 = -sin(q1) * (a2 * sin(q2) + a3 * sin(q2 + q3));
    J32 = a2 * cos(q2) + a3 * cos(q2 + q3);

    J13 = -a3 * cos(q1) * sin(q3 + q2);
    J23 = -a3 * sin(q1) * sin(q3 + q2);
    J33 = a3 * cos(q3 + q2);

    jacobian[0][0] = J11;
    jacobian[0][1] = J12;
    jacobian[0][2] = J13;

    jacobian[1][0] = J21;
    jacobian[1][1] = J22;
    jacobian[1][2] = J23;

    jacobian[2][0] = J31;
    jacobian[2][1] = J32;
    jacobian[2][2] = J33;
}
/*************************************************/

/**
 * 输入：
 *  关节角度
 *  末端力矩
 * 输出：
 *  关节力矩
 * */ 
void LlArm3dof::end2joint_force(float q[3], float end_force[3], float joint_force[3])
{
    float J11, J12, J13, J21, J22, J23, J31, J32, J33, a1, a2, a3;
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    a1 = 0;
    //   a2 = 0.184;
    //   a3 = 0.186;

    a2 = link_lenth[0];
    a3 = link_lenth[1];

    J11 = -sin(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    J21 = cos(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    J31 = 0;

    J12 = -cos(q1) * (a2 * sin(q2) + a3 * sin(q2 + q3));
    J22 = -sin(q1) * (a2 * sin(q2) + a3 * sin(q2 + q3));
    J32 = a2 * cos(q2) + a3 * cos(q2 + q3);

    J13 = -a3 * cos(q1) * sin(q3 + q2);
    J23 = -a3 * sin(q1) * sin(q3 + q2);
    J33 = a3 * cos(q3 + q2);

    // ��ùؽڿռ���迹��
    joint_force[0] = J11 * end_force[0] + J21 * end_force[1] + J31 * end_force[2];
    joint_force[1] = J12 * end_force[0] + J22 * end_force[1] + J32 * end_force[2];
    joint_force[2] = J13 * end_force[0] + J23 * end_force[1] + J33 * end_force[2];

    // joint_force[2] = joint_force[2];
}

/*************************************************/

/**
 * 输入：关节力矩
 * 输出：末端力矩
 * */ 
void LlArm3dof::joint2end_force(float q[3], float joint_force[3], float end_force[3])
{
    float J11, J12, J13, J21, J22, J23, J31, J32, J33, a1, a2, a3;
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    a1 = 0;
    //   a2 = 0.184;
    //   a3 = 0.186;

    a2 = link_lenth[0];
    a3 = link_lenth[1];

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
/*************************************************/

/**
 * 输入：关节角度
 * 
 * 输出：末端位置
 * */ 
void LlArm3dof::fk(float q[3], float X[3])
{
    float a1 = 0, a2 = 0.184, a3 = 0.186;
    a2 = link_lenth[0];
    a3 = link_lenth[1];

    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];

    X[0] = cos(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    X[1] = sin(q1) * (a2 * cos(q2) + a3 * cos(q2 + q3));
    X[2] = a2 * sin(q2) + a3 * sin(q2 + q3);
}
/*************************************************/


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
void LlArm3dof::end_impedance_control(float desir_end_pos[3], float desir_end_vel[3], 
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

    a2 = link_lenth[0];
    a3 = link_lenth[1];

    // �����ռ䵯�Ե����
    X_desired[0] = desir_end_pos[0];
    X_desired[1] = desir_end_pos[1];
    X_desired[2] = desir_end_pos[2];
    // �ӵ���Ƕ�ת���ɹؽڽǶ�
    float q1 = current_joint_pos[0];
    float q2 = current_joint_pos[1];
    float q3 = current_joint_pos[2];
    // current_joint_vel[2] = current_joint_vel[2];

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
/***********************************************************************/


// ������ϵ����
void LlArm3dof::spherical(float desir_point[3], float desir_theta, float desir_fa, float current_point[3], float out_point[3])
{
    float x, y, z;
    x = current_point[0] - desir_point[0];
    y = current_point[1] - desir_point[1];
    z = current_point[2] - desir_point[2];
    float r = sqrt(x * x + y * y + z * z);
    out_point[0] = desir_point[0] + r * sin(desir_theta) * cos(desir_fa);
    out_point[1] = desir_point[1] + r * sin(desir_theta) * sin(desir_fa);
    out_point[2] = desir_point[2] + r * cos(desir_theta);
}
/*********************************************************************************/

void LlArm3dof::spherical_force(float desir_theta, float desir_fa, float force_r, float out_point[3])
{
    out_point[0] = force_r * sin(desir_theta) * cos(desir_fa);
    out_point[1] = force_r * sin(desir_theta) * sin(desir_fa);
    out_point[2] = force_r * cos(desir_theta);
}
/*********************************************************************************/

int LlArm3dof::now2desir(float now_end_pos[3], float dlt_end_pos[3], float cur_pos[3], float cur_vel[3], float cur_acc[3], float next, int is_run)
{
    float dlt[3] = {0};
    static int times = 0;

    if (is_run == 1)
    {

        if (next == 1)
            times++;
        float time = times * 0.001;
        float rate = 1.0;
        if (times >= 1570)
        {
            times = 0;
            return 1; // ����1����ʾ��������λ��
        }
        else
        {
            cur_pos[0] = now_end_pos[0] + dlt_end_pos[0] * sin(rate * time);
            cur_pos[1] = now_end_pos[1] + dlt_end_pos[1] * sin(rate * time);
            cur_pos[2] = now_end_pos[2] + dlt_end_pos[2] * sin(rate * time);

            cur_vel[0] = rate * dlt_end_pos[0] * cos(rate * time);
            cur_vel[1] = rate * dlt_end_pos[1] * cos(rate * time);
            cur_vel[2] = rate * dlt_end_pos[2] * cos(rate * time);

            cur_acc[0] = -rate * rate * dlt_end_pos[0] * sin(rate * time);
            cur_acc[1] = -rate * rate * dlt_end_pos[1] * sin(rate * time);
            cur_acc[2] = -rate * rate * dlt_end_pos[2] * sin(rate * time);

            return 0; // ����0����ʾ��û�е���
        }
    }
    else
    {
        return 1; // ����1����ʾ��������λ��
    }
}
//***************************************************************************

/**
 * 输入：
 *      末端位置
 * 
 * 输出：
 *      关节位置
*/
void LlArm3dof::Ik_3Dof(float end_effect_pos[3], float q[3])
{
    float FuY, r, a, b, c, elbow, m1, m2, m3, a1, a2, a3, detlta, q21, q22;
    float q1, q2, q3;
    float x = end_effect_pos[0];
    float y = end_effect_pos[1];
    float z = end_effect_pos[2];

    a2 = link_lenth[0];
    a3 = link_lenth[1];

    /******************************************************/
    // 把加爪的位置转换为机械臂末端的位置
    q1 = atan2(y, x);
    x = end_effect_pos[0] - end2grp * cos(q1);
    y = end_effect_pos[1] - end2grp * sin(q1);
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
//*******************************************************************

/**
 * 输入: 关节的角度
 * 输出：关节力矩
 * 
*/
void LlArm3dof::Closed_Arm_Modle_decoup(float q[3], float dq[3], float ddq[3], float tao_[3])
{
    float Joint_Torque[9];
    float tao3, tao2, tao1, n11x, n11y, n11z, n22y, n22x, n33x, n33y, n33z;
    float q1, qd1, qdd1, q2, qd2, qdd2, q3, qd3, qdd3; // �ؽ�״̬
    float a1, a2, a3, m1, m2, m3;
    float Ixx1, Iyy1, Izz1, Ixx2, Izz2, Iyy2, Izz3, Ixx3, Iyy3;
    float M1_Q1, C1_Q1Q3, C1_Q1Q2, G2, C2_Q1Q1, C2_Q2Q2, C2_Q2Q3, C2_Q3Q3, M2_Q3, M2_Q2, G3, C3_Q1Q1, C3_Q2Q2, M3_Q3, M3_Q2;
    q1 = q[0];
    qd1 = dq[0];
    qdd1 = ddq[0];
    q2 = q[1];
    qd2 = dq[1];
    qdd2 = ddq[1];
    q3 = q[2];
    qd3 = dq[2];
    qdd3 = ddq[2];

    a1 = 0.0;
    //   a2 = 0.184;
    //   a3 = 0.186;
    a2 = link_lenth[0];
    a3 = link_lenth[1];

    m1 = 0.0;
    m2 = 0.1;
    m3 = 0.055;

    m2 = link_mass[0];
    m3 = link_mass[1];

 
    // Izz1 = 0.0015;
    // Izz2 = 0.00437;
    // Izz3 = 0.00108;
    // Iyy1 = 0.0015;
    // Iyy2 = 0.00437;
    // Iyy3 = 0.00108;
    // Ixx1 = 0.0015;
    // Ixx2 = 0.00;
    // Ixx3 = 0.00;


    // Izz1 = 0.0035;
    // Izz2 = 0.0067;
    // Izz3 = 0.0048;
    // Iyy1 = 0.0025;
    // Iyy2 = 0.006;
    // Iyy3 = 0.0048;
    // Ixx1 = 0.0025;
    // Ixx2 = 0.0001;
    // Ixx3 = 0.00005;

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

    tao_[0] = tao1;
    tao_[1] = tao2;
    tao_[2] = tao3;
}
//*******************************************************************

/**
 * �� �� �� �� ѧ ģ ��
 *
 * ���룺�ؽ�״̬ => [q1,q2,q3 , qd1,qd2,qd3 , qdd1,qdd2,qdd3]
 * ������ؽ����� => [tao1 , tao2 , tao3]
 * �ڲ��Ѿ��Թؽ�3�ķ������˵���
 * **/
void LlArm3dof::Closed_Arm_Modle_teach(float q[3], float dq[3], float ddq[3], float tao_[3])
{

    float Joint_Torque[9];
    float tao3, tao2, tao1, n11x, n11y, n11z, n22y, n22x, n33x, n33y, n33z;
    float q1, qd1, qdd1, q2, qd2, qdd2, q3, qd3, qdd3; // �ؽ�״̬
    float a1, a2, a3, m1, m2, m3;
    float Ixx1, Iyy1, Izz1, Ixx2, Izz2, Iyy2, Izz3, Ixx3, Iyy3;
    float M1_Q1, C1_Q1Q3, C1_Q1Q2, G2, C2_Q1Q1, C2_Q2Q2, C2_Q2Q3, C2_Q3Q3, M2_Q3, M2_Q2, G3, C3_Q1Q1, C3_Q2Q2, M3_Q3, M3_Q2;
    q1 = q[0];
    qd1 = dq[0];
    qdd1 = ddq[0];
    q2 = q[1];
    qd2 = dq[1];
    qdd2 = ddq[1];
    q3 = -q[2];
    qd3 = -dq[2];
    qdd3 = -ddq[2];

    a1 = 0.0;
    //   a2 = 0.184;
    //   a3 = 0.186;

    a2 = link_lenth[0];
    a3 = link_lenth[1];

    m1 = 0.0;
    //   m2 = 0.01;
    //   m3 = 0.01;
    m2 = link_mass[0];
    m3 = link_mass[1];

    Izz1 = 0.0015;
    Izz2 = 0.00437;
    Izz3 = 0.00108;
    Iyy1 = 0.0015;
    Iyy2 = 0.00437;
    Iyy3 = 0.00108;
    Ixx1 = 0.0015;
    Ixx2 = 0.00;
    Ixx3 = 0.00;

    G3 = a3 * m3 * cos(q2 + q3) * 9.81;
    tao3 = G3;

    G2 = tao3 + a2 * cos(q2) * (9.81 * m2 + 9.81 * m3);
    tao2 = G2;
    tao1 = 0;

    tao_[0] = tao1;
    tao_[1] = tao2;
    tao_[2] = -tao3;
}

//*******************************************************
/**
 * 输入：
 *      末端位置
 *      末端速度
 *      末端加速度
 *      反馈的关节位置
 *      反馈的关节速度
 * 输出：
 *      关节位置
 *      关节速度
 *      关节力矩
*/
void LlArm3dof::PID_impdence(float xyz[3], float end_vel[3], float end_acc[3], float motor_pos_bk[3], float motor_vel_bk[3], float c, float k, float tao_imp[3], float motor_pos[3], float motor_vel[3])
{
    float joint_state[6] = {0, 0, 0, 0, 0, 0}, imp_desir_pos[3] = {0}, imp_desir_vel[3] = {0};
    float motor_acc[3] = {0}, joint_pos[3] = {0};
    float x = xyz[0];
    float y = xyz[1];
    float z = xyz[2];
    //计算关节位置
    Ik_3Dof(xyz, motor_pos);
    //将电机位置转换为关节位置
    joint_pos[0] = motor_pos[0];
    joint_pos[1] = motor_pos[1];
    joint_pos[2] = motor_pos[2];
    //计算关节加速度，对应期望末端速度和加速度
    Dof_3_Arm_iAV_Sca(joint_pos, end_vel, end_acc, joint_state);
    // 将关节速度和加速度转化为电机速度和加速度
    motor_vel[0] = joint_state[0];
    motor_vel[1] = joint_state[1];
    motor_vel[2] = joint_state[2];
    motor_acc[0] = joint_state[3];
    motor_acc[1] = joint_state[4];
    motor_acc[2] = joint_state[5];
    //期望的末端位置？？？？？？
    fk(motor_pos, imp_desir_pos); 
    //期望的末端速度？？？
    joint2end_force(motor_pos, motor_vel, imp_desir_vel); // �����������ĩ���ٶ�

    // �ڲ��Ὣ����Ƕ�ת���ɹؽڽǶ�, Ҳ�Ὣ�ؽ�����ת���ɵ������
    float t;
    // end_impedance_control(imp_desir_pos, imp_desir_vel, motor_pos_bk, motor_vel_bk, tao_imp, c, k, &t);
}

//****************************************
   
   
    /**
     * 输入：
     *      参考末端位置
     *      当前关节位置
     *      当前关节速度
     *      期望末端加速度（当前的末端加速度难获取）
     *      期望的末端受力方向
     *      期望的末端力
     * 输出：
     *      关节位置
     *      关节速度
     *      关节力矩
     *  
     * 
    */   
void LlArm3dof::admittance(float reference_point[3], float motor_pos_bk[3], float motor_vel_bk[3], float end_force_orien[3], float end_force_value, float end_acc[3], float motor_pos[3], float motor_vel[3], float tao_dan[3])
{
      /*  输入
        motor_pos_bk
        end_force_orien
        motor_vel_bk
        end_acc
        end_force_value

        输出
        motor_pos
        motor_vel
        tao_dan
        */

    float current_end_pos[3] = {0.0}, current_end_vel[3] = {0.0}, imp_desir_pos[3] = {0.0}, imp_desir_vel[3] = {0.0};
    float end_force[3] = {0}, joint_pos[3] = {0}, joint_state[6] = {0};
    float refer_mask[3] = {0.0};
    fk(motor_pos_bk, current_end_pos); // 获取当前末端位置

    // spherical(reference_point, theta, fa, current_end_pos, imp_desir_pos);//
    //在期望位置的基础下，允许某个轴的期望位置实时更新。

    for (size_t i = 0; i < 3; i++)
    {
        refer_mask[i] = 1 - abs(end_force_orien[i]);
    }
    // std::cout << "desir_motor_pos  : " << current_end_pos[0] << " " << current_end_pos[1] << " " << current_end_pos[2]<< std::endl;
    imp_desir_pos[0] = reference_point[0] * refer_mask[0] + current_end_pos[0] * abs(end_force_orien[0]); 
    imp_desir_pos[1] = reference_point[1] * refer_mask[1] + current_end_pos[1] * abs(end_force_orien[1]); 
    imp_desir_pos[2] = reference_point[2] * refer_mask[2] + current_end_pos[2] * abs(end_force_orien[2]); 

    // std::cout << "imp_desir_pos    : " << imp_desir_pos[0] << " " << imp_desir_pos[1] << " " << imp_desir_pos[2]<< std::endl;

    joint2end_force(motor_pos_bk, motor_vel_bk, imp_desir_vel); // / 获取当前末端位速度

    end_force[0] = end_force_orien[0] * end_force_value;
    end_force[1] = end_force_orien[1] * end_force_value;
    end_force[2] = end_force_orien[2] * end_force_value;
    end2joint_force(motor_pos_bk, end_force, tao_dan); // 期望末端力，对应的关节的力矩

    //						x = imp_desir_pos[0];
    //						y = imp_desir_pos[1];
    //						z = imp_desir_pos[2];

    float dx = imp_desir_vel[0] * end_force_orien[0];
    float dy = imp_desir_vel[1] * end_force_orien[1];
    float dz = imp_desir_vel[2] * end_force_orien[2];

    float ddx = end_acc[0];
    float ddy = end_acc[1];
    float ddz = end_acc[2];

    float end_vel_[3] = {dx, dy, dz};
    float end_acc_[3] = {ddx, ddy, ddz};
    imp_desir_vel[0] = end_vel_[0];
    imp_desir_vel[1] = end_vel_[1];
    imp_desir_vel[2] = end_vel_[2];
    Ik_3Dof(imp_desir_pos, motor_pos);
    joint_pos[0] = motor_pos[0];
    joint_pos[1] = motor_pos[1];
    joint_pos[2] = motor_pos[2];
    //加速度用的期望值，其实给0也可
    //速度用的当前值（受力方向影响）
    //位置用的期望值
    Dof_3_Arm_iAV_Sca(joint_pos, end_vel_, end_acc_, joint_state); 
    motor_vel[0] = joint_state[0];
    motor_vel[1] = joint_state[1];
    motor_vel[2] = joint_state[2];
    // std::cout << "motor_vel    : " << motor_vel[0] << " " << motor_vel[1] << " " << motor_vel[2]<< std::endl;

}

void LlArm3dof::updatArmState(float motor_pos[3], float motor_vel[3], float motor_tau[3])
{
    //不更新关节加速度
    current_joint_pos[0] = motor_direction[0]*motor_pos[0];
    current_joint_pos[1] = motor_direction[1]*motor_pos[1];
    current_joint_pos[2] = motor_direction[2]*motor_pos[2];

    current_joint_vel[0] = motor_direction[0]*motor_vel[0];
    current_joint_vel[1] = motor_direction[1]*motor_vel[1];
    current_joint_vel[2] = motor_direction[2]*motor_vel[2];

    current_joint_tau[0] = motor_direction[0]*motor_tau[0];
    current_joint_tau[1] = motor_direction[1]*motor_tau[1];
    current_joint_tau[2] = motor_direction[2]*motor_tau[2];

    // std::cout << "manualServo_STSMC-motor_control_tau: " << arm.desir_joint_vel[0] << " " << arm.desir_joint_vel[1] << " " << arm.desir_joint_vel[2] << std::endl;


}

void LlArm3dof::joint2motor(float q[ARM_DOF], float dq[ARM_DOF], float ddq[ARM_DOF], float tau_joint[ARM_DOF], float motor_pos[ARM_DOF], float motor_vel[ARM_DOF], float motor_acc[ARM_DOF], float tau_motor[ARM_DOF])
{
    for (size_t i = 0; i < 3; i++)
    {
        motor_pos[i] = motor_direction[i] * q[i];
        motor_vel[i] = motor_direction[i] * dq[i];
        motor_acc[i] = motor_direction[i] * ddq[i];
        tau_motor[i] = motor_direction[i] * tau_joint[i];
    }
}
//*****************************************************************

/**
 * 插值
 * 现在到期望的误差只算一次
 * 输入：
 *      关节位置
 * 
 * 输出：
 *      关节位置
 *
 */
bool  LlArm3dof::now2aim(float sta_pos[3], float end_pos[3], float now_pos[3], float steps, int delay)
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

    // std::cout << "------------" <<std::endl;

    // std::cout << "start_pos: " << start_pos[0] << " " << start_pos[1] << " " << start_pos[2]<<std::endl;

    // std::cout << "end_pos  : " << end_pos[0] << " " << end_pos[1] << " " << end_pos[2]<<std::endl;

    // std::cout << "now_pos  : " << now_pos[0] << " " << now_pos[1] << " " << now_pos[2]<<std::endl;

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