#pragma once


#include <memory>
#include <vector>

#include <rtt/base/RunnableInterface.hpp>

#include "../robot/robot.hpp"




namespace RSR
{
class RobotRunnable : public RTT::base::RunnableInterface{
public:
    explicit RobotRunnable(Robot::Ptr Ptr);

    ~RobotRunnable();

    bool initialize() override;

    void step() override;

    void finalize() override;

    void loop() override;

    float get_duration_realtime(float time_step, float max_time);

public:


private:

    Robot::Ptr                  m_robot_ptr;    
    id_t                        m_time_cnt;

};
} // namespace Tiromu
