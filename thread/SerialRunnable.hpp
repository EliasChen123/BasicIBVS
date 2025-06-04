#pragma once


#include <memory>
#include <vector>

#include <rtt/base/RunnableInterface.hpp>

#include "../commu/serial_port_interface.hpp"
#include "../data_pool/data_pool.hpp"



namespace RSR
{
class SerialRunnable : public RTT::base::RunnableInterface{
public:
    explicit SerialRunnable(SerialPortInterface::Ptr Ptr, DataPool::Ptr dataPtr);

    ~SerialRunnable();

    bool initialize() override;

    void step() override;

    void finalize() override;

    void loop() override;

public:


private:

    SerialPortInterface::Ptr    m_serial_interface_ptr; 
    DataPool::Ptr               m_data_pool_ptr;    

};
} // namespace Tiromu
