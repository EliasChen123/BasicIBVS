
#include "SerialRunnable.hpp"


namespace RSR
{
    
    SerialRunnable::SerialRunnable(SerialPortInterface::Ptr Ptr, DataPool::Ptr dataPtr)
        :m_serial_interface_ptr(Ptr),
        m_data_pool_ptr(dataPtr)
    {

    }

    SerialRunnable::~SerialRunnable(){

    }

    bool SerialRunnable::initialize(){
        
        return true;
    }

    void SerialRunnable::step(){
        std::vector<uint8_t> data = m_serial_interface_ptr->receiveData();
        std::vector<uint8_t> parse_data;
        m_serial_interface_ptr->parseFrame(data, parse_data);


        
        m_data_pool_ptr->SetUsbParsedData(parse_data);

        return ;
    }

    void SerialRunnable::finalize(){

    }

    void SerialRunnable::loop(){

    }

} // namespace Tiromu
