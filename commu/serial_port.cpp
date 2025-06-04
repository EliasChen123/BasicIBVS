#include "serial_port.hpp"

namespace RSR
{

    SerialPort::SerialPort(const std::string& port_name, LibSerial::BaudRate baud_rate)
        :m_serial_port(std::make_shared<LibSerial::SerialPort>())
    {
        try
        {
            m_serial_port->Open(port_name);
            m_serial_port->SetBaudRate(baud_rate);
            m_serial_port->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            m_serial_port->SetParity(Parity::PARITY_NONE);
            m_serial_port->SetStopBits(StopBits::STOP_BITS_1);
            m_serial_port->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

        }
        catch (const OpenFailed &)
        {
            std::cerr << "无法打开串口：" << port_name << std::endl;
            throw;
        }
    }

    SerialPort::~SerialPort()
    {
        if (m_serial_port->IsOpen())
        {
            m_serial_port->Close();
        }
    }

    void SerialPort::send(const DataBuffer &buffer)
    {
        try
        {
            m_serial_port->Write(buffer);

            // std::cout << "已发送数据：" << buffer.size() << " 字节" << std::endl;
            // for (const auto &byte : buffer)
            // {
            //     std::cout << std::hex << static_cast<int>(byte) << " ";
            // }
        }
        catch (const std::exception &e)
        {
            std::cerr << "发生错误: " << e.what() << std::endl;
        }
    }

    void SerialPort::read(DataBuffer& byte, size_t numberOfBytes){
        try
        {
            m_serial_port->Read(byte, 1);
        }
        catch (const std::exception &e)
        {
            std::cerr << "发生错误: " << e.what() << std::endl;
        }
    }

    void SerialPort::close(){
        if (m_serial_port->IsOpen())
        {
            // m_serial_port->Close();
        }
    }

} // namespace libserial_example
