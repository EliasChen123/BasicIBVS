#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <stdbool.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>

#include <memory>


using namespace LibSerial;

namespace RSR
{

    class SerialPort
    {

        
    public:
        typedef std::shared_ptr<SerialPort> Ptr;

        SerialPort(const std::string& port_name, LibSerial::BaudRate baud_rate);
        ~SerialPort();

        using DataBuffer = std::vector<uint8_t>;

        void send(const DataBuffer &buffer);

        void read(DataBuffer& byte, size_t numberOfBytes);

        void close();

        std::vector<uint8_t> receiveData();


    private:
        std::shared_ptr<LibSerial::SerialPort>   m_serial_port;

    };

} // namespace libserial_example

#endif // LIBSERIAL_EXAMPLE_SERIALPORT_H
