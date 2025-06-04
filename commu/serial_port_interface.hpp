#pragma once

#include "serial_port.hpp"

#include <memory>
#include <boost/noncopyable.hpp>



namespace RSR
{
class SerialPortInterface : public boost::noncopyable {
public:
    static constexpr uint8_t FRAME_HEAD = 0xAA;
    static constexpr uint8_t FRAME_TAIL = 0x55;
    static constexpr uint16_t FRAME_DATA_LENGTH = 59;
    static constexpr uint16_t FRAME_SIZE = 1 + 1 + FRAME_DATA_LENGTH + 2 + 1;
    static constexpr unsigned char DATA_SIZE = FRAME_SIZE;
    
public:
    typedef std::shared_ptr<SerialPortInterface> Ptr;

    explicit SerialPortInterface(const SerialPort::Ptr& ptr);

    ~SerialPortInterface();

    std::vector<uint8_t> receiveData();

    void packFrame(const std::vector<uint8_t> &data, std::vector<uint8_t> &frame);
    
    bool parseFrame(const std::vector<uint8_t> &frame, std::vector<uint8_t> &data);

    uint16_t calculateCRC16(const uint8_t *data, uint16_t length);

    void PackAndSend(const std::vector<uint8_t>& data);





private:
    SerialPort::Ptr         m_serial_ptr;


};
    
} // namespace RSR
