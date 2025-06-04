#pragma once

#include <vector>
#include <cstdint>
#include <memory>


#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>

namespace RSR
{
typedef RTT::os::Mutex OsMutex;

class DataPool
{
public:
    typedef std::shared_ptr<DataPool> Ptr;

    DataPool(){}

    ~DataPool(){}

    inline void SetUsbParsedData(const std::vector<uint8_t>& data)
    {
        RTT::os::MutexLock lock(m_usb_data_mutex);
        m_usb_data = data;
    }

    inline std::vector<uint8_t> GetUsbParsedData()
    {
        RTT::os::MutexLock lock(m_usb_data_mutex);
        return m_usb_data;
    }

private:
    std::vector<uint8_t>    m_usb_data;
    OsMutex                 m_usb_data_mutex;

};
    
} // namespace RSR
