#ifndef __CALIBUR_USB_COMMUNICATION_H__
#define __CALIBUR_USB_COMMUNICATION_H__

#include <string>
#include <cstdint>
#include <cstddef>
#include "protocol_data.hpp"

namespace calibur {

    class USBCommunication {
        public:
        USBCommunication(const std::string& device_path);
        ~USBCommunication();
        
        bool open();
        bool close();
        bool isOpen() const;

        std::uint16_t crc16_(const std::uint8_t* data, std::size_t size);
        
        bool sendData(Protocol::Type type, const void* data, std::size_t size); 
        // Configuration
        bool configure(int baudrate = 115200);


        private:
            std::string device_path_;
            int fd_;
            bool is_open_;
    };  
}

#endif