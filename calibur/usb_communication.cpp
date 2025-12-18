#include "usb_communication.h"
#include "log.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <errno.h>

namespace calibur {

// static calibur::Logger::ptr g_logger = CALIBUR_LOG_NAME("usb");

    USBCommunication::USBCommunication(const std::string& device_path)
        : device_path_(device_path)
        , fd_(-1)
        , is_open_(false) {
        // CALIBUR_LOG_INFO(g_logger) << "USBCommunication created for device: " << device_path_;
    }

    USBCommunication::~USBCommunication() {
        close();
    }

    bool USBCommunication::open() {
        if (is_open_) {
            // CALIBUR_LOG_WARN(g_logger) << "Device already open: " << device_path_;
            return true;
        }
        
        // CALIBUR_LOG_INFO(g_logger) << "Opening USB device: " << device_path_;
        
        fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            // CALIBUR_LOG_ERROR(g_logger) << "Failed to open USB device: " << device_path_ << ", error: " << strerror(errno);
            std::cout << "Failed to open USB device: " << device_path_ << ", error: " << strerror(errno) << std::endl;
            return false;
        }
        
        is_open_ = true;
        // CALIBUR_LOG_INFO(g_logger) << "USB device opened successfully";
        return configure();
    }

    bool USBCommunication::close() {
        if (fd_ >= 0) {
            // CALIBUR_LOG_INFO(g_logger) << "Closing USB device: " << device_path_;
            ::close(fd_);
            fd_ = -1;
        }
        is_open_ = false;
        return true;
    }

    bool USBCommunication::isOpen() const {
        return is_open_;
    }

    bool USBCommunication::configure(int baudrate) {
        if (fd_ < 0) {
            // CALIBUR_LOG_ERROR(g_logger) << "Cannot configure: device not open";
            return false;
        }
        
        // CALIBUR_LOG_INFO(g_logger) << "Configuring USB device with baudrate: " << baudrate;
        
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            // CALIBUR_LOG_ERROR(g_logger) << "Error getting termios attributes: " << strerror(errno);
            return false;
        }
        
        // Set baud rate
        speed_t speed = B115200;
        switch(baudrate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default:
                // CALIBUR_LOG_WARN(g_logger) << "Unsupported baudrate " << baudrate  << ", using 115200";
                speed = B115200;
        }
        
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        
        // 8N1 mode
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= (CLOCAL | CREAD);
        
        // Raw mode
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        // Non-blocking writes
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            // CALIBUR_LOG_ERROR(g_logger) << "Error setting termios attributes: " << strerror(errno);
            return false;
        }
        
        // CALIBUR_LOG_INFO(g_logger) << "USB device configured successfully";
        return true;
    }

    bool USBCommunication::sendData(Protocol::Type type, const void* data, std::size_t size) {
        if (!is_open_ || fd_ < 0) {
            // CALIBUR_LOG_ERROR(g_logger) << "Cannot send: USB device not open";
            return false;
        }

        if(!data || size == 0 || size > Protocol::MAX_PAYLOAD_SIZE) {
            return false;
        }

        uint8_t packet[Protocol::MAX_PACKET_SIZE];
        const std::size_t payload_offset = Protocol::HEADER_SIZE;
        const std::size_t total_no_crc = payload_offset + size;
        const std::size_t total_with_crc = total_no_crc + Protocol::CRC_SIZE;
 
        packet[0] = Protocol::HEADER_BYTE;
        packet[1] = static_cast<std::uint8_t>(size & 0xFF);
        packet[2] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
        packet[3] = static_cast<std::uint8_t>(type);

        std::memcpy(&packet[payload_offset], data, size);

        std::uint16_t crc = crc16(packet, total_no_crc);
        packet[total_no_crc] = crc & 0xFF;
        packet[total_no_crc + 1] = (crc >> 8) & 0xFF;
        
        ssize_t written = write(fd_, packet, total_with_crc);
        if (written != static_cast<ssize_t>(total_with_crc)) {
            // CALIBUR_LOG_ERROR(g_logger) << "Write failed: expected 11 bytes, wrote "  << written << ", error: " << strerror(errno);
            return false;
        }
        
        // CALIBUR_LOG_DEBUG(g_logger) << "Sent data: yaw=" << yaw << ", pitch=" << pitch  << ", is_fire=" << (is_fire ? "true" : "false");
        return true;
    }

    constexpr std::uint16_t crc16(const std::uint8_t* data, std::size_t size) noexcept {
        std::uint16_t crc = 0xFFFF;

        for(std::size_t i = 0; i < size; ++i) {
            crc ^= static_cast<std::uint16_t>(data[i]) << 8;
            for (int j =0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }


} 