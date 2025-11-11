#include "calibur/usb_communication.h"
#include "calibur/log.h"
#include <unistd.h>

int main() {
    calibur::Logger::ptr logger = CALIBUR_LOG_ROOT();
    
    CALIBUR_LOG_INFO(logger) << "=== USB Sender Test ===";
    
    calibur::USBCommunication usb("/dev/ttyUSB0");
    
    if (!usb.open()) {
        CALIBUR_LOG_FATAL(logger) << "Failed to open USB device";
        return 1;
    }
    
    CALIBUR_LOG_INFO(logger) << "Sending angles continuously (Ctrl+C to stop)...";
    
    float yaw = 0.0f;
    float pitch = 0.0f;
    
    while (true) {
        yaw += 1.0f;
        pitch += 0.5f;
        
        if (yaw > 360.0f) yaw = 0.0f;
        if (pitch > 90.0f) pitch = -90.0f;
        
        if (usb.sendData(yaw, pitch)) {
            CALIBUR_LOG_INFO(logger) << "Sent: yaw=" << yaw << ", pitch=" << pitch;
        } else {
            CALIBUR_LOG_ERROR(logger) << "Send failed";
        }
        
        sleep(1);
    }
    
    usb.close();
    return 0;
}