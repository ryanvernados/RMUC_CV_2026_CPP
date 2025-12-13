#include "types.hpp"
#include <thread>
#include "workers.hpp"
#include "usb_communication.h"
#include "calibur/log.h"

static calibur::Logger::ptr g_logger = CALIBUR_LOG_NAME("usb");

USBWorker::USBWorker(SharedLatest &shared,
            SharedScalars &scalars,
            std::atomic<bool> &stop_flag)
    : shared_(shared), scalars_(scalars), stop_(stop_flag), last_pred_ver_(0) {}

void USBWorker::operator()() {
    while (!stop_.load(std::memory_order_relaxed)) {
        process_usb_rx(); // updates scalars_.bullet_speed etc.

        uint64_t cur_ver = shared_.prediction_ver.load(std::memory_order_relaxed);
        if (cur_ver == last_pred_ver_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue; // no new prediction
        }
        last_pred_ver_ = cur_ver;

        auto pred = std::atomic_load(&shared_.prediction_out);
        if (pred) {
            usb_send_tx(*pred);
        }
    }
}

void USBWorker::process_usb_rx() {
    // parse incoming packets
    // e.g. update scalars_.bullet_speed.store(new_speed);
}

void USBWorker::usb_send_tx(const PredictionOut &out) {
    // encode yaw, pitch, aim, fire, chase to COM

    if (!is_open_ || fd_ < 0) {
        CALIBUR_LOG_ERROR(g_logger) << "Cannot send: USB device not open";
        return false;
    }

    // packet 1
    {
    uint16_t len = 8;
    uint8_t data_type = 0x01;
    std::vector<uint8_t> packet = {0xAA, static_cast<uint8_t>(len & 0xFF), static_cast<uint8_t>((len >> 8) & 0xFF), data_type};
    
    // yaw
    uint8_t yaw_len = 4;
    uint8_t yaw_bytes[4];
    memcpy(yaw_bytes, &out.yaw, yaw_len);
    packet.insert(packet.end(), yaw_bytes, yaw_bytes + 4);

    //pitch
    uint8_t pitch_len = 4;
    uint8_t pitch_bytes[4];
    memcpy(pitch_bytes, &out.pitch, pitch_len);
    packet.insert(packet.end(), pitch_bytes, pitch_bytes + 4);

    //checksum so far

    uint8_t checksum = 0;
    for (int i = 0; i < packet.size(); i++) {
        checksum ^= packet[i];
    }
    packet.push_back(checksum);
    
    ssize_t written = write(fd_, packet.data(), packet.size());
    if (written != packet.size()) {
        CALIBUR_LOG_ERROR(g_logger) << "Write failed: yaw, pitch not sent " 
                                    << written << ", error: " << strerror(errno);
        return false;
    }
    
    CALIBUR_LOG_DEBUG(g_logger) << "Sent data: yaw=" << yaw << ", pitch=" << pitch 
                                << ", is_fire=" << (is_fire ? "true" : "false");
    }

    // Packet 2: 0x02 - aim, fire, chase (three ints, 12 bytes)
    {
    uint16_t len = 12;
    uint8_t data_type = 0x02;
    std::vector<uint8_t> packet_2 = {0xAA, static_cast<uint8_t>(len & 0xFF), static_cast<uint8_t>((len >> 8) & 0xFF), data_type};

    // aim
    uint8_t aim_bytes[4];
    memcpy(aim_bytes, &out.aim, 4);
    packet_2.insert(packet_2.end(), aim_bytes, aim_bytes + 4);

    // fire
    uint8_t fire_bytes[4];
    memcpy(fire_bytes, &out.fire, 4);
    packet_2.insert(packet_2.end(), fire_bytes, fire_bytes + 4);

    // chase
    uint8_t chase_bytes[4];
    memcpy(chase_bytes, &out.chase, 4);
    packet_2.insert(packet_2.end(), chase_bytes, chase_bytes + 4);

    uint8_t checksum_2 = 0;
    for (int i = 0; i < packet_2.size(); i++) {
        checksum_2 ^= packet_2[i];
    }
    packet_2.push_back(checksum_2);

    ssize_t written = write(fd_, packet_2.data(), packet_2.size());
    if (written != packet_2.size()) {
        CALIBUR_LOG_ERROR(g_logger) << "Failed to send aim/fire/chase packet: wrote " << written << " of " << packet_2.size();
        return false;
    }
    }

    CALIBUR_LOG_DEBUG(g_logger) << "Sent prediction: yaw=" << out.yaw << ", pitch=" << out.pitch
                                << ", aim=" << out.aim << ", fire=" << out.fire << ", chase=" << out.chase;
    return true;
}