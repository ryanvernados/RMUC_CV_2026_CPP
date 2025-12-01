// calibur/imu/imu_reader.hpp
#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstdint>

#include "imu_data.hpp"

class IMUReader {
public:
    IMUReader(const std::string &device,
              int baud = 460800,
              float timeout_s = 0.2f);

    ~IMUReader();

    void start();
    void stop();

    // Thread-safe copy of latest IMU data
    bool get_latest(IMUData &out);

private:
    void worker();

    int  open_serial();
    void close_serial();

    bool read_exact(uint8_t *buf, size_t n);
    bool read_frame(IMUData &out);

private:
    std::string device_;
    int         baud_;
    float       timeout_s_;

    int fd_;   // POSIX file descriptor

    std::atomic<bool> running_;
    std::thread       thread_;

    std::mutex data_mutex_;
    IMUData    latest_;
    bool       has_data_;
};
