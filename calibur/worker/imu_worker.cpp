// calibur/worker/imu_worker.cpp
#include <thread>
#include <chrono>
#include <iostream>

#include "workers.hpp"

IMUWorker::IMUWorker(SharedLatest &shared, std::atomic<bool> &stop_flag)
    : shared_(shared),
      stop_flag_(stop_flag),
      reader_("/dev/ttyACM0", 460800, 0.2f)
{
}

void IMUWorker::operator()() {
    reader_.start();

    size_t print_counter = 0;  // throttle printing

    while (!stop_flag_.load(std::memory_order_relaxed)) {
        IMUData raw;
        if (!reader_.get_latest(raw)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        IMUState st;

        // euler_angle = [roll, pitch, yaw] in degrees
        if (raw.has_euler) {
            st.euler_angle.clear();
            st.euler_angle.reserve(3);

            float pitch = raw.euler_deg[0];
            float roll  = raw.euler_deg[1];
            float yaw   = raw.euler_deg[2];

            st.euler_angle.push_back(roll);
            st.euler_angle.push_back(pitch);
            st.euler_angle.push_back(yaw);
        }

        // quaternion
        if (raw.has_quat) {
            st.quaternion.clear();
            st.quaternion.reserve(4);
            st.quaternion.push_back(raw.quat[0]);
            st.quaternion.push_back(raw.quat[1]);
            st.quaternion.push_back(raw.quat[2]);
            st.quaternion.push_back(raw.quat[3]);
        }

        // time from IMU
        if (raw.has_ts_sample) {
            st.time = raw.ts_sample_us * 1e-6f;
        } else {
            st.time = 0.0f;
        }

        st.timestamp = Clock::now();

        // write to shared memory
        auto ptr = std::make_shared<IMUState>(st);
        std::atomic_store(&shared_.imu, ptr);
        shared_.imu_ver.fetch_add(1, std::memory_order_relaxed);

        // -------------------------------
        // PRINT OUTPUT EVERY ~10 ITERATIONS
        // -------------------------------
        if (++print_counter >= 10) {
            print_counter = 0;

            std::cout << "[IMU] ";

            if (!st.euler_angle.empty()) {
                std::cout << "Roll="  << st.euler_angle[0]
                          << "  Pitch=" << st.euler_angle[1]
                          << "  Yaw="   << st.euler_angle[2];
            }

            if (!st.quaternion.empty()) {
                std::cout << "  |  Q=("
                          << st.quaternion[0] << ", "
                          << st.quaternion[1] << ", "
                          << st.quaternion[2] << ", "
                          << st.quaternion[3] << ")";
            }

            if (raw.has_temp) {
                std::cout << "  |  Temp=" << raw.temp_c << "°C";
            }

            std::cout << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    reader_.stop();
}
