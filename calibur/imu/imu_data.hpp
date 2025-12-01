// calibur/imu/imu_data.hpp
#pragma once
#include <cstdint>

struct IMUData {
    uint16_t seq = 0;

    bool has_temp = false;
    float temp_c = 0.0f;

    bool has_accel = false;
    float accel_mps2[3] = {0.0f, 0.0f, 0.0f};

    bool has_gyro = false;
    float gyro_dps[3] = {0.0f, 0.0f, 0.0f};

    bool has_mag_norm = false;
    float mag_norm[3] = {0.0f, 0.0f, 0.0f};

    bool has_mag_mg = false;
    float mag_mg[3] = {0.0f, 0.0f, 0.0f};

    bool has_euler = false;
    // pitch, roll, yaw from sensor (deg)
    float euler_deg[3] = {0.0f, 0.0f, 0.0f};

    bool has_quat = false;
    // w, x, y, z
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    bool has_ts_sample = false;
    uint32_t ts_sample_us = 0;

    bool has_ts_dataready = false;
    uint32_t ts_dataready_us = 0;

    bool has_fusion = false;
    uint8_t fusion_state = 0;
    uint8_t gnss_state = 0;
};
