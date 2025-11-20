#pragma once

#include <vector>
#include <utility>
#include <cstdint>
#include <chrono>
#include <memory>
#include <atomic>

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

#define ROBOT_STATE_VEC_LEN     15

// =======================
// Data Structures
// =======================

struct CameraFrame {
    std::vector<uint8_t> raw_data;
    TimePoint            timestamp;
    int                  width  = 640;
    int                  height = 480;
};

struct IMUState {
    std::vector<float> euler_angle;   // [roll, pitch, yaw] etc.
    float              time = 0.0f;   // seconds since some ref
    std::vector<float> quaternion;    // optional, may be empty
    TimePoint          timestamp;
};

struct DetectionResult {
    cv::Rect detection;
    std::vector<cv::Point2f> keypoints;

    int   class_id         = -1;
    float confidence_level = 0.0f;

    cv::Vec3f tvec;  // size 3
    cv::Vec3f rvec;  // size 3

    float yaw_rad    = 0.0f;
    int   armor_type = 0;     // 0 = small, 1 = big, etc.
};

enum PfStateFlag {
    PF_STATE_OK    = 0,
    PF_STATE_RESET = 1
};

struct RobotState {
    std::array<float, ROBOT_STATE_VEC_LEN> state; // size 15
    int   class_id  = -1;
    int   pf_state  = PF_STATE_OK;
    TimePoint timestamp;
};

struct PredictionOut {
    float yaw   = 0.0f;
    float pitch = 0.0f;
    int   aim   = 0;
    int   fire  = 0;
    int   chase = 0;
};


// Shared latest values plus version counters for
// "use-latest, consume-once" semantics.

struct SharedLatest {
    // Data
    std::shared_ptr<CameraFrame>   camera;
    std::shared_ptr<IMUState>      imu;
    std::shared_ptr<RobotState>    detection_out;
    std::shared_ptr<RobotState>    pf_out;
    std::shared_ptr<PredictionOut> prediction_out;

    // Version counters (increment per new publish)
    std::atomic<uint64_t> camera_ver     {0};
    std::atomic<uint64_t> imu_ver        {0};
    std::atomic<uint64_t> detection_ver  {0};
    std::atomic<uint64_t> pf_ver         {0};
    std::atomic<uint64_t> prediction_ver {0};
};

struct SharedScalars {
    std::atomic<float> bullet_speed;

    SharedScalars() {
        bullet_speed.store(20.0f, std::memory_order_relaxed); // default m/s
    }
};
