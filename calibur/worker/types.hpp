#pragma once

#include <vector>
#include <utility>
#include <cstdint>
#include <chrono>
#include <memory>
#include <atomic>

#include <opencv2/core.hpp>
#include <Eigen/Dense>


using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

#define ROBOT_STATE_VEC_LEN     15

// =======================
// Data Structures
// =======================

// State indices in X
enum StateIdx {
    IDX_TX = 0, IDX_TY = 1, IDX_TZ = 2,
    IDX_VX = 3, IDX_VY = 4, IDX_VZ = 5,
    IDX_AX = 6, IDX_AY = 7, IDX_AZ = 8,
    IDX_YAW   = 9, IDX_OMEGA = 10, IDX_ALPHA = 11,
    IDX_R1 = 12, IDX_R2 = 13, IDX_H  = 14
};

struct CameraFrame {
    cv::Mat      raw_data;
    TimePoint    timestamp;
    int          width  = 640;
    int          height = 640;
};

struct IMUState {
    std::vector<float> euler_angle;   // [roll, pitch, yaw] etc.
    float              time = 0.0f;   // seconds since some ref
    std::vector<float> quaternion;    // optional, may be empty
    TimePoint          timestamp;
};


struct DetectionResult {
    cv::Rect bbox;
    std::vector<cv::Point2f> keypoints;

    int   class_id         = -1;
    float confidence_level = 0.0f;

    Eigen::Vector3f tvec;  // size 3
    cv::Vec3f rvec;  // size 3

    float yaw_rad    = 0.0f;
    int   armor_type = 0;     // 0 = small, 1 = big, etc.
};

struct YoloOutput {
    std::vector<DetectionResult>    dets;
    int                             width;
    int                             height;
    TimePoint                       timestamp;
};

enum PfStateFlag {
    PF_STATE_OK    = 0,
    PF_STATE_RESET = 1
};

struct RobotState {
    // [x, y, z, vx, vy, vz, ax, ay, az, yaw, omega, alpha, r1, r2, h]
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
    std::shared_ptr<YoloOutput> yolo;

    // Version counters (increment per new publish)
    std::atomic<uint64_t> camera_ver     {0};
    std::atomic<uint64_t> imu_ver        {0};
    std::atomic<uint64_t> detection_ver  {0};
    std::atomic<uint64_t> pf_ver         {0};
    std::atomic<uint64_t> prediction_ver {0};
    std::atomic<uint64_t> yolo_ver       {0};
};

struct SharedScalars {
    std::atomic<float> bullet_speed;

    SharedScalars() {
        bullet_speed.store(20.0f, std::memory_order_relaxed); // default m/s
    }
};
