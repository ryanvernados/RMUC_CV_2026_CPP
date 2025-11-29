#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include <unordered_map>

#include "types.hpp"
#include "rbpf.cuh"
#include "infer.h"


// ------------------------------------------- Constants -------------------------------------------
// ------------- File Paths ------------------------
const std::string YOLO_MODEL_PATH   = "./calibur/models/best.engine";  //relative to the binary executable
const std::string VIDEO_PATH        = "./sample_videos/video1.mp4";

// const std::string kOnnxPath         = "../calibur/models/best.onnx";

// ------------- Camera Worker ---------------------
// #define USE_VIDEO_FILE
#define DISPLAY_DETECTION
#define PERFORMANCE_BENCHMARK

// ------------- Detection Constants ---------------
#define YOLO_CONFIDENCE_THRESHOLD               0.5f

#define DEFAULT_ROBOT_RADIUS                    0.2f
#define DEFAULT_ROBOT_HEIGHT                    0.0f
#define SELECTOR_TTL                            0.5f    // seconds

// ------------- PF constants ----------------------
// #define PF_CONDITIONAL_RESAMPLE                
static constexpr int NUM_PARTICLES = 10000;


// ------------- Prediction Constants --------------
#define ALPHA_BULLET_SPEED                      0.1f    // low-pass filter coeff, detemine using cutoff freq formula: alpha = 2πf_c * dt / (2πf_c * dt + 1)
#define ALPHA_PROCESSING_TIME                   0.1f
#define PREDICTION_CONVERGENCE_THRESHOLD        0.01f
#define CHASE_THREASHOLD                        6.0f
#define PRED_CONV_MAX_ITERS                     10
#define WIDTH_TOLERANCE                         0.13f  // meters
#define HEIGHT_TOLERANCE                        0.13f  // meters
#define TOLERANCE_COEFF                         1.0f

//--------------------------------------------Camera Worker--------------------------------------------

enum class CameraMode {
    HIK_USB,
    VIDEO_FILE
};


class CameraWorker {
public:
    CameraWorker(void* cam_handle,
                 SharedLatest& shared,
                 std::atomic<bool>& stop_flag,
                 CameraMode mode);

    void operator()();  // For thread pool execution

private:
    void* cam_;                // Hikvision handle
    SharedLatest& shared_;
    std::atomic<bool>& stop_;
    CameraMode mode_;

    // Only used for VIDEO_FILE mode
    cv::VideoCapture cap_;
    bool use_stub_ = false;

    void grab_frame_stub(CameraFrame& frame);
    void grab_frame_from_hik(CameraFrame& frame);
    void grab_frame_from_video(CameraFrame& frame);
};

//--------------------------------------------IMU Worker--------------------------------------------

class IMUWorker {
public:
    IMUWorker(SharedLatest &shared,
              std::atomic<bool> &stop_flag);

    void operator()();

private:
    SharedLatest    &shared_;
    std::atomic<bool> &stop_;

    // Replace with real IMU driver in .cpp
    bool read_imu_stub(IMUState &imu);
};

//--------------------------------------------YOLO Worker--------------------------------------------
class YoloWorker {
public:
    YoloWorker(SharedLatest& shared,
               std::atomic<bool>& stop_flag,
               const std::string& engine_path);

    ~YoloWorker() = default;

    void operator()();

    YoloWorker(const YoloWorker&) = delete;
    YoloWorker& operator=(const YoloWorker&) = delete;
    YoloWorker(YoloWorker&&) = default;
    YoloWorker& operator=(YoloWorker&&) = default;

private:
    SharedLatest&       shared_;
    std::atomic<bool>&  stop_;
    uint64_t            last_cam_ver_ = 0;
    YoloDetector        detector_;      // <-- persistent member
};

//--------------------------------------------Detection Worker--------------------------------------------

class DetectionWorker {
public:
    DetectionWorker(SharedLatest &shared,
                    std::atomic<bool> &stop_flag);

    void operator()();

private:
    SharedLatest    &shared_;
    std::atomic<bool> &stop_;

    // "class variables"
    TimePoint   start_time_;
    int         selected_robot_id_;
    float       ttl_;
    float       initial_yaw_;
    uint64_t    last_cam_ver_;
    RobotState prev_robot_{};
    bool       has_prev_robot_ = false;

    // Camera intrinsics
    cv::Mat     camera_matrix;
    cv::Mat     dist_coeffs;

    // Yaw optimization helpers
    std::unordered_map<int, float> yaw_smooth_state;
    float yaw_alpha = 0.3f; // smoothing factor: 0 = very smooth, 1 = no smoothing

    void sleep_small();

    // ---- Interfaces for you to implement in .cpp ----

    std::vector<DetectionResult>
    yolo_predict(const std::vector<uint8_t> &raw, int width, int height, std::vector<DetectionResult> &dets);

    void refine_keypoints(std::vector<DetectionResult> &dets,
                          int width, int height);

    void solvepnp_and_yaw(std::vector<DetectionResult> &dets);

    void group_armors(const std::vector<DetectionResult> &dets,
                      std::vector<std::vector<DetectionResult>> &grouped);


    void select_armor(const std::vector<std::vector<DetectionResult>> &grouped,
                 float &ttl,
                 int &selected_robot_id,
                 float &initial_yaw,
                 std::vector<DetectionResult> &selected);

    std::unique_ptr<RobotState>
    form_robot(const std::vector<DetectionResult> &armors);
};


//--------------------------------------------PF Worker--------------------------------------------
struct RBPFPosYawModelGPU;

class PFWorker {
public:
    PFWorker(SharedLatest &shared,
             std::atomic<bool> &stop_flag);

    // Runs as dedicated thread (not via pool)
    void operator()();

    PFWorker(const PFWorker&) = delete;
    PFWorker& operator=(const PFWorker&) = delete;
    PFWorker(PFWorker&&) = default;
    PFWorker& operator=(PFWorker&&) = default;

private:
    static constexpr float kDt = 0.01f;
    SharedLatest      &shared_;
    std::atomic<bool> &stop_;
    uint64_t           last_det_ver_ = 0;

    // Heap-allocated PF model
    std::unique_ptr<RBPFPosYawModelGPU> g_pf;

    // PF / CUDA interfaces to implement in .cpp
    void gpu_pf_init();
    void gpu_pf_reset(const RobotState &meas);
    void gpu_pf_predict_only();
    void gpu_pf_step(const RobotState &meas);
    RobotState gpu_return_result();
};


//--------------------------------------------Prediction Worker--------------------------------------------

class PredictionWorker {
public:
    PredictionWorker(SharedLatest &shared,
                     SharedScalars &scalars,
                     std::atomic<bool> &stop_flag);

    void operator()();

private:
    SharedLatest    &shared_;
    SharedScalars   &scalars_;
    std::atomic<bool> &stop_;
    uint64_t        last_pf_ver_;
    float           bullet_speed = 20.0f; 
    float           processing_time = 0.05f;
    float           t_gimbal_actuation = 0.1f;
    int            fire_state = false;
    int            chase_state = false;
    int            aim_state = false;

    void sleep_small();

    void compute_prediction(const RobotState &rs,
                            const IMUState *imu,
                            float bullet_speed,
                            PredictionOut &out);

    double time_to_double(const TimePoint &tp);
};

//--------------------------------------------Prediction Worker--------------------------------------------

class USBWorker {
public:
    USBWorker(SharedLatest &shared,
              SharedScalars &scalars,
              std::atomic<bool> &stop_flag);

    void operator()();

private:
    SharedLatest    &shared_;
    SharedScalars   &scalars_;
    std::atomic<bool> &stop_;
    uint64_t        last_pred_ver_;

    void process_usb_rx();
    void usb_send_tx(const PredictionOut &out);
};


