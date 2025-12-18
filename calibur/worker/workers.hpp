#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include <unordered_map>

#include "../imu/imu_reader.hpp"
#include "../imu/imu_data.hpp"
#include "../usb_communication.h"

#include "types.hpp"
#include "rbpf.cuh"
#include "infer.h"

namespace calibur { class USBCommunication; }

const std::string YOLO_MODEL_PATH   = "./calibur/models/best.engine";
const std::string VIDEO_PATH        = "./sample_videos/video1.mp4";

#define YOLO_CONFIDENCE_THRESHOLD               0.5f
#define DEFAULT_ROBOT_RADIUS                    0.2f
#define DEFAULT_ROBOT_HEIGHT                    0.0f
#define SELECTOR_TTL                            0.5f
static constexpr int NUM_PARTICLES = 10000;
#define ALPHA_BULLET_SPEED                      0.1f
#define ALPHA_PROCESSING_TIME                   0.1f
#define PREDICTION_CONVERGENCE_THRESHOLD        0.01f
#define CHASE_THREASHOLD                        6.0f
#define PRED_CONV_MAX_ITERS                     10
#define WIDTH_TOLERANCE                         0.13f
#define HEIGHT_TOLERANCE                        0.13f
#define TOLERANCE_COEFF                         1.0f

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

    void operator()();

private:
    void* cam_;
    SharedLatest& shared_;
    std::atomic<bool>& stop_;
    CameraMode mode_;
    cv::VideoCapture cap_;
    bool use_stub_ = false;

    void grab_frame_stub(CameraFrame& frame);
    void grab_frame_from_hik(CameraFrame& frame);
    void grab_frame_from_video(CameraFrame& frame);
};

class IMUWorker {
public:
    IMUWorker(SharedLatest &shared, std::atomic<bool> &stop_flag);
    IMUWorker(const IMUWorker&) = delete;
    IMUWorker& operator=(const IMUWorker&) = delete;
    IMUWorker(IMUWorker&&) = default;
    IMUWorker& operator=(IMUWorker&&) = default;
    void operator()();

private:
    SharedLatest &shared_;
    std::atomic<bool> &stop_flag_;
    IMUReader reader_;
};

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
    YoloDetector        detector_;
};

class DetectionWorker {
public:
    DetectionWorker(SharedLatest &shared, SharedScalars &shared_scalars,
                    std::atomic<bool> &stop_flag);
    void operator()();

private:
    SharedLatest    &shared_;
    SharedScalars   &scalars_;
    std::atomic<bool> &stop_;
    TimePoint   start_time_;
    int         selected_robot_id_;
    float       ttl_;
    float       initial_yaw_;
    uint64_t    last_cam_ver_;
    TrackingFSM tracking_fsm_ = TrackingFSM::TARGET_SEARCHING;
    RobotState prev_robot_{};
    bool       has_prev_robot_ = false;
    cv::Mat     camera_matrix;
    cv::Mat     dist_coeffs;
    std::unordered_map<int, float> yaw_smooth_state;
    float yaw_alpha = 0.3f;

    void sleep_small();
    std::vector<DetectionResult>
    yolo_predict(const std::vector<uint8_t> &raw, int width, int height, std::vector<DetectionResult> &dets);
    std::vector<DetectionResult> refine_keypoints(std::shared_ptr<CameraFrame> camera_frame,
        std::vector<DetectionResult> &dets);
    bool solvepnp_and_yaw(DetectionResult &det,
                                      const Eigen::Matrix3f &R_cam2world);
    void group_armors(const std::vector<DetectionResult> &dets,
                      std::vector<std::vector<DetectionResult>> &grouped);
    void select_armor(const std::vector<std::vector<DetectionResult>> &grouped_armors,
                 std::vector<DetectionResult> &selected_armors);
    std::unique_ptr<RobotState>
    form_robot(const std::vector<DetectionResult> &armors);
};

struct RBPFPosYawModelGPU;

class PFWorker {
public:
    PFWorker(SharedLatest &shared, std::atomic<bool> &stop_flag);
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
    int frames_without_detection_;  
    std::unique_ptr<RBPFPosYawModelGPU> g_pf;
    void gpu_pf_init();
    void gpu_pf_reset(const RobotState &meas);
    void gpu_pf_predict_only();
    void gpu_pf_step(const RobotState &meas);
    bool is_state_valid(const RobotState &state);
    RobotState gpu_return_result();
};

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
    float vis_yaw_   = 0.0f;
    float vis_pitch_ = 0.0f;
    bool  vis_init_  = false;

    void sleep_small();
    void compute_prediction(const RobotState &rs,
                            const std::shared_ptr<IMUState>& imu,
                            float bullet_speed,
                            PredictionOut &out);
    double time_to_double(const TimePoint &tp);
};

class USBWorker {
public:
    USBWorker(SharedLatest &shared,
              SharedScalars &scalars,
              std::atomic<bool> &stop_flag,
              std::shared_ptr<calibur::USBCommunication> usb_comm_);
    void operator()();

private:
    SharedLatest    &shared_;
    SharedScalars   &scalars_;
    std::atomic<bool> &stop_;
    uint64_t        last_pred_ver_;
    void process_usb_rx();
    void usb_send_tx(const PredictionOut &out);
    std::shared_ptr<calibur::USBCommunication> usb_comm_;
};

class DisplayWorker {
public:
    DisplayWorker(SharedLatest &shared, std::atomic<bool> &stop_flag);
    void operator()();

private:
    SharedLatest &shared_;
    std::atomic<bool> &stop_flag_;
    void draw_crosshair(cv::Mat &img, const PredictionOut *pred);
    void draw_yolo_overlay(cv::Mat &img, const YoloOutput *yolo);
    void draw_refined_overlay(cv::Mat &img, const std::vector<DetectionResult>* refined_dets);
    void draw_info_panel(cv::Mat &img, const PredictionOut *pred, double fps);
    void draw_target_dot(cv::Mat &img, const PredictionOut *pred, int width, int height);
};