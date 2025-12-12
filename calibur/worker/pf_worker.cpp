#include "types.hpp"
#include <thread>
#include "workers.hpp"
#include "rbpf.cuh"

using timestamp_clock_t= std::chrono::steady_clock;

PFWorker::PFWorker(SharedLatest &shared,
                   std::atomic<bool> &stop_flag)
    : shared_(shared),
      stop_(stop_flag),
      last_det_ver_(0),
      g_pf(nullptr),
      frames_without_detection_(0) 
{
}

void PFWorker::gpu_pf_init() {
    if (!g_pf) {
        g_pf.reset(rbpf_create(NUM_PARTICLES));
        RobotState init{};
        // Initialize to zero/invalid state
        for (int i = 0; i < ROBOT_STATE_VEC_LEN; i++) {
            init.state[i] = 0.0f;
        }
        rbpf_reset_from_meas(g_pf.get(), init);
    }
}

void PFWorker::gpu_pf_reset(const RobotState &meas) {
    rbpf_reset_from_meas(g_pf.get(), meas);
}

void PFWorker::gpu_pf_predict_only() {
    rbpf_predict(g_pf.get(), kDt);
}

void PFWorker::gpu_pf_step(const RobotState &meas) {
    rbpf_step(g_pf.get(), meas, kDt);
}

RobotState PFWorker::gpu_return_result(){
    return rbpf_get_mean(g_pf.get());
}

bool PFWorker::is_state_valid(const RobotState &state) {
    // Check for NaN/Inf
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(state.state[i])) {
            return false;
        }
    }
    
    // Check if position is reasonable (within 100m)
    const float dist_sq = state.state[0]*state.state[0] + 
                         state.state[1]*state.state[1] + 
                         state.state[2]*state.state[2];
    
    if (dist_sq > 100.0f * 100.0f) {  // More than 100m
        return false;
    }
    
    return true;
}

void PFWorker::operator()() {
    gpu_pf_init();

    auto next_tick = timestamp_clock_t::now();
    
    constexpr int MAX_FRAMES_WITHOUT_DETECTION = 30;  // ~0.3 seconds at 100Hz
    bool pf_initialized = false;
    
    while (!stop_.load(std::memory_order_acquire)) {
        next_tick += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next_tick);
        
        bool       has_meas = false;
        RobotState meas;
        
        uint64_t det_ver = shared_.detection_ver.load(std::memory_order_acquire);
        if (det_ver != last_det_ver_) {
            auto det_ptr = shared_.detection_out;
            if (det_ptr) {
                meas          = *det_ptr;
                last_det_ver_ = det_ver;
                has_meas      = true;
            }
        }

        RobotState pf_state;
        
        if (has_meas) {
            // ============================================================
            // Got new detection (already in WORLD frame)
            // ============================================================
            
            // std::cout << "[PF] got meas (WORLD frame): "
            //           << "x=" << meas.state[IDX_TX]
            //           << " y=" << meas.state[IDX_TY]
            //           << " z=" << meas.state[IDX_TZ]
            //           << " yaw=" << meas.state[IDX_YAW]
            //           << std::endl;
            
            // Validate detection
            if (!is_state_valid(meas)) {
                std::cout << "[PF WARNING] Invalid detection received, skipping\n";

                continue;
            }

            // If not initialized or diverged, initialize/reset from measurement
            if (!pf_initialized) {
                std::cout << "[PF] Initializing from first detection\n";
                gpu_pf_reset(meas);
                pf_initialized = true;
            } else {
                // Normal update
                gpu_pf_step(meas);
            }
            
            frames_without_detection_ = 0;  // Reset counter
            
        } else {
            // ============================================================
            // No new detection - just predict
            // ============================================================
            
            frames_without_detection_++;
            
            if (!pf_initialized) {
                // Don't predict if never initialized
                continue;
            }
            
            if (frames_without_detection_ > MAX_FRAMES_WITHOUT_DETECTION) {
                std::cout << "[PF WARNING] No detection for " << frames_without_detection_ 
                          << " frames, PF may diverge. Waiting for new detection...\n";
                // Don't predict anymore - wait for new detection to reset
                continue;
            }
            
            // Predict
            gpu_pf_predict_only();
        }
        
        if (!pf_initialized) {
            continue;  // Don't output until initialized
        }
        
        pf_state = gpu_return_result();
        
        // ============ VALIDATE PF OUTPUT ============
        if (!is_state_valid(pf_state)) {
            std::cout << "[PF ERROR] PF output invalid/diverged! "
                      << "x=" << pf_state.state[IDX_TX]
                      << " y=" << pf_state.state[IDX_TY]
                      << " z=" << pf_state.state[IDX_TZ] << std::endl;


            // Force re-initialization on next detection
            pf_initialized = false;
            frames_without_detection_ = MAX_FRAMES_WITHOUT_DETECTION + 1;
            continue;
        }
        
        std::cout << "[PF ] x=" << pf_state.state[IDX_TX]
                  << " y=" << pf_state.state[IDX_TY]
                  << " z=" << pf_state.state[IDX_TZ]
                  << " yaw=" << pf_state.state[IDX_YAW]
                  << " h= " << pf_state.state[IDX_H]
                  << " r1= " << pf_state.state[IDX_R1]
                  << " r2= " << pf_state.state[IDX_R2] << std::endl;


        shared_.pf_out = std::make_shared<RobotState>(pf_state);
        shared_.pf_ver.fetch_add(1, std::memory_order_acq_rel);
    }
}