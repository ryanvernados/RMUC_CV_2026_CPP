#include "types.hpp"
#include <thread>
#include "workers.hpp"
#include "rbpf.cuh"

#include <rerun.hpp> // [RERUN CHANGE]
#include <deque> //[RERUN CHANGE]
#include <cmath>  // [RERUN CHANGE]
#include <vector>  // [RERUN FIX] needed for LineStrips3D track conversion


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

// ===================== [RERUN CHANGE] helpers =====================

// [RERUN FIX] Mapping for your diagnosed state frame:
// state: x=right/left, y=up(height), z=forward(depth)
// rerun: X=right/left, Y=forward, Z=up
static inline rerun::datatypes::Vec3D to_rerun_xyz(float x, float y, float z) {
    return { x, z, y };
}

static inline rerun::datatypes::Quaternion quat_from_yaw(float yaw) {
    const float h = 0.5f * yaw;
    return rerun::datatypes::Quaternion::from_xyzw(
        0.0f, 0.0f, std::sin(h), std::cos(h)
    );
}

// [RERUN FIX] rotate in X-Z plane (yaw about +Y because Y is UP)
static inline void rot_xz(float yaw, float x, float z, float& ox, float& oz) {
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    ox = c * x - s * z;
    oz = s * x + c * z;
}


static void log_robot_glyph(
    rerun::RecordingStream& rec,
    const char* entity_prefix,
    const RobotState& s,
    const rerun::Color& col
) {
    const float x   = s.state[IDX_TX];
    const float y   = s.state[IDX_TY];
    const float z   = s.state[IDX_TZ];
    const float yaw = s.state[IDX_YAW];
    const float r1  = s.state[IDX_R1];
    const float r2  = s.state[IDX_R2];
    const float h   = s.state[IDX_H];

    // Plate dimensions (tune if needed)
    constexpr float T = 0.01f;   // thickness
    constexpr float W = 0.08f;   // width
    const float H = (h > 1e-4f) ? h : 0.12f;

    // [RERUN FIX] in state frame: X=left/right, Z=forward
    // front/back are ±r1 along Z, left/right are ±r2 along X
    const float lx[4] = {  0.0f,  0.0f, +r2, -r2 }; // x offsets
    const float lz[4] = { +r1,  -r1,  0.0f,  0.0f }; // z offsets


    // [RERUN CHANGE] use datatypes::Vec3D (matches with_* APIs)
    std::array<rerun::datatypes::Vec3D, 4> centers;
    for (int i = 0; i < 4; ++i) {
        float dx, dz;
        rot_xz(yaw, lx[i], lz[i], dx, dz);
        centers[i] = to_rerun_xyz(x + dx, y, z + dz);   // y(height) stays the same
    }

    std::array<rerun::datatypes::Vec3D, 4> half_sizes = {
        rerun::datatypes::Vec3D{W/2, T/2, H/2}, // front  (wide in X, thin in Y)
        rerun::datatypes::Vec3D{W/2, T/2, H/2}, // back
        rerun::datatypes::Vec3D{T/2, W/2, H/2}, // left   (thin in X, wide in Y)
        rerun::datatypes::Vec3D{T/2, W/2, H/2}, // right
    };


    const auto q = quat_from_yaw(yaw);
    std::array<rerun::datatypes::Quaternion, 4> quats = { q, q, q, q };

    // [RERUN CHANGE] Boxes3D default ctor + builder methods (your SDK expects this)
    rec.log(
        std::string(entity_prefix).append("/armor").c_str(),
        rerun::Boxes3D()
            .with_half_sizes(half_sizes)
            .with_centers(centers)
            .with_quaternions(quats)
            .with_colors({col, col, col, col})
    );

    // Heading arrow
    constexpr float ARROW_LEN = 0.25f;
    rec.log(
        std::string(entity_prefix).append("/heading").c_str(),
        rerun::Arrows3D::from_vectors({{ARROW_LEN * std::sin(yaw), ARROW_LEN * std::cos(yaw), 0.0f}})
            // [RERUN FIX] map origin to rerun coords
            .with_origins({to_rerun_xyz(x, y, z)})
            .with_colors({col})
    );
}

// =================== [RERUN CHANGE] track helper ===================

static void push_track_and_log(
    rerun::RecordingStream& rec,
    std::deque<rerun::datatypes::Vec3D>& track,
    const char* entity,
    float x, float y, float z,
    const rerun::Color& col,
    size_t max_len
) {
    track.emplace_back(x, y, z);
    if (track.size() > max_len) track.pop_front();

    if (track.size() < 2) return;

    // [RERUN FIX] make a concrete polyline container (more robust than initializer_list inference)
    std::vector<rerun::datatypes::Vec3D> pts(track.begin(), track.end());
    std::vector<std::vector<rerun::datatypes::Vec3D>> strips;
    strips.emplace_back(std::move(pts));

    // 1) Thick line strip (so it's visible)
    rec.log(
        entity,
        rerun::LineStrips3D(strips)
            .with_colors({col})
            .with_radii({0.005f})  // [RERUN FIX] increase if still not visible
    );

}



// ===================== [RERUN CHANGE] end =====================



void PFWorker::operator()() {
    gpu_pf_init();

    auto next_tick = timestamp_clock_t::now();
    
    constexpr int MAX_FRAMES_WITHOUT_DETECTION = 30;  // ~0.3 seconds at 100Hz
    bool pf_initialized = false;
    
    // [RERUN] ---- create a recording stream once (per process) ----
    static rerun::RecordingStream rec("RMUC_PF_Debug");
    static bool rerun_started = false;
    static uint64_t tick = 0;
    static std::deque<rerun::datatypes::Vec3D> meas_track;
    static std::deque<rerun::datatypes::Vec3D> pf_track;
    constexpr size_t MAX_TRACK_LEN = 300; // ~3 seconds at 100Hz

    if (!rerun_started) {
        // In your SDK build, connect_grpc returns rerun::Error.
        // Convention: default-constructed Error means "ok".
        const rerun::Error err = rec.connect_grpc("rerun+http://127.0.0.1:9876/proxy");
        if (err.is_err()) {
            std::cerr << "[RERUN] connect_grpc failed\n";
        } else {
            std::cout << "[RERUN] connected to viewer\n";
        }
        rerun_started = true;
    }
    // [RERUN] ------------------------------------------------------


    while (!stop_.load(std::memory_order_acquire)) {
        next_tick += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next_tick);
        
        // [RERUN] set time for this thread’s subsequent logs
        rec.set_time_sequence("tick", tick++);  // per-thread timeline :contentReference[oaicite:2]{index=2}
        // [RERUN] --------------------------------------------

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

            // [RERUN CHANGE] robot glyph + measurement track
            log_robot_glyph(rec, "world/meas_robot", meas, rerun::Color(0, 255, 0));

            // [RERUN FIX] map meas point into rerun coords for track
            const auto mrr = to_rerun_xyz(meas.state[IDX_TX], meas.state[IDX_TY], meas.state[IDX_TZ]);
            push_track_and_log(
                rec, meas_track, "world/meas_track",
                mrr.x(), mrr.y(), mrr.z(),   // [RERUN FIX]
                rerun::Color(0, 255, 0),
                MAX_TRACK_LEN
            );
            
            // If not initialized or diverged, initialize/reset from measurement
            if (!pf_initialized) {
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
        
        // [RERUN CHANGE] robot glyph + PF track
        log_robot_glyph(rec, "world/pf_robot", pf_state, rerun::Color(255, 0, 0));

        // [RERUN FIX] map pf point into rerun coords for track
        const auto prr = to_rerun_xyz(pf_state.state[IDX_TX], pf_state.state[IDX_TY], pf_state.state[IDX_TZ]);
        push_track_and_log(
            rec, pf_track, "world/pf_track",
            prr.x(), prr.y(), prr.z(),   // [RERUN FIX]
            rerun::Color(255, 0, 0),
            MAX_TRACK_LEN
        );
        // [RERUN] --------------------------------------------

        // std::cout << "[PF ] x=" << pf_state.state[IDX_TX]
        //           << " y=" << pf_state.state[IDX_TY]
        //           << " z=" << pf_state.state[IDX_TZ]
        //           << " yaw=" << pf_state.state[IDX_YAW]
        //           << " h= " << pf_state.state[IDX_H]
        //           << " r1= " << pf_state.state[IDX_R1]
        //           << " r2= " << pf_state.state[IDX_R2] << std::endl;


        shared_.pf_out = std::make_shared<RobotState>(pf_state);
        shared_.pf_ver.fetch_add(1, std::memory_order_acq_rel);
    }
}