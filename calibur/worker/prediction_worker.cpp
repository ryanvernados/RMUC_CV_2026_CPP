#include <atomic>
#include <vector>
#include <cmath>
#include <thread>
#include <algorithm> 
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "types.hpp"
#include "workers.hpp"
#include "helper.hpp"

// ===================== Helper Prototypes =========================

inline void  filtering(float &value, float measurement, float alpha);
inline bool  is_converged(float v, float threshold);
inline float t_lead_calculation(const Eigen::Vector3f &tvec, const float &bullet_speed);
inline int   sector_from_yaw(const float yaw);
inline void  calculate_robot_final_target_point(Eigen::Vector3f &final_pos, float &final_yaw,
                                                const float height_offset, const float r1, const float r2);
inline void  motion_model_robot_pos(const std::array<float, ROBOT_STATE_VEC_LEN> &state, 
                                    Eigen::Vector3f &robot_center_lead,
                                    float &yaw_lead, const float &t);
inline void  calculate_gimbal_correction(const Eigen::Vector3f &tvec, Eigen::Vector2f &correction);
inline int   should_fire(const Eigen::Vector2f &angular_error); 


// ===================== PredictionWorker ============================

PredictionWorker::PredictionWorker(SharedLatest &shared,
                                   SharedScalars &scalars,
                                   std::atomic<bool> &stop_flag)
    : shared_(shared),
      scalars_(scalars),
      stop_(stop_flag),
      last_pf_ver_(0),
      bullet_speed(15.0f),
      processing_time(0.0f),
      t_gimbal_actuation(0.02f),
      fire_state(false),
      chase_state(false),
      aim_state(false),
      vis_yaw_(0.0f),
      vis_pitch_(0.0f),
      vis_init_(false)
{}

void PredictionWorker::operator()() {
    while (!stop_.load(std::memory_order_relaxed)) {
        uint64_t cur_ver = shared_.pf_ver.load(std::memory_order_relaxed);
        if (cur_ver == last_pf_ver_) {
            sleep_small();
            continue;
        }
        last_pf_ver_ = cur_ver;

        auto pf  = std::atomic_load(&shared_.pf_out);
        auto imu = std::atomic_load(&shared_.imu);
        if (!pf || !imu) {
            continue;
        }
        float measured_speed = scalars_.bullet_speed.load(std::memory_order_relaxed);

        PredictionOut out{};
        compute_prediction(*pf, imu, measured_speed, out);

        std::atomic_store(&shared_.prediction_out, std::make_shared<PredictionOut>(out));
        shared_.prediction_ver.fetch_add(1, std::memory_order_relaxed);
    }
}

void PredictionWorker::sleep_small() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void PredictionWorker::compute_prediction(const RobotState &rs,
                                          const std::shared_ptr<IMUState>& imu,
                                          float measured_speed,
                                          PredictionOut &out)
{
    // Re-used buffers per thread
    static thread_local Eigen::Vector3f world_pos;
    static thread_local Eigen::Vector3f world_pos_lead;
    static thread_local Eigen::Vector3f cam_pos_lead;
    static thread_local Eigen::Vector2f correction;
    static thread_local float imu_yaw, imu_pitch;
    static thread_local float yaw_world, yaw_lead_world;

    const auto &state = rs.state;

    // ----------------- 0) Basic sanity of PF state -----------------
    bool state_valid = true;
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(state[i])) {
            state_valid = false;
            break;
        }
    }

    const float dist_sq =
        state[0] * state[0] +
        state[1] * state[1] +
        state[2] * state[2];

    // More than 100 m away → treat as invalid for now
    if (dist_sq > 100.0f * 100.0f) {
        state_valid = false;
    }

    if (!state_valid) {
        out.yaw   = 0.0f;
        out.pitch = 0.0f;
        out.aim   = 0;
        out.fire  = 0;
        out.chase = 0;
        return;
    }

    // ----------------- 1) Bullet speed filtering -----------------
    float bs = this->bullet_speed;
    filtering(bs, measured_speed, ALPHA_BULLET_SPEED);
    this->bullet_speed = bs;

    if (bs < 1.0f || !std::isfinite(bs)) {
        bs = 15.0f;  // fallback
        this->bullet_speed = bs;
    }

    // ----------------- 2) Processing time estimation --------------
    const auto now = Clock::now();
    const float proc_time =
        std::chrono::duration_cast<std::chrono::duration<float>>(now - rs.timestamp).count();

    float proc = this->processing_time;
    filtering(proc, proc_time, ALPHA_PROCESSING_TIME);
    this->processing_time = proc;

    // ----------------- 3) WORLD frame position / yaw --------------
    world_pos[0] = state[0];
    world_pos[1] = state[1];
    world_pos[2] = state[2];
    yaw_world    = state[9];

    // ----------------- 4) Decide stationary vs moving -------------
    const float vx = state[3];
    const float vy = state[4];
    const float vz = state[5];
    const float speed = std::sqrt(vx*vx + vy*vy + vz*vz);

    constexpr float STATIONARY_SPEED_THRESH = 0.05f; // m/s
    float t_lead = 0.0f;

    if (speed < STATIONARY_SPEED_THRESH) {
        // Treat as static target
        world_pos_lead   = world_pos;
        yaw_lead_world   = yaw_world;
        // Only compensate measurement+gimbal delay (no travel lead)
        t_lead = proc + this->t_gimbal_actuation;
    } else {
        // Full constant-acceleration lead
        world_pos_lead = world_pos;
        yaw_lead_world = yaw_world;

        // Initial guess including bullet travel + delays
        t_lead = t_lead_calculation(world_pos_lead, bs)
               + proc
               + this->t_gimbal_actuation;

        int   iter = 0;
        constexpr int MAX_ITERS = PRED_CONV_MAX_ITERS;
        float t_lead_prev = t_lead;

        do {
            motion_model_robot_pos(state, world_pos_lead, yaw_lead_world, t_lead_prev);

            t_lead = t_lead_calculation(world_pos_lead, bs)
                   + proc
                   + this->t_gimbal_actuation;

            float diff = std::fabs(t_lead - t_lead_prev);
            t_lead_prev = t_lead;
            ++iter;

            if (diff < PREDICTION_CONVERGENCE_THRESHOLD)
                break;

        } while (iter < MAX_ITERS);
    }

    // ----------------- 5) world -> camera using IMU ----------------
    // Eigen::Quaternionf q_WI_local;
    // bool imu_ok;
    // if (!get_imu_quat_world_from_imu(imu, q_WI_local)) {
    //     out.aim = out.fire = out.chase = 0;
    //     out.yaw = out.pitch = 0.0f;
    //     std::cout <<"[PRED ERROR] Invalid IMU pitch yaw!\n";
    //     return;
    // }
    // static const Eigen::Quaternionf q_IC = Eigen::Quaternionf::Identity();;
    // Eigen::Matrix3f R_world2cam = make_R_world2cam_from_quat(q_WI_local, q_IC);

    if (!get_imu_yaw_pitch(shared_, imu_yaw, imu_pitch)) {
        out.aim = out.fire = out.chase = 0;
        out.yaw = out.pitch = 0.0f;
        std::cout <<"[PRED ERROR] Invalid IMU pitch yaw!\n";
        return;
    }
    const Eigen::Matrix3f R_world2cam = make_R_world2cam_from_yaw_pitch(imu_yaw, imu_pitch);
    cam_pos_lead = R_world2cam * world_pos_lead;
    float yaw_cam_lead = yaw_lead_world;

    if (!std::isfinite(cam_pos_lead[0]) ||
        !std::isfinite(cam_pos_lead[1]) ||
        !std::isfinite(cam_pos_lead[2])) {
        // std::cout << "[PRED ERROR] NaN after world2cam transform!\n";
        out.yaw = out.pitch = 0.0f;
        out.aim = out.fire = out.chase = 0;
        return;
    }

    if (cam_pos_lead[2] <= 0.0f) {
        // std::cout << "[PRED ERROR] Robot behind camera! z="
        //           << cam_pos_lead[2] << std::endl;
        out.yaw = out.pitch = 0.0f;
        out.aim = out.fire = out.chase = 0;
        return;
    }

    // ----------------- 6) Offset to armor in CAM frame ------------
    Eigen::Vector3f armor_cam = cam_pos_lead;
    calculate_robot_final_target_point(
        armor_cam, yaw_cam_lead,
        state[14],   // height offset
        state[12],   // r1
        state[13]);  // r2

    // std::cout << "[Pre]: " 
    //     << armor_cam[0] << ", "
    //     << armor_cam[1] << ", "
    //     << armor_cam[2] << std::endl;

    // ----------------- 7) Bullet drop correction ------------------
    const float t_bullet_travel =
        std::max(0.0f, t_lead - this->t_gimbal_actuation - proc);

    const float drop_correction = 0.5f * 9.81f * t_bullet_travel * t_bullet_travel;
    armor_cam[1] += drop_correction;

    // ----------------- 8) Gimbal correction (yaw, pitch) ----------
    calculate_gimbal_correction(armor_cam, correction);

    if (!std::isfinite(correction[0]) || !std::isfinite(correction[1])) {
        // std::cout << "[PRED ERROR] NaN in gimbal correction!\n";
        out.yaw = out.pitch = 0.0f;
        out.aim = out.fire = out.chase = 0;
        return;
    }

    const bool fire_state_raw  = should_fire(correction);
    const bool chase_state_raw = (armor_cam[2] > CHASE_THREASHOLD);
    const bool aim_state_raw   = true; // TODO: hook PF state machine

    fire_state  = fire_state_raw;
    chase_state = chase_state_raw;
    aim_state   = aim_state_raw;

    // ----------------- 9) VISUAL / COMMAND SMOOTHING --------------

    float raw_yaw   = correction[0];
    float raw_pitch = correction[1];

    if (!vis_init_) {
        vis_yaw_   = raw_yaw;
        vis_pitch_ = raw_pitch;
        vis_init_  = true;
    }

    float dy = raw_yaw   - vis_yaw_;
    float dp = raw_pitch - vis_pitch_;

    // handle yaw wrap
    if (dy > M_PI)       dy -= 2.0f * M_PI;
    else if (dy < -M_PI) dy += 2.0f * M_PI;

    const float error_mag      = std::sqrt(dy*dy + dp*dp);
    const float alpha_min      = 0.3f;
    const float alpha_max      = 0.7f;
    const float error_threshold= 0.1f;   // ~5.7 deg

    float alpha = alpha_min +
                  (alpha_max - alpha_min) *
                  std::min(1.0f, error_mag / error_threshold);

    const float max_step = 0.05f;        // ~2.9 deg per update
    dy = std::clamp(dy, -max_step, max_step);
    dp = std::clamp(dp, -max_step, max_step);

    vis_yaw_   += alpha * dy;
    vis_pitch_ += alpha * dp;

    const float deadzone = 0.001f;       // ~0.06 deg
    if (std::fabs(vis_yaw_)   < deadzone) vis_yaw_   = 0.0f;
    if (std::fabs(vis_pitch_) < deadzone) vis_pitch_ = 0.0f;

    clamp_to_gimbal_limits(vis_yaw_, vis_pitch_);

    // target reachability check uses raw correction (hardware limits)
    bool target_reachable = is_target_reachable(raw_yaw, raw_pitch);
    // if (!target_reachable) {
    //     std::cout << "[PRED WARNING] Target out of gimbal range! raw_pitch="
    //               << (raw_pitch * 180.0f / M_PI) << " deg\n";
    //     fire_state = false;
    // }

    if (is_at_pitch_limit(vis_pitch_)) {
        fire_state = false;
        static int limit_warning_counter = 0;
        if (++limit_warning_counter % 30 == 0) {
            char status[128];
            get_gimbal_status_string(vis_yaw_, vis_pitch_, status, sizeof(status));
            std::cout << "[GIMBAL LIMIT] " << status << std::endl;
        }
    }

    // ----------------- 10) Write Outputs --------------------------
    out.yaw   = vis_yaw_;
    out.pitch = vis_pitch_;
    out.aim   = aim_state   ? 1 : 0;
    out.fire  = fire_state  ? 1 : 0;
    out.chase = chase_state ? 1 : 0;
}


// ===================== Helper Implementations =====================

inline void filtering(float &value, float measurement, float alpha) {
    const float one_minus_alpha = 1.0f - alpha;
    value = alpha * measurement + one_minus_alpha * value;
}

inline bool is_converged(float v, float threshold) {
    return std::fabs(v) < threshold;
}

inline float t_lead_calculation(const Eigen::Vector3f &tvec, const float &bullet_speed) {
    const float dx = tvec[0];
    const float dy = tvec[1];
    const float dz = tvec[2];

    const float distance2 = dx*dx + dy*dy + dz*dz;
    const float distance  = std::sqrt(distance2);

    return distance / bullet_speed;      // assume bullet_speed > 0
}

inline int sector_from_yaw(const float yaw) {
    const float theta    = wrap_pi(yaw);
    const float sector_f = (theta + QUARTER_PI) / HALF_PI;
    return static_cast<int>(std::floor(sector_f)) & 3;
}

inline void calculate_robot_final_target_point(Eigen::Vector3f &final_pos, float &final_yaw,
                                               const float height_offset,
                                               const float r1, const float r2)
{
    const int armor_plate_idx = sector_from_yaw(final_yaw);

    constexpr float TWO       = 2.0f;
    constexpr float QUARTER_P = static_cast<float>(M_PI_4); // pi/4
    constexpr float HALF_P    = static_cast<float>(M_PI_2); // pi/2
    
    const float yaw_shifted  = final_yaw + QUARTER_P;
    const float yaw_restrict = std::fmod(yaw_shifted, TWO * HALF_P) - QUARTER_P;

    const float radius = armor_plate_idx ? r2 : r1;
    const float s      = std::sin(yaw_restrict);
    const float c      = std::cos(yaw_restrict);

    // camera frame: +z forward, +x right
    final_pos[0] += radius * s;
    final_pos[2] += radius * c;

    if (armor_plate_idx)
        final_pos[1] += height_offset;
}

inline void motion_model_robot_pos(const std::array<float, ROBOT_STATE_VEC_LEN> &state, 
                                   Eigen::Vector3f &robot_center_lead,
                                   float &yaw_lead, const float &t)
{
    const float t2 = t * t;
    robot_center_lead[0] = state[0] + state[3] * t + 0.5f * state[6] * t2;
    robot_center_lead[1] = state[1] + state[4] * t + 0.5f * state[7] * t2;
    robot_center_lead[2] = state[2] + state[5] * t + 0.5f * state[8] * t2;
    yaw_lead             = state[9] + state[10] * t + state[11] * t2;
}

inline void calculate_gimbal_correction(const Eigen::Vector3f &tvec,
                                        Eigen::Vector2f &correction)
{
    const float x = tvec[0];
    const float y = tvec[1];
    const float z = tvec[2];

    correction[0] = std::atan2(x, z);  // yaw

    const float horizontal_dist = std::sqrt(x*x + z*z);
    correction[1] = std::atan2(y, horizontal_dist); // pitch
}

inline int should_fire(const Eigen::Vector2f &angular_error) {
    constexpr float HALF = 0.5f;
    const float x_tolerance = WIDTH_TOLERANCE  * TOLERANCE_COEFF * HALF;
    const float y_tolerance = HEIGHT_TOLERANCE * TOLERANCE_COEFF * HALF;
    const float ax = std::fabs(angular_error[0]);
    const float ay = std::fabs(angular_error[1]);

    return (ax < x_tolerance) && (ay < y_tolerance);
}



