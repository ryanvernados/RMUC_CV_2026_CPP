#include <atomic>
#include <vector>
#include <cmath>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "types.hpp"
#include "workers.hpp"
#include "helper.hpp"


PredictionWorker::PredictionWorker(SharedLatest &shared,
                    SharedScalars &scalars,
                    std::atomic<bool> &stop_flag)
    : shared_(shared), scalars_(scalars), stop_(stop_flag), last_pf_ver_(0) {}

void PredictionWorker::operator()() {
    while (!stop_.load(std::memory_order_relaxed)) {
        uint64_t cur_ver = shared_.pf_ver.load(std::memory_order_relaxed);
        if (cur_ver == last_pf_ver_) {
            sleep_small();
            continue; // no new PF state
        }
        last_pf_ver_ = cur_ver;

        auto pf = std::atomic_load(&shared_.pf_out);
        auto imu = std::atomic_load(&shared_.imu);
        if (!pf || !imu) {
            continue;
        }

        float measured_speed = scalars_.bullet_speed.load(std::memory_order_relaxed);

        PredictionOut out{};
        compute_prediction(*pf, imu.get(), measured_speed, out);

        auto ptr = std::make_shared<PredictionOut>(out);
        std::atomic_store(&shared_.prediction_out, ptr);
        shared_.prediction_ver.fetch_add(1, std::memory_order_relaxed);
    }
}

void PredictionWorker::sleep_small() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void PredictionWorker::compute_prediction(const RobotState &rs,
                        const IMUState *imu,
                        float measured_speed,
                        PredictionOut &out)
{
    // Reuse small buffers instead of reallocating every call
    static thread_local Eigen:Vector3f tvec;
    static thread_local Eigen:Vector3f pos_lead;
    static thread_local Eigen:Vector2f correction;
    static thread_local float yaw_lead, imu_yaw, imu_pitch;

    // --- bullet_speed filtering ---
    float bullet_speed = this->bullet_speed; // keep in a register
    filtering(bullet_speed, measured_speed, ALPHA_BULLET_SPEED);
    this->bullet_speed = bullet_speed;

    // --- t_processing_time filtering ---
    const auto now = Clock::now();
    const float proc_time =
        std::chrono::duration_cast<std::chrono::duration<float>>(now - rs.timestamp).count();

    float processing_time = this->processing_time;
    filtering(processing_time, proc_time, ALPHA_PROCESSING_TIME);
    this->processing_time = processing_time;

    // --- convergence method for motion model (WORLD frame) ---
    tvec.resize(3);
    tvec[0] = rs.state[0];
    tvec[1] = rs.state[1];
    tvec[2] = rs.state[2];

    float t_lead = t_lead_calculation(tvec, bullet_speed)
                 + processing_time
                 + this->t_gimbal_actuation;

    int iter = 0;
    constexpr int MAX_ITERS = PRED_CONV_MAX_ITERS;

    do {
        //pos_lead = motion_model_robot(rs.state, t_lead);
        // convert yaw from w2c approach, compromise lead time calculation accuracy
        motion_model_robot_pos(rs.state, tvec, yaw_lead, t_lead);
        t_lead = t_lead_calculation(tvec, bullet_speed)
               + processing_time
               + this->t_gimbal_actuation;
        ++iter;
    } while (!is_converged(t_lead, PREDICTION_CONVERGENCE_THRESHOLD) &&
             iter < MAX_ITERS);

    // --- world2cam using imu yaw/pitch ---
    //Eigen::Matrix3f R_world2cam = world2cam(this->shared_);
    //pos_lead = pos_world2cam(pos_lead, R_world2cam);
    //TODO: convert yaw from world to cam for better yaw accuracy
    bool success = get_imu_yaw_pitch(this->shared_, imu_yaw, imu_pitch);
    //if success: ...
    Eigen::Matrix3f R_world2cam = make_R_world2cam_from_yaw_pitch(imu_yaw, imu_pitch);
    tvec = R_world2cam * tvec;
    yaw_lead = yaw_lead - imu_yaw;    // + initial_yaw?
    calculate_robot_final_target_point(pos_lead, yaw_lead, state[14], state[12], state[13]);

    // --- bullet drop correction ---
    const float t_bullet_travel = (t_lead - this->t_gimbal_actuation - processing_time);
    const float drop_correction = 0.5f * 9.81f * t_bullet_travel * t_bullet_travel;
    pos_lead[1] += drop_correction;

    correction = calculate_gimbal_correction(pos_lead);
    const bool fire_state  = should_fire(pos_lead);
    
    const bool chase_state = (pos_lead[2] > CHASE_THREASHOLD);
    const bool aim_state   = true;  // TODO: hook to PF state machine

    this->fire_state  = fire_state;
    this->chase_state = chase_state;
    this->aim_state   = aim_state;
}

inline void filtering(float &value, float measurement, float alpha) {
    const float one_minus_alpha = 1.0f - alpha;
    value = alpha * measurement + one_minus_alpha * value;
}

inline bool is_converged(float v, float threshold) {
    return std::fabs(v) < threshold;
}

inline float t_lead_calculation(const Eigen::Vector3f> &tvec, const float &bullet_speed) {
    const float dx = tvec[0];
    const float dy = tvec[1];
    const float dz = tvec[2];

    const float distance2 = dx * dx + dy * dy + dz * dz;
    const float distance  = std::sqrt(distance2);

    // Caller should ensure bullet_speed > 0
    return distance / bullet_speed;
}

inline int sector_from_yaw(const float yaw) {
    // Precompute constants once per TU, not per call
    constexpr float HALF_PI    = static_cast<float>(M_PI_2);
    constexpr float QUARTER_PI = static_cast<float>(M_PI_4);
    const float theta = wrap_pi(yaw);
    const float sector_f = (theta + QUARTER_PI) / HALF_PI;
    return static_cast<int>(std::floor(sector_f)) & 3;
}

inline void calculate_robot_final_target_point(Eigen::Vector3f &final_pos, float &final_yaw,
                                        const float height_offset, const float r1, const float r2) {
    // yaw with yaw_rate and yaw_acc
    const int armor_plate_idx = sector_from_yaw(final_yaw);
    constexpr float TWO = 2.0f;
    constexpr float QUARTER_PI = static_cast<float>(M_PI_4); // pi/4
    constexpr float HALF_PI    = static_cast<float>(M_PI_2); // pi/2
    const float yaw_shifted  = final_yaw + QUARTER_PI;
    const float yaw_restrict = std::fmod(yaw_shifted, TWO * HALF_PI) - QUARTER_PI;
    const float radius = armor_plate_idx ? r2 : r1;
    const float s = std::sin(yaw_restrict);
    const float c = std::cos(yaw_restrict);
    final_pos[0] += radius * s;
    final_pos[2] -= radius * c;
    // height offset
    final_pos[1] += armor_plate_idx ? height_offset : 0;
}

inline void motion_model_robot_pos(const Eigen::Vector3f &state, Eigen::Vector3f &robot_center_lead, float &yaw_lead, const float &t) {
    const float t2 = t * t;
    robot_center_lead[0] = state[0] + state[3] * t + 0.5 * state[6] * t2;
    robot_center_lead[1] = state[1] + state[4] * t + 0.5 * state[7] * t2;
    robot_center_lead[2] = state[2] + state[5] * t + 0.5 * state[8] * t2;
    yaw_lead = state[9] + state[10] * t + state[11] * t2;
}

inline void calculate_gimbal_correction(const Eigen::Vector3f &tvec, Eigen::Vector2f &correction) {
    const float x = tvec[0];
    const float y = tvec[1];
    const float z = tvec[2];
    correction[0] = std::atan2(x, z);
    correction[1] = std::atan2(y, z);

    return correction;
}

inline int should_fire(const Eigen::Vector3f &tvec) {
    // Assume tvec.size() >= 2 (tvec[0]=x, tvec[1]=y in angle or pixel units)
    constexpr float HALF = 0.5f;
    const float x_tolerance = WIDTH_TOLERANCE  * TOLERANCE_COEFF * HALF;
    const float y_tolerance = HEIGHT_TOLERANCE * TOLERANCE_COEFF * HALF;
    const float ax = std::fabs(tvec[0]);
    const float ay = std::fabs(tvec[1]);

    return (ax < x_tolerance) && (ay < y_tolerance);
}






