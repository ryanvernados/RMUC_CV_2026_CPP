#ifndef HELPER_HPP
#define HELPER_HPP

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include "types.hpp"

using namespace std;

constexpr float PI     = static_cast<float>(M_PI);
constexpr float HALF_PI = 0.5f * PI;
constexpr float QUARTER_PI = 0.25f * PI;
constexpr float TWO_PI  = 2.0f * PI;

constexpr float GIMBAL_PITCH_MIN = -0.30f;  // ~-10° (looking down) - ADJUST THIS
constexpr float GIMBAL_PITCH_MAX =  0.87f;  // ~+50° (looking up)   

// Yaw limits - Full 360° rotation (no limits)
constexpr float GIMBAL_YAW_MIN = -3.14f;    // -180°
constexpr float GIMBAL_YAW_MAX =  3.14f;    // +180°

// Software safety margin (stay away from hard limits)
constexpr float GIMBAL_SAFETY_MARGIN = 0.05f;  // ~3 degrees

// For 360° gimbal, yaw wrapping is more important than limits
constexpr bool GIMBAL_HAS_YAW_LIMITS = false;  // Set to true if yaw is limited

inline float wrap_pi(const float angle) {
    return std::fmod(angle + M_PI, 2.0f * M_PI) - M_PI;
}

inline float deg2rad(float deg) {
    return deg * (PI / 180.0f);
}
static inline Eigen::Vector3d project_to_plane(const Eigen::Vector3d& v,
                                               const Eigen::Vector3d& up_unit) {
    return v - v.dot(up_unit) * up_unit;
}

static inline double signed_angle_about_up(const Eigen::Vector3d& a_in,
                                           const Eigen::Vector3d& b_in,
                                           const Eigen::Vector3d& up_unit) {
    const Eigen::Vector3d a = a_in.normalized();
    const Eigen::Vector3d b = b_in.normalized();
    const double s = up_unit.dot(a.cross(b));
    const double c = a.dot(b);
    return std::atan2(s, c);
}


static inline Eigen::Quaternionf normalize_quat(Eigen::Quaternionf q) {
    q.normalize();
    if (q.w() < 0.0f) q.coeffs() *= -1.0f;
    return q;
}

static inline bool get_imu_quat_world_from_imu(const std::shared_ptr<IMUState>& imu,
                                              Eigen::Quaternionf& q_WI_out) {
    if (!imu) return false;

    // IMU provides (w, x, y, z)
    Eigen::Quaternionf q_raw(imu->quaternion[0],
                             imu->quaternion[1],
                             imu->quaternion[2],
                             imu->quaternion[3]);
    q_raw = normalize_quat(q_raw);
    static const Eigen::Quaternionf q_Wfix = []{
        Eigen::Matrix3f T;
        T <<  1,  0,  0,
              0,  1,  0,
              0,  0,  1;
        return Eigen::Quaternionf(T);
    }();

    // World-frame change => LEFT multiply
    q_WI_out = normalize_quat(q_Wfix * q_raw);
    return true;
}


static inline Eigen::Matrix3f make_R_cam2world_from_quat(
    const Eigen::Quaternionf& q_WI,
    const Eigen::Quaternionf& q_IC
) {
    Eigen::Quaternionf q_WC = q_WI * q_IC;
    return q_WC.toRotationMatrix();
}

static inline Eigen::Matrix3f make_R_world2cam_from_quat(
    const Eigen::Quaternionf& q_WI,
    const Eigen::Quaternionf& q_IC
) {
    Eigen::Quaternionf q_WC = q_WI * q_IC;
    return q_WC.conjugate().toRotationMatrix();
}

inline bool get_imu_yaw_pitch(const SharedLatest &shared,
                              float &yaw_left_pos,
                              float &pitch_down_pos) {
    std::shared_ptr<IMUState> imu_ptr = std::atomic_load(&shared.imu);
    if (!imu_ptr) return false;

    const IMUState &imu = *imu_ptr;
    if (imu.euler_angle.size() < 3) return false;

    float pitch_deg = imu.euler_angle[1];
    float yaw_deg   = imu.euler_angle[2]; 
    pitch_down_pos = -deg2rad(pitch_deg);
    yaw_left_pos   = -deg2rad(yaw_deg);

    return true;
}


inline Eigen::Matrix3f make_R_cam2world_from_yaw_pitch(float yaw_left_pos,
                                                       float pitch_down_pos)
{
    const float cy = std::cos(-yaw_left_pos);
    const float sy = std::sin(-yaw_left_pos);
    const float cp = std::cos(-pitch_down_pos);
    const float sp = std::sin(-pitch_down_pos);

    Eigen::Matrix3f R;

    R <<  cy,        sy * sp,    -sy * cp,
          0.f,       cp,         -sp,
          sy,       -cy * sp,     cy * cp;

    return R;
}


inline Eigen::Matrix3f make_R_world2cam_from_yaw_pitch(float yaw_left_pos,
                                                       float pitch_down_pos) {
    Eigen::Matrix3f R_cam2world = make_R_cam2world_from_yaw_pitch(yaw_left_pos, pitch_down_pos);
    return R_cam2world.transpose();
}


inline void clamp_to_gimbal_limits(float &yaw, float &pitch) {
    // Clamp pitch with safety margin
    const float pitch_min_safe = GIMBAL_PITCH_MIN + GIMBAL_SAFETY_MARGIN;
    const float pitch_max_safe = GIMBAL_PITCH_MAX - GIMBAL_SAFETY_MARGIN;
    pitch = std::clamp(pitch, pitch_min_safe, pitch_max_safe);
    
    // For 360° gimbal, wrap yaw instead of clamping
    if (!GIMBAL_HAS_YAW_LIMITS) {
        // Wrap yaw to [-π, π]
        yaw = wrap_pi(yaw);
    } else {
        // Clamp yaw if there are physical limits
        const float yaw_min_safe = GIMBAL_YAW_MIN + GIMBAL_SAFETY_MARGIN;
        const float yaw_max_safe = GIMBAL_YAW_MAX - GIMBAL_SAFETY_MARGIN;
        yaw = std::clamp(yaw, yaw_min_safe, yaw_max_safe);
    }
}

inline bool is_at_pitch_limit(float pitch) {
    const float tolerance = 0.08f;  // ~4.5 degrees from limit
    
    return (pitch < GIMBAL_PITCH_MIN + tolerance) || 
           (pitch > GIMBAL_PITCH_MAX - tolerance);
}

inline bool is_target_reachable(float yaw, float pitch) {
    // Yaw is always reachable for 360° gimbal
    // Only check pitch
    return (pitch >= GIMBAL_PITCH_MIN && pitch <= GIMBAL_PITCH_MAX);
}

inline void get_gimbal_status_string(float yaw, float pitch, char* buffer, size_t bufsize) {
    const char* pitch_status = "";
    if (pitch < GIMBAL_PITCH_MIN + 0.08f) {
        pitch_status = " [AT MIN]";
    } else if (pitch > GIMBAL_PITCH_MAX - 0.08f) {
        pitch_status = " [AT MAX]";
    }
    
    snprintf(buffer, bufsize, "yaw=%.2f° pitch=%.2f°%s", 
             yaw * 180.0f / M_PI, 
             pitch * 180.0f / M_PI,
             pitch_status);
}


// inline Eigen::Matrix3f make_R_cam2world_from_yaw_pitch(float yaw_cam_world,
//                                                         float pitch_cam_world)
// {
//     Eigen::Matrix3f Rpitch;
//     Rpitch << 1.0f,                  0.0f,                   0.0f,
//               0.0f,  std::cos(pitch_cam_world), -std::sin(pitch_cam_world),
//               0.0f,  std::sin(pitch_cam_world),  std::cos(pitch_cam_world);
    
//     Eigen::Matrix3f Ryaw;
//     Ryaw <<  std::cos(yaw_cam_world), 0.0f, std::sin(yaw_cam_world),
//              0.0f,                    1.0f, 0.0f,
//             -std::sin(yaw_cam_world), 0.0f, std::cos(yaw_cam_world);
    
//     Eigen::Matrix3f R_cam2world = Ryaw * Rpitch;
    
//     return R_cam2world;
// }

#endif
