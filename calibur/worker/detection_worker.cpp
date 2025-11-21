#include <cmath>
#include <vector>
#include <stdlib.h>
#include <atomic>
#include <thread>

#include "workers.hpp"
#include "types.hpp"
#include "helper.hpp"

using RobotStatePtr = std::shared_ptr<const RobotState>;

DetectionWorker::DetectionWorker(SharedLatest &shared,
                std::atomic<bool> &stop_flag)
    : shared_(shared), stop_(stop_flag)
{
    start_time_        = Clock::now();
    selected_robot_id_ = -1;
    ttl_               = 0.0f;
    initial_yaw_       = 0.0f;
    last_cam_ver_      = 0;
}

void DetectionWorker::operator()() {

    static thread_local float imu_yaw, imu_pitch;
    static thread_local RobotState local_robot_state;   //keep track of what robot is being tracked
    static thread_local std::shared_ptr<IMUState> imu;
    static thread_local std::vector<DetectionResult> selected_armors;
    static thread_local std::vector<DetectionResult> dets;
    static thread_local std::vector<std::vector<DetectionResult>> grouped_armors;
    //static thread_local RobotState robot;


    while (!stop_.load(std::memory_order_relaxed)) {


        uint64_t cur_ver = shared_.camera_ver.load(std::memory_order_relaxed);
        if (cur_ver == last_cam_ver_) {
            sleep_small();
            continue; // no new camera frame
        }
        last_cam_ver_ = cur_ver;

        auto cam = std::atomic_load(&shared_.camera);
        if (!cam) {
            sleep_small();
            continue;
        }

        // snapshot IMU (ref only)
        imu = std::atomic_load(&shared_.imu);

        // 1) YOLO
        yolo_predict(cam->raw_data, cam->width, cam->height, dets);

        // 2) keypoint refine + filtering by confidence
        refine_keypoints(dets, cam->width, cam->height);

        // 3) solvePnP + yaw in cam frame
        solvepnp_and_yaw(dets);

        // 4) group + select
        group_armors(dets, grouped_armors);
        select_armor(grouped_armors, ttl_, selected_robot_id_, initial_yaw_, selected_armors);

        // 5) transform to world using latest IMU
        //TODO: considering whether to do pnp first or select robot first
        bool success = get_imu_yaw_pitch(this->shared_, imu_yaw, imu_pitch);
        //if success: ...
        const Eigen::Matrix3f R_world2cam = make_R_cam2world_from_yaw_pitch(imu_yaw, imu_pitch);
        for (auto &det : selected_armors) {
            cam2world(det, R_world2cam, imu_yaw, imu_pitch);
        }

        // 6) form RobotState
        auto robot = form_robot(selected_armors);
        if (robot) {
            robot->timestamp = cam->timestamp;
            auto ptr = std::make_shared<RobotState>(*robot);
            std::atomic_store(&shared_.detection_out, ptr);
            shared_.detection_ver.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void DetectionWorker::sleep_small() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

std::vector<DetectionResult>
DetectionWorker::yolo_predict(const std::vector<uint8_t> &raw, int w, int h, std::vector<DetectionResult> &dets) {
    return {};
}

void DetectionWorker::refine_keypoints(std::vector<DetectionResult> &dets, int w, int h) {
    // trad CV refine
}

void DetectionWorker::solvepnp_and_yaw(std::vector<DetectionResult> &dets) {
    // solvePnP, Rodrigues, decompose, yaw_rad

    //need to convert cv::Vec3f to Eigen::Vector3f here

}


void DetectionWorker::group_armors(const std::vector<DetectionResult> &dets,
                                    std::vector<std::vector<DetectionResult>> &grouped_armors) {

}

void DetectionWorker::select_armor(const std::vector<std::vector<DetectionResult>> &grouped_armors,
                 float &ttl,
                 int &selected_robot_id,
                 float &initial_yaw,
                 std::vector<DetectionResult> &selected_armors) {

    constexpr float MAX_TTL = SELECTOR_TTL;
    float dt = 0.02f;                  // or compute real dt

    // NO DETECTIONS
    if (grouped_armors.empty()) {
        ttl -= dt;
        if (ttl <= 0) {
            selected_robot_id = -1;
            selected_armors.clear();
        }
        return;
    }

    // NO TARGET CURRENTLY SELECTED
    if (selected_robot_id & 0x80000000) {   // id < 0
        selected_robot_id = choose_best_robot(grouped_armors);
        selected_armors = grouped_armors[selected_robot_id];
        ttl = MAX_TTL;
        return;
    }

    // CHECK IF CURRENTLY TRACKED ROBOT IS SEEN
    int tracked_idx = -1;
    for (int i = 0; i < grouped_armors.size(); i++) {
        if (grouped_armors[i][0].class_id == selected_robot_id) {
            tracked_idx = i;
            break;
        }
    }

    if (tracked_idx != -1) {
        selected_armors = grouped_armors[tracked_idx];
        ttl = MAX_TTL;
        return;            // continue tracking same robot
    }

    // TRACKED ROBOT MISSING → GRACE PERIOD
    ttl -= dt;
    if (ttl > 0) {
        selected_armors.clear();   // missing 1–N frames, but tracking continues
        return;
    }

    // FULL LOST → SWITCH TARGET
    selected_robot_id = choose_best_robot(grouped_armors);
    selected_armors = grouped_armors[selected_robot_id];
    ttl = MAX_TTL;

}

std::unique_ptr<RobotState>
DetectionWorker::form_robot(const std::vector<DetectionResult> &armors) {
    if (armors.empty() || armors.size() > 2) {
        if (!this->has_prev_robot_) {
            // No previous state yet → nothing meaningful to return
            return nullptr;
        }
        // Return a copy of the previous robot state
        return std::make_unique<RobotState>(prev_robot_);
    }
    auto rs = std::make_unique<RobotState>();
    bool form_robot_success
    if (armors.size() == 1) {
        form_robot_success = from_one_armor(armors[0], prev_robot_, this->has_prev_robot_);
    } else {
        form_robot_success = from_two_armors(armors[0], armors[1], prev_robot_, this->has_prev_robot_);
    }

    return form_robot_success ? rs : std::make_unique<RobotState>(prev_robot_);
}

inline void cam2world(DetectionResult &det, const Eigen::Matrix3f &R_world2cam,
                     const float imu_yaw, const float imu_pitch) {
    det.tvec = R_world2cam * det.tvec;
    det.yaw_rad += imu_yaw;
}

// Compute Euclidean distance quickly
inline float tvec_distance(const Eigen::Vector3f& t) {
    return std::sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
}

// Choose robot with minimum average distance of its armors
int choose_best_robot(const std::vector<std::vector<DetectionResult>>& grouped_armors)
{
    int best_idx = 0;
    float best_dist = std::numeric_limits<float>::max();

    for (int i = 0; i < grouped_armors.size(); i++) {
        const auto& g = grouped_armors[i];

        float avg_dist;
        if (g.size() == 1) {
            avg_dist = tvec_distance(g[0].tvec);
        } else {
            // two armors → average the distance
            float d1 = tvec_distance(g[0].tvec);
            float d2 = tvec_distance(g[1].tvec);
            avg_dist = 0.5f * (d1 + d2);
        }

        if (avg_dist < best_dist) {
            best_dist = avg_dist;
            best_idx = i;
        }
    }

    return best_idx;
}

inline int sector_yaw(const float yaw_meas, const float prev_yaw) {
    const float yaw_diff = wrap_pi(prev_yaw - yaw_meas);
    return std::round(yaw_diff / M_PI_2);
}

inline bool from_one_armor(const DetectionResult &det, RobotState &robot, bool &valid) {
    const float yaw_meas = det.yaw_rad;
    const float c = std::cos(yaw_meas);
    const float s = std::sin(yaw_meas);
    if (!valid) {
        robot.state[IDX_R1]   = DEFAULT_ROBOT_RADIUS;
        robot.state[IDX_R2]   = DEFAULT_ROBOT_RADIUS;
        robot.state[IDX_YAW]  = det.yaw_rad;

        robot.state[IDX_TX] = det.tvec[0] - robot.state[IDX_R1] * s;
        robot.state[IDX_TY] = det.tvec[1];
        robot.state[IDX_TZ] = det.tvec[2] + robot.state[IDX_R1] * c;
        valid = true;
    } else {        
        //need to assume radius and height is the same as previous
        const float prev_yaw = robot.state[IDX_YAW];
        const sector = sector_yaw(yaw_meas, prev_yaw);
        robot.state[IDX_YAW] = wrap_pi(yaw_meas + M_PI_2 * sector);
        const float r = sector % 2 ? robot.state[IDX_R2] : robot.state[IDX_R1];
        const float h = sector % 2 ? robot.state[IDX_H] : 0;
        
        robot.state[IDX_TX] = det.tvec[0] - r * s;
        robot.state[IDX_TY] = det.tvec[1] + h;
        robot.state[IDX_TZ] = det.tvec[2] + r * c;
    }
    robot.class_id = det.class_id;
    return true;
}

inline bool correct_yaw_to_90(float &yaw1, float &yaw2) {
    const float mid_point = (yaw1 + yaw2) / 2;
    yaw1 = wrap_pi(mid_point - M_PI_4);
    yaw2 = wrap_pi(mid_point + M_PI_4);
}

inline void solve_linear_sys(
    float yaw1, float yaw2,
    float det1_x, float det1_z,
    float det2_x, float det2_z,
    float &x, float &z, float &r1, float &r2)
{
    Eigen::Matrix4f A;
    Eigen::FullPivLU<Eigen::Matrix4f> lu(A);
    if (lu.rank() < 4) {
        return false;
    }
    A << 1, 0, std::sin(yaw1), 0,
         0, 1, -std::cos(yaw1), 0,
         1, 0, 0, std::cos(yaw2),
         0, 1, 0, -std::sin(yaw2);

    Eigen::Vector4f b;
    b << det1_x, det1_z, det2_x, det2_z;
    Eigen::Vector4f solution = A.fullPivLu().solve(b);
    x  = solution(0);
    z  = solution(1);
    r1 = solution(2);
    r2 = solution(3);

    return true;
}

inline bool from_two_armors(const DetectionResult &det1, const DetectionResult &det2,
                           RobotState &robot, bool &valid) {
    const float yaw1 = det1.yaw_rad;
    const float yaw2 = det2.yaw_rad;
    coorect_yaw_to_90(yaw1, yaw2);
    if (!valid) {
        //define the robot yaw to be det1's yaw, TODO: make sure that det1 yaw < det 2 yaw and they are in -90 < yaw < 90 deg
        robot.state[IDX_YAW]  = yaw1;
        robot.state[IDX_H]    = det1.tvec[1] - det2.tvec[1];
        robot.state[IDX_TY]   = det1.tvec[1];
        bool solve_lin_sys_success = solve_linear_sys(yaw1, yaw2, det1.tvec[0], det1.tvec[2], det2.tvec[0], det2.tvec[2],
                                            robot.state[IDX_TX], robot.state[IDX_TZ], robot.state[IDX_R1], robot.state[IDX_R2]);
        valid = solve_lin_sys_success;
    } else {
        const float prev_yaw = robot.state[IDX_YAW];
        const int sector_armor_1 = sector_yaw(yaw1, prev_yaw);
        const int sector_armor_2 = sector_yaw(yaw2, prev_yaw);
//check correctness -> a way to calculate true yaw based on the 2 measured yaw
        float y1 = wrap_pi(yaw1 + sector_armor1 * M_PI_2);
        float y2 = wrap_pi(yaw2 + sector_armor2 * M_PI_2);
        float mean_yaw = std::atan2(std::sin(y1) + std::sin(y2), std::cos(y1) + std::cos(y2));
        robot.state[IDX_yaw] = wrap_pi(mean_yaw);
        
        robot.state[IDX_H]   = (sector_armor_1 % 2) ? (det2.tvec[1] - det1.tvec[1]) : (det1.tvec[1] - det2.tvec[1]);
        robot.state[IDX_TY]  = (sector_armor_1 % 2) ? det2.tvec[1] : det1.tvec[1];
        bool solve_lin_sys_success = solve_linear_sys(yaw1, yaw2, det1.tvec[0], det1.tvec[2], det2.tvec[0], det2.tvec[2],
                                            robot.state[IDX_TX], robot.state[IDX_TZ], robot.state[IDX_R1], robot.state[IDX_R2]);
    }
    return solve_lin_sys_success; 
}
