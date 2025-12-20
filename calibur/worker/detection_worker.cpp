#include "workers.hpp"
#include "helper.hpp"
#include <atomic>
#include <vector>
#include <memory>
#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>


inline float tvec_distance(const Eigen::Vector3f& t);
static std::array<cv::Point2f, 4>
order_quad_clockwise(const std::vector<cv::Point2f> &pts);
inline static void get_object_points(int armor_type, std::vector<cv::Point3f> &obj_pts);
inline static float wrap_deg(float a);
inline static float angle_diff_deg(float a, float b);
inline int choose_best_robot(const std::vector<std::vector<DetectionResult>>& grouped_armors);
inline int sector_yaw(const float yaw_meas, const float prev_yaw);
bool from_one_armor(const DetectionResult &det, RobotState &robot, bool &valid);
inline bool from_two_armors(const DetectionResult &det1, const DetectionResult &det2,
                           RobotState &robot, bool &valid);

// ===================== DetectionWorker ============================

DetectionWorker::DetectionWorker(SharedLatest &shared,
                                 SharedScalars &shared_scalars,
                                 std::atomic<bool> &stop_flag)
    : shared_(shared),
      scalars_(shared_scalars),
      stop_(stop_flag),
      start_time_(Clock::now()),
      selected_robot_id_(-1),
      ttl_(0.0f),
      initial_yaw_(0.0f),
      last_cam_ver_(0)
{
    camera_matrix = (cv::Mat_<double>(3, 3) <<
        1220.3055,    0.0,          694.5965,
        0.0,          1217.5784,    575.5400,
        0.0,          0.0,          1.0
    );
    //dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    dist_coeffs = (cv::Mat_<double>(1, 5) <<
        -0.0482,
        -0.0967,
         0.0002,
         0.0081,
         0.7152
    );
}

void DetectionWorker::sleep_small() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void DetectionWorker::operator()() {

    static thread_local float imu_yaw = 0.0f;
    static thread_local float imu_pitch = 0.0f;


    {   //init yaw storage
        std::shared_ptr<IMUState> imu_ptr = std::atomic_load(&shared_.imu);
        if (imu_ptr) {
            const float yaw_deg = imu_ptr->euler_angle[2];
            std::atomic_store(&scalars_.initial_yaw, yaw_deg);
        } else {
            // fallback: zero heading if IMU not ready yet
            std::atomic_store(&scalars_.initial_yaw, 0.0f);
        }
    }

    while (!stop_.load(std::memory_order_relaxed)) {
        // Check for new YOLO output
        uint64_t cur_yolo_ver = shared_.yolo_ver.load(std::memory_order_relaxed);
        if (cur_yolo_ver == last_cam_ver_) {
            sleep_small();
            continue;
        }
        last_cam_ver_ = cur_yolo_ver;

        // Get camera and YOLO data
        auto cam_ptr = std::atomic_load(&shared_.camera);
        auto yolo_ptr = std::atomic_load(&shared_.yolo);
        auto imu = std::atomic_load(&shared_.imu);
        
        if (!cam_ptr || !yolo_ptr || !imu) {
            sleep_small();
            continue;
        }

        // Eigen::Quaternionf q_WI_local;
        // if (!get_imu_quat_world_from_imu(imu, q_WI_local)) {
        //     sleep_small();
        //     continue;
        // }

        // Make a copy of YOLO detections
        auto raw_dets = yolo_ptr->dets;
        
        // 1. Refine keypoints
        // std::cout << "Refining keypoints" << std::endl;
        // std::vector<DetectionResult> refined_results = refine_keypoints(cam_ptr, raw_dets);
        
        // 2. Store refined detections for visualization
        // if (!refined_results.empty()) {
        //     shared_.refined_dets = std::make_shared<std::vector<DetectionResult>>(refined_results);
        // } else {
        //     // Clear refined detections if empty
        //     shared_.refined_dets = std::make_shared<std::vector<DetectionResult>>();
        // }
        
        // 3. Solve PnP and compute yaw for refined detections
        // static const Eigen::Quaternionf q_IC = Eigen::Quaternionf::Identity();
        // const Eigen::Matrix3f R_cam2world = make_R_cam2world_from_quat(q_WI_local, q_IC);
        bool imu_ok = get_imu_yaw_pitch(shared_, scalars_, imu_yaw, imu_pitch);
        if (!imu_ok) {
            sleep_small();
            continue;
        }
        std::cout << "[IMU] Yaw: " << imu_yaw * 180 / M_PI << ", Pitch: " << imu_pitch * 180 / M_PI << std::endl;
        const Eigen::Matrix3f R_cam2world = make_R_cam2world_from_yaw_pitch(imu_yaw, imu_pitch);
        std::cout << "right   " << R_cam2world.col(0).transpose() << "\n";
        std::cout << "up      " << R_cam2world.col(1).transpose() << "\n";
        std::cout << "forward " << R_cam2world.col(2).transpose() << "\n";

        for (auto &det : raw_dets) {
            solvepnp_and_yaw(det, R_cam2world); // modifies det in-place (tvec/yaw now in WORLD)
        }
        
        // 4. Group armors by robot
        std::vector<std::vector<DetectionResult>> grouped_armors;
        group_armors(raw_dets, grouped_armors);
        // 5. Select best armor set
        std::vector<DetectionResult> selected_armors;
        select_armor(grouped_armors, selected_armors);

        for (const auto& armor : selected_armors) {
	        std::cout << "[PNP_After_R]: x=" << armor.tvec[0]
                             << " y="     << armor.tvec[1]
                             << " z="     << armor.tvec[2] <<
                         " yaw_rad: " << armor.yaw_rad
                         << std::endl;
        }

        auto robot = form_robot(selected_armors);
        if (robot) {
            robot->timestamp = yolo_ptr->timestamp;

	        std::cout << "[DET] x=" << robot->state[IDX_TX]
                << " y="     << robot->state[IDX_TY]
                << " z="     << robot->state[IDX_TZ]
                << " yaw="   << robot->state[IDX_YAW]
                << " h="    << robot->state[IDX_H]
                << " r1="   << robot->state[IDX_R1]
                << " r2="   << robot->state[IDX_R2]
                << std::endl;
            auto robot_ptr = std::make_shared<RobotState>(*robot);
            std::atomic_store(&shared_.detection_out, robot_ptr);
            shared_.detection_ver.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

std::vector<DetectionResult> DetectionWorker::refine_keypoints(
    std::shared_ptr<CameraFrame> camera_frame,
    std::vector<DetectionResult> &dets)
{
    std::vector<DetectionResult> refined_dets;
    
    if (!camera_frame || dets.empty()) {
        return refined_dets;
    }
    
    cv::Mat img_gray;
    if (camera_frame->raw_data.channels() == 3) {
        cv::cvtColor(camera_frame->raw_data, img_gray, cv::COLOR_BGR2GRAY);
    } else {
        img_gray = camera_frame->raw_data.clone();
    }
    
    for (auto &det : dets) {
        DetectionResult refined = det;
        
        // Skip if no keypoints
        if (det.keypoints.empty()) {
            refined_dets.push_back(refined);
            continue;
        }
        
        // Apply subpixel refinement to keypoints
        for (size_t i = 0; i < refined.keypoints.size(); i++) {
            if (refined.keypoints[i].x >= 0 && refined.keypoints[i].y >= 0 &&
                refined.keypoints[i].x < img_gray.cols && refined.keypoints[i].y < img_gray.rows) {
                
                cv::Point2f kp = refined.keypoints[i];
                cv::Size winSize(5, 5);
                cv::Size zeroZone(-1, -1);
                cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
                
                std::vector<cv::Point2f> points = {kp};
                cv::cornerSubPix(img_gray, points, winSize, zeroZone, criteria);
                
                refined.keypoints[i] = points[0];
            }
        }
        
        // Optionally update bbox based on refined keypoints
        if (refined.keypoints.size() >= 4) {
            float min_x = refined.keypoints[0].x;
            float min_y = refined.keypoints[0].y;
            float max_x = refined.keypoints[0].x;
            float max_y = refined.keypoints[0].y;
            
            for (const auto &kp : refined.keypoints) {
                min_x = std::min(min_x, kp.x);
                min_y = std::min(min_y, kp.y);
                max_x = std::max(max_x, kp.x);
                max_y = std::max(max_y, kp.y);
            }
            
            refined.bbox.x = static_cast<int>(min_x);
            refined.bbox.y = static_cast<int>(min_y);
            refined.bbox.width = static_cast<int>(max_x - min_x);
            refined.bbox.height = static_cast<int>(max_y - min_y);
        }
        
        refined_dets.push_back(refined);
    }
    
    return refined_dets;
}

static inline float wrap_pi_f(float a) {
    while (a <= -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    while (a >   static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    return a;
}

bool DetectionWorker::solvepnp_and_yaw(
    DetectionResult &det,
    const Eigen::Matrix3f &R_cam2world)
{
    if (det.keypoints.size() != 4) {
        det.yaw_rad = 0.0f;
        return false;
    }

    auto img_pts_arr = order_quad_clockwise(det.keypoints);
    std::vector<cv::Point2f> img_pts(img_pts_arr.begin(), img_pts_arr.end());

    std::vector<cv::Point3f> obj_pts;
    get_object_points(det.armor_type, obj_pts);
    if (obj_pts.size() != 4) {
        det.yaw_rad = 0.0f;
        return false;
    }

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(
        obj_pts, img_pts,
        camera_matrix, dist_coeffs,
        rvec, tvec,
        false,
        cv::SOLVEPNP_IPPE);

    if (!ok) {
        det.yaw_rad = 0.0f;
        return false;
    }

    det.rvec[0] = static_cast<float>(rvec.at<double>(0));
    det.rvec[1] = static_cast<float>(rvec.at<double>(1));
    det.rvec[2] = static_cast<float>(rvec.at<double>(2));
    const Eigen::Matrix3f F = (Eigen::Matrix3f() <<
        1,  0, 0,
        0, -1, 0,
        0,  0, 1).finished();

    // ---------------- TRANSLATION ----------------
    Eigen::Vector3f t_cam_cv(
        static_cast<float>(tvec.at<double>(0)),
        static_cast<float>(tvec.at<double>(1)),
        static_cast<float>(tvec.at<double>(2))
    );

    Eigen::Vector3f t_cam = F * t_cam_cv;
    det.tvec = R_cam2world * t_cam;

    // ---------------- ROTATION ----------------
    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);

    Eigen::Matrix3f R_armor2cam_cv;
    R_armor2cam_cv <<
        static_cast<float>(Rcv.at<double>(0,0)), static_cast<float>(Rcv.at<double>(0,1)), static_cast<float>(Rcv.at<double>(0,2)),
        static_cast<float>(Rcv.at<double>(1,0)), static_cast<float>(Rcv.at<double>(1,1)), static_cast<float>(Rcv.at<double>(1,2)),
        static_cast<float>(Rcv.at<double>(2,0)), static_cast<float>(Rcv.at<double>(2,1)), static_cast<float>(Rcv.at<double>(2,2));

    Eigen::Matrix3f R_armor2cam = F * R_armor2cam_cv * F;
    Eigen::Matrix3f R_armor2world = R_cam2world * R_armor2cam;
    // Eigen::Matrix3f R_armor2world = F * R_armor2cam_cv * F;
    Eigen::Vector3f f_world = R_armor2world.col(2);
    float yaw = std::atan2(f_world.x(), f_world.z());
    yaw = wrap_pi_f(yaw); 

    if (yaw >  static_cast<float>(M_PI_2)) yaw -= static_cast<float>(M_PI);
    if (yaw < -static_cast<float>(M_PI_2)) yaw += static_cast<float>(M_PI);

    det.yaw_rad = yaw;
    return true;
}


void DetectionWorker::group_armors(const std::vector<DetectionResult> &dets,
                                   std::vector<std::vector<DetectionResult>> &grouped) {
    if (dets.empty()) return;
    
    std::vector<bool> assigned(dets.size(), false);
    
    for (size_t i = 0; i < dets.size(); i++) {
        if (assigned[i]) continue;
        
        std::vector<DetectionResult> group;
        group.push_back(dets[i]);
        assigned[i] = true;
        
        for (size_t j = i + 1; j < dets.size(); j++) {
            if (assigned[j]) continue;
            
            // Group detections with same class_id (same robot)
            if (dets[j].class_id == dets[i].class_id) {
                // Check spatial proximity
                cv::Rect intersection = dets[i].bbox & dets[j].bbox;
                float iou = static_cast<float>(intersection.area()) /
                           static_cast<float>(dets[i].bbox.area() + dets[j].bbox.area() - intersection.area());
                
                if (iou < 0.3f) {  // Not overlapping too much
                    group.push_back(dets[j]);
                    assigned[j] = true;
                }
            }
        }
        
        if (!group.empty()) {
            grouped.push_back(group);
        }
    }
}


void DetectionWorker::select_armor(const std::vector<std::vector<DetectionResult>> &grouped_armors,
                                  std::vector<DetectionResult> &selected_armors)
{
    std::cout << "FSM state: " << static_cast<int>(tracking_fsm_) << ", selected_robot_id_: " << selected_robot_id_ << ", ttl_: " << ttl_ << std::endl;
    selected_armors.clear();
    auto pick_best_group_idx = [&]() -> int {
        if (grouped_armors.empty()) return -1;

        int best_group_idx = -1;
        float best_score = -1.0f;

        for (size_t i = 0; i < grouped_armors.size(); i++) {
            const auto &group = grouped_armors[i];
            if (group.empty()) continue;

            float group_score = 0.0f;

            for (const auto &det : group) {
                float conf_score = det.confidence_level;

                cv::Point center(det.bbox.x + det.bbox.width/2,
                                 det.bbox.y + det.bbox.height/2);

                float center_dist = std::sqrt(
                    std::pow(center.x - 320, 2) +
                    std::pow(center.y - 320, 2)
                );
                float center_score = 1.0f / (1.0f + center_dist / 100.0f);

                float size_score = std::min(1.0f,
                    (det.bbox.width * det.bbox.height) / (640.0f * 480.0f) * 10.0f);

                group_score += (conf_score * 0.5f + center_score * 0.3f + size_score * 0.2f);
            }

            group_score /= static_cast<float>(group.size());

            if (group_score > best_score) {
                best_score = group_score;
                best_group_idx = static_cast<int>(i);
            }
        }

        return best_group_idx;
    };

    // Helper: find group index by class_id
    auto find_group_by_id = [&](int id) -> int {
        if (id < 0) return -1;
        for (size_t i = 0; i < grouped_armors.size(); i++) {
            const auto &g = grouped_armors[i];
            if (!g.empty() && g[0].class_id == id) return static_cast<int>(i);
        }
        return -1;
    };

    // Helper: reset init yaw ONLY when searching (per your requirement)
    auto reset_init_yaw_from_imu = [&]() {
        std::shared_ptr<IMUState> imu_ptr = std::atomic_load(&shared_.imu);
        if (imu_ptr && imu_ptr->euler_angle.size() >= 3) {
            const float yaw_deg = imu_ptr->euler_angle[2];
            std::atomic_store(&scalars_.initial_yaw, yaw_deg);
        } else {
            std::atomic_store(&scalars_.initial_yaw, 0.0f);
        }
    };

    // Ensure TTL has a sane default
    if (ttl_ <= 0.0f) ttl_ = SELECTOR_TTL;

    // ======================= FSM =======================
    if (tracking_fsm_ == TrackingFSM::TARGET_SEARCHING) {
        // Always reset init_yaw when searching
        reset_init_yaw_from_imu();

        const int best_idx = pick_best_group_idx();
        if (best_idx >= 0) {
            selected_armors = grouped_armors[best_idx];
            selected_robot_id_ = selected_armors[0].class_id;
            ttl_ = SELECTOR_TTL;
            tracking_fsm_ = TrackingFSM::TARGET_LOCKED;
        } else {
            // stay searching
            selected_robot_id_ = -1;
            ttl_ = SELECTOR_TTL;
        }
        return;
    }

    // If we are locked or temp-lost, prefer the previously selected id if present
    const int locked_idx = find_group_by_id(selected_robot_id_);

    if (tracking_fsm_ == TrackingFSM::TARGET_LOCKED) {
        if (locked_idx >= 0) {
            // Still see same robot -> keep locked, refresh TTL
            selected_armors = grouped_armors[locked_idx];
            ttl_ = SELECTOR_TTL;
            // remain TARGET_LOCKED
        } else {
            // Missing -> decay TTL, go temp-lost
            ttl_ -= TTL_DECAY;
            tracking_fsm_ = TrackingFSM::TARGET_TEMPORARY_LOST;
            // do not change selected_robot_id_ here
        }
        return;
    }

    if (tracking_fsm_ == TrackingFSM::TARGET_TEMPORARY_LOST) {
        if (locked_idx >= 0) {
            // Reacquired -> back to locked, refresh TTL
            selected_armors = grouped_armors[locked_idx];
            ttl_ = SELECTOR_TTL;
            tracking_fsm_ = TrackingFSM::TARGET_LOCKED;
            return;
        }

        // Still missing
        if (ttl_ > 0.0f) {
            ttl_ -= TTL_DECAY;
            // stay in TARGET_TEMPORARY_LOST
            return;
        }

        // TTL expired -> back to searching, reset id + ttl + init_yaw (next call)
        tracking_fsm_ = TrackingFSM::TARGET_SEARCHING;
        selected_robot_id_ = -1;
        ttl_ = SELECTOR_TTL;
        reset_init_yaw_from_imu();
        return;
    }

    // Fallback: if unknown state, reset to searching
    tracking_fsm_ = TrackingFSM::TARGET_SEARCHING;
    selected_robot_id_ = -1;
    ttl_ = SELECTOR_TTL;
    reset_init_yaw_from_imu();
}

std::unique_ptr<RobotState>
DetectionWorker::form_robot(const std::vector<DetectionResult> &armors) {
    if (armors.empty() || armors.size() > 2) {
        if (!this->has_prev_robot_) {
            return nullptr;
        }
        return std::make_unique<RobotState>(prev_robot_);
    }

    auto rs = std::make_unique<RobotState>();
    bool form_ok = false;
    if (armors.size() == 1) {
        form_ok = from_one_armor(armors[0], prev_robot_, this->has_prev_robot_);
    } else {
        form_ok = from_two_armors(armors[0], armors[1], prev_robot_, this->has_prev_robot_);
    }

    if (form_ok) {
        this->has_prev_robot_ = true;

        auto rs = std::make_unique<RobotState>(prev_robot_);
        return rs;
    }
    if (this->has_prev_robot_) {
        return std::make_unique<RobotState>(prev_robot_);
    }

    return nullptr;
}


//---------------------------- detection helper -----------------------------//

// Compute Euclidean distance quickly
inline float tvec_distance(const Eigen::Vector3f& t) {
    return std::sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
}

static std::array<cv::Point2f, 4>
order_quad_clockwise(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() != 4) {
        return { cv::Point2f(), cv::Point2f(), cv::Point2f(), cv::Point2f() };
    }

    std::array<cv::Point2f, 4> out;
    std::vector<float> sum(4), diff(4);

    for (int i = 0; i < 4; ++i) {
        sum[i]  = pts[i].x + pts[i].y;
        diff[i] = pts[i].x - pts[i].y;
    }

    int tl = std::min_element(sum.begin(), sum.end()) - sum.begin();
    int br = std::max_element(sum.begin(), sum.end()) - sum.begin();
    int bl = std::min_element(diff.begin(), diff.end()) - diff.begin();
    int tr = std::max_element(diff.begin(), diff.end()) - diff.begin();

    out[0] = pts[tl];  // TL
    out[1] = pts[tr];  // TR
    out[2] = pts[br];  // BR
    out[3] = pts[bl];  // BL
    return out;
}

inline static void get_object_points(int armor_type, std::vector<cv::Point3f> &obj_pts)
{
    obj_pts.clear();

    float half_w, half_h;

    if (armor_type == 1) {
        // big armor
        half_w = 0.1125f;   // replace with your model
        half_h = 0.0275f;
    } else {
        // small armor
        half_w = 0.0675f;
        half_h = 0.0275f;
    }

    obj_pts.emplace_back(-half_w,  half_h, 0.0f);  // TL
    obj_pts.emplace_back( half_w,  half_h, 0.0f);  // TR
    obj_pts.emplace_back( half_w, -half_h, 0.0f);  // BR
    obj_pts.emplace_back(-half_w, -half_h, 0.0f);  // BL
}

inline static float wrap_deg(float a)
{
    // wrap angle to (-180, 180]
    while (a <= -180.0f) a += 360.0f;
    while (a >   180.0f) a -= 360.0f;
    return a;
}

inline static float angle_diff_deg(float a, float b)
{
    // smallest signed difference a - b in (-180, 180]
    return wrap_deg(a - b);
}

// Choose robot with minimum average distance of its armors
inline int choose_best_robot(const std::vector<std::vector<DetectionResult>>& grouped_armors)
{
    int best_idx = 0;
    float best_dist = std::numeric_limits<float>::max();

    for (int i = 0; i < static_cast<int>(grouped_armors.size()); i++) {
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

bool from_one_armor(const DetectionResult &det, RobotState &robot, bool &valid) {
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
        const int sector = sector_yaw(yaw_meas, prev_yaw);
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

inline void correct_yaw_to_90(float &yaw1, float &yaw2) {
    const float mid_point = (yaw1 + yaw2) / 2;
    yaw1 = wrap_pi(mid_point - M_PI_4);
    yaw2 = wrap_pi(mid_point + M_PI_4);
}

inline bool solve_linear_sys(
    float yaw1, float yaw2,
    float det1_x, float det1_z,
    float det2_x, float det2_z,
    float &x, float &z, float &r1, float &r2) {

    float s1 = std::sin(yaw1), c1 = std::cos(yaw1);
    float s2 = std::sin(yaw2), c2 = std::cos(yaw2);

    const float denom_x = (s1 - s2);
    const float denom_z = (c1 - c2);

    const float eps = 1e-4f;
    bool ok_x = std::fabs(denom_x) > eps;
    bool ok_z = std::fabs(denom_z) > eps;

    if (!ok_x && !ok_z) return false;

    float r_x = 0.f, r_z = 0.f;
    if (ok_x) r_x = (det1_x - det2_x) / denom_x;
    if (ok_z) r_z = (det2_z - det1_z) / denom_z; 
    float r = 0.f;
    if (ok_x && ok_z) {
        const float wx = std::fabs(denom_x);
        const float wz = std::fabs(denom_z);
        r = (wx * r_x + wz * r_z) / (wx + wz);
    } else if (ok_x) {
        r = r_x;
    } else {
        r = r_z;
    }

    if (r < 0.f) {
        r = -r;
        yaw1 = yaw1 + static_cast<float>(M_PI);
        yaw2 = yaw2 + static_cast<float>(M_PI);
        s1 = std::sin(yaw1); c1 = std::cos(yaw1);
        s2 = std::sin(yaw2); c2 = std::cos(yaw2);
    }

    const float x1 = det1_x - r * s1;
    const float x2 = det2_x - r * s2;
    const float z1 = det1_z + r * c1;
    const float z2 = det2_z + r * c2;

    x = 0.5f * (x1 + x2);
    z = 0.5f * (z1 + z2);

    r1 = r;
    r2 = r;
    return true;
}

inline bool from_two_armors(const DetectionResult &det1, const DetectionResult &det2,
                           RobotState &robot, bool &valid) {

    float yaw1 = det1.yaw_rad;
    float yaw2 = det2.yaw_rad;

    // Assumption: det1 yaw < det2 yaw, enforce strict 90deg constraint
    correct_yaw_to_90(yaw1, yaw2);

    float x_new, z_new, r1_new, r2_new;

    bool ok = solve_linear_sys(
        yaw1, yaw2,
        det1.tvec[0], det1.tvec[2],
        det2.tvec[0], det2.tvec[2],
        x_new, z_new, r1_new, r2_new
    );
    if (!ok) return false;

    // Height / TY as you already do (keep)
    const float h_new  = det1.tvec[1] - det2.tvec[1];
    const float ty_new = det1.tvec[1];

    // Update yaw as you already do OR keep it fixed from geometry; your code below is fine.
    // --- your existing yaw update logic ---
    if (!valid) {
        robot.state[IDX_YAW] = yaw1;
    } else {
        const float prev_yaw = robot.state[IDX_YAW];
        const int sector_armor_1 = sector_yaw(yaw1, prev_yaw);
        const int sector_armor_2 = sector_yaw(yaw2, prev_yaw);
        float y1 = wrap_pi(yaw1 + sector_armor_1 * M_PI_2);
        float y2 = wrap_pi(yaw2 + sector_armor_2 * M_PI_2);
        float mean_yaw = std::atan2(std::sin(y1) + std::sin(y2), std::cos(y1) + std::cos(y2));
        robot.state[IDX_YAW] = wrap_pi(mean_yaw);
    }

    // -------- stability layer (smoothing + constraints) ----------
    // Use a small alpha for stability; increase if laggy.
    const float alpha = valid ? 0.2f : 1.0f; // first time snap, then smooth

    auto smooth = [&](float &state, float meas) {
        state = (1.0f - alpha) * state + alpha * meas;
    };

    // Enforce r >= 0 and unify r1/r2 (ground truth)
    const float r_new = std::max(0.0f, 0.5f * (r1_new + r2_new));

    if (!valid) {
        robot.state[IDX_TX] = x_new;
        robot.state[IDX_TZ] = z_new;
        robot.state[IDX_R1] = r_new;
        robot.state[IDX_R2] = r_new;
        robot.state[IDX_H]  = h_new;
        robot.state[IDX_TY] = ty_new;
        valid = true;
        return true;
    }

    // Optional: outlier rejection to stop flicker on bad PnP frames
    const float dx = x_new - robot.state[IDX_TX];
    const float dz = z_new - robot.state[IDX_TZ];
    const float jump = std::sqrt(dx*dx + dz*dz);
    if (jump > 1.0f) { // 1 meter jump in one frame? probably bad PnP
        return true;   // keep previous robot
    }

    smooth(robot.state[IDX_TX], x_new);
    smooth(robot.state[IDX_TZ], z_new);
    smooth(robot.state[IDX_R1], r_new);
    smooth(robot.state[IDX_R2], r_new);
    smooth(robot.state[IDX_H],  h_new);
    smooth(robot.state[IDX_TY], ty_new);

    // Re-enforce non-negativity after smoothing
    robot.state[IDX_R1] = std::max(0.0f, robot.state[IDX_R1]);
    robot.state[IDX_R2] = std::max(0.0f, robot.state[IDX_R2]);

    return true;
}


