#include <thread>

#include "infer.h"
#include "workers.hpp"

// ==================== Function Prototype Declaration ======================
inline void parse_detection_result(const Detection& d, DetectionResult& r);

// ==================== YOLO Worker Class Functions =========================
YoloWorker::YoloWorker(SharedLatest& shared,
                       std::atomic<bool>& stop_flag,
                       const std::string& engine_path)
    : shared_(shared),
      stop_(stop_flag),
      detector_("calibur/models/yolo11-pose3.engine")          // <-- construct member here
{}

void YoloWorker::operator()() {
    static thread_local std::vector<DetectionResult> dets;

    while (!stop_.load(std::memory_order_relaxed)) {

        uint64_t cur_ver = shared_.camera_ver.load(std::memory_order_relaxed);
        if (cur_ver == last_cam_ver_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue; // no new camera frame
        }
        last_cam_ver_ = cur_ver;

        auto cam = std::atomic_load(&shared_.camera);
        if (!cam || cam->raw_data.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        std::vector<Detection> yolo_dets = detector_.inference(cam->raw_data);

        dets.clear();
        dets.reserve(yolo_dets.size());
        for (const auto& d : yolo_dets) {
            DetectionResult r{};
            parse_detection_result(d, r);
            dets.emplace_back(r);
        }

        // ---- 4) publish YOLO output ----
        auto yo = std::make_shared<YoloOutput>();
        yo->dets      = dets;           // or std::move(dets) if you don’t reuse
        yo->width     = cam->width;
        yo->height    = cam->height;
        yo->timestamp = cam->timestamp;

        std::atomic_store(&shared_.yolo, yo);
        shared_.yolo_ver.fetch_add(1, std::memory_order_relaxed);
    }
}


inline void parse_detection_result(const Detection& d, DetectionResult& r) {
    const float x1 = d.bbox[0];
    const float y1 = d.bbox[1];
    const float x2 = d.bbox[2];
    const float y2 = d.bbox[3];

    int ix = static_cast<int>(x1 + 0.5f);
    int iy = static_cast<int>(y1 + 0.5f);
    int iw = static_cast<int>(x2 - x1 + 0.5f);
    int ih = static_cast<int>(y2 - y1 + 0.5f);

    if (iw < 0) iw = 0;
    if (ih < 0) ih = 0;

    r.bbox = cv::Rect(ix, iy, iw, ih);

    // --- basic fields ---
    r.class_id         = d.classId;
    r.confidence_level = d.conf;

    // leave these for later stages to fill
    r.tvec.setZero();
    r.rvec     = cv::Vec3f(0.f, 0.f, 0.f);
    r.yaw_rad  = 0.f;
    r.armor_type = 0;   // or map from classId if you have a rule

    // --- keypoints ---
    // Prefer vKpts if it’s already scaled; otherwise fall back to raw kpts[]
    if (!d.vKpts.empty()) {
        const std::size_t n = d.vKpts.size();
        r.keypoints.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            const auto& k = d.vKpts[i];  // {x, y, visible}
            // assume k.size() >= 2
            r.keypoints[i].x = k[0];
            r.keypoints[i].y = k[1];
        }
    } else {
        constexpr int N = kNumKpt;
        constexpr int D = kKptDims; // 3
        r.keypoints.resize(N);
        for (int i = 0; i < N; ++i) {
            const int base = i * D;
            r.keypoints[i].x = d.kpts[base + 0];
            r.keypoints[i].y = d.kpts[base + 1];
        }
    }
}
