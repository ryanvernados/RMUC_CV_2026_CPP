#include "workers.hpp"

#include <chrono>
#include <thread>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
DisplayWorker::DisplayWorker(SharedLatest &shared, std::atomic<bool> &stop_flag)
    : shared_(shared), stop_flag_(stop_flag)
{}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void DisplayWorker::operator()() {
    cv::namedWindow("Aimbot Debug", cv::WINDOW_NORMAL);

    auto last_time = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    double fps = 0.0;

    while (!stop_flag_.load(std::memory_order_relaxed)) {

        // --- 1. CAMERA ---
        auto cam_ptr = std::atomic_load(&shared_.camera);
        if (!cam_ptr || cam_ptr->raw_data.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        cv::Mat img = cam_ptr->raw_data.clone();

        // --- 2. PREDICTION (for aim marker) ---
        auto pred_ptr = std::atomic_load(&shared_.prediction_out);
        const PredictionOut *pred = pred_ptr.get();

        // --- 3. YOLO (for original bboxes) ---
        auto yolo_ptr = std::atomic_load(&shared_.yolo);
        const YoloOutput *yolo = yolo_ptr.get();

        // --- 4. REFINED DETECTIONS (for refined keypoint boxes) ---
        auto refined_dets = shared_.refined_dets;  // Direct access, not atomic_load

        // --- Draw all overlays ---
        draw_yolo_overlay(img, yolo);          // Original YOLO boxes
        if (refined_dets && refined_dets->size() > 0) {
            draw_refined_overlay(img, refined_dets.get()); // Refined boxes from keypoints
        }
        draw_crosshair(img, pred);
        draw_target_dot(img, pred, cam_ptr->width, cam_ptr->height);
        draw_info_panel(img, pred, fps);

        // --- FPS measure ---
        frame_count++;
        auto now = std::chrono::high_resolution_clock::now();
        if (frame_count >= 20) {
            fps = 20.0 / std::chrono::duration<double>(now - last_time).count();
            frame_count = 0;
            last_time = now;
        }

        // --- Render window ---
        cv::imshow("Aimbot Debug", img);
        int key = cv::waitKey(1);
        if (key == 27) stop_flag_.store(true);
    }

    cv::destroyWindow("Aimbot Debug");
}

// ============================================================================
//  DRAW CROSSHAIR (center reticle, color-coded)
// ============================================================================
void DisplayWorker::draw_crosshair(cv::Mat &img, const PredictionOut *pred) {
    const int cx = img.cols / 2;
    const int cy = img.rows / 2;
    cv::Point center(cx, cy);

    cv::Scalar color_idle(180, 180, 180);
    cv::Scalar color_aim(0, 255, 255);
    cv::Scalar color_fire(0, 0, 255);
    cv::Scalar color = color_idle;

    if (pred) {
        if (pred->aim)  color = color_aim;
        if (pred->fire) color = color_fire;
    }

    int len = std::min(img.cols, img.rows) / 8;

    cv::line(img, cv::Point(cx - len, cy), cv::Point(cx + len, cy), color, 2);
    cv::line(img, cv::Point(cx, cy - len), cv::Point(cx, cy + len), color, 2);
    cv::circle(img, center, 5, color, 2);
}

// ============================================================================
//  DRAW YOLO BOXES (original detections - BLUE)
// ============================================================================
void DisplayWorker::draw_yolo_overlay(cv::Mat &img, const YoloOutput *yolo) {
    if (!yolo) return;

    for (const auto &det : yolo->dets) {
        cv::rectangle(img, det.bbox, cv::Scalar(255, 100, 0), 2);
        
        for (const auto &kp : det.keypoints) {
            if (kp.x >= 0 && kp.y >= 0 && kp.x < img.cols && kp.y < img.rows) {
                cv::circle(img, kp, 4, cv::Scalar(0, 255, 0), -1);
            }
        }

        char txt[32];
        std::sprintf(txt, "YOLO %d", det.class_id);
        cv::putText(img, txt,
            cv::Point(det.bbox.x, det.bbox.y - 4),
            cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(255, 100, 0), 1);
    }
}

// ============================================================================
//  DRAW REFINED OVERLAY (refined boxes from keypoints - RED)
// ============================================================================
void DisplayWorker::draw_refined_overlay(cv::Mat &img, const std::vector<DetectionResult>* refined_dets) {
    if (!refined_dets || refined_dets->empty()) return;

    for (const auto &det : *refined_dets) {
        if (det.keypoints.size() >= 4) {
            std::vector<cv::Point> polygon;
            for (size_t i = 0; i < std::min(det.keypoints.size(), size_t(4)); i++) {
                polygon.push_back(det.keypoints[i]);
            }
            
            cv::polylines(img, polygon, true, cv::Scalar(0, 0, 255), 2);
            
            for (const auto &kp : det.keypoints) {
                if (kp.x >= 0 && kp.y >= 0 && kp.x < img.cols && kp.y < img.rows) {
                    cv::circle(img, kp, 6, cv::Scalar(0, 255, 255), -1);
                }
            }
            
            if (det.keypoints.size() >= 4) {
                for (size_t i = 0; i < 4; i++) {
                    size_t j = (i + 1) % 4;
                    cv::line(img, det.keypoints[i], det.keypoints[j], 
                            cv::Scalar(0, 200, 255), 1);
                }
            }
            
            char txt[32];
            std::sprintf(txt, "REF %d", det.class_id);
            cv::putText(img, txt,
                cv::Point(det.keypoints[0].x, det.keypoints[0].y - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 255), 1);
                
        } else if (det.bbox.area() > 0) {
            cv::rectangle(img, det.bbox, cv::Scalar(0, 0, 255), 2);
            
            char txt[32];
            std::sprintf(txt, "REF %d", det.class_id);
            cv::putText(img, txt,
                cv::Point(det.bbox.x, det.bbox.y - 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 255), 1);
        }
        
        if (det.tvec.norm() > 0) {
            char dist_txt[32];
            std::sprintf(dist_txt, "d=%.2fm", det.tvec.norm());
            cv::putText(img, dist_txt,
                cv::Point(det.bbox.x + det.bbox.width + 5, det.bbox.y + 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(200, 200, 200), 1);
        }
    }
}

// ============================================================================
//  DRAW TARGET DOT (predicted hit point using yaw/pitch)
// ============================================================================
void DisplayWorker::draw_target_dot(cv::Mat &img,
                                    const PredictionOut *pred,
                                    int width, int height)
{
    if (!pred) {
        return;
    }

    const float fx = 1159.0f;
    const float fy = 1159.0f;
    const float cx = width / 2.0f;
    const float cy = height / 2.0f;

    float u = cx + fx * std::tan(pred->yaw);
    float v = cy - fy * std::tan(pred->pitch);

    u = std::max(0.0f, std::min(static_cast<float>(width - 1), u));
    v = std::max(0.0f, std::min(static_cast<float>(height - 1), v));

    cv::Point target(cvRound(u), cvRound(v));

    cv::circle(img, target, 8,
               pred->fire ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 255),
               -1);
    
    cv::circle(img, target, 8,
               cv::Scalar(255, 255, 255), 1);

    cv::line(img,
             cv::Point(cx, cy),
             target,
             cv::Scalar(0, 255, 255), 2);
}

// ============================================================================
//  DRAW INFO PANEL (yaw/pitch/fire + FPS + legend)
// ============================================================================
void DisplayWorker::draw_info_panel(cv::Mat &img, const PredictionOut *pred, double fps) {
    cv::Scalar col(0, 255, 255);
    int y = 30;

    char buf[128];
    
    cv::putText(img, "LEGEND:", cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    y += 20;
    
    cv::rectangle(img, cv::Rect(10, y, 15, 15), cv::Scalar(255, 100, 0), -1);
    cv::putText(img, ": YOLO (original)", cv::Point(30, y + 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    y += 20;
    
    cv::rectangle(img, cv::Rect(10, y, 15, 15), cv::Scalar(0, 0, 255), -1);
    cv::putText(img, ": Refined (keypoints)", cv::Point(30, y + 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    y += 20;
    
    cv::circle(img, cv::Point(10 + 7, y + 7), 4, cv::Scalar(0, 255, 0), -1);
    cv::putText(img, ": YOLO keypoints", cv::Point(30, y + 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    y += 20;
    
    cv::circle(img, cv::Point(10 + 7, y + 7), 6, cv::Scalar(0, 255, 255), -1);
    cv::putText(img, ": Refined keypoints", cv::Point(30, y + 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    y += 30;

    if (pred) {
        std::snprintf(buf, sizeof(buf),
                      "Yaw=%.3f  Pitch=%.3f",
                      pred->yaw, pred->pitch);
        cv::putText(img, buf, cv::Point(10, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, col, 2);
        y += 25;
        
        std::snprintf(buf, sizeof(buf),
                      "Aim=%d  Fire=%d  Chase=%d",
                      pred->aim, pred->fire, pred->chase);
        cv::putText(img, buf, cv::Point(10, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, col, 2);
        y += 30;
    }

    std::snprintf(buf, sizeof(buf), "FPS = %.1f", fps);
    cv::putText(img, buf, cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
}