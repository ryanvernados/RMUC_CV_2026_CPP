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

        // --- 3. YOLO (for bboxes) ---
        auto yolo_ptr = std::atomic_load(&shared_.yolo);
        const YoloOutput *yolo = yolo_ptr.get();

        // --- Draw all overlays ---
        draw_yolo_overlay(img, yolo);
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
    // std::cout << "img.cols=" << img.cols << " img.rows=" << img.rows << std::endl;
    const int cx = img.cols / 2;
    const int cy = img.rows / 2;
    cv::Point center(cx, cy);

    cv::Scalar color_idle  (180,180,180);
    cv::Scalar color_aim   (  0,255,255);
    cv::Scalar color_fire  (  0,  0,255);
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
//  DRAW TARGET DOT (predicted hit point using yaw/pitch)
// ============================================================================
void DisplayWorker::draw_target_dot(cv::Mat &img,
                                    const PredictionOut *pred,
                                    int width, int height)
{
    // std::cout << "img.cols=" << width << " img.rows=" << height << std::endl;
    if (!pred) return;

    // Camera intrinsics approximated → replace with your exact values
    const float fx = 1219.50f;
    const float fy = 1218.2f;
    const float cx = 676.9765;
    const float cy = 584.9604;

    // Convert yaw/pitch to pixel displacement
    float u = cx + fx * std::tan(pred->yaw);
    float v = cy - fy * std::tan(pred->pitch);

    cv::Point target(cvRound(u), cvRound(v));

    // Draw predicted point
    cv::circle(img, target, 6,
               pred->fire ? cv::Scalar(0,0,255) : cv::Scalar(0,255,255),
               -1);

    // Draw line from center → target
    cv::line(img,
             cv::Point(cx, cy),
             target,
             cv::Scalar(0,255,255), 2);
}



// ============================================================================
//  DRAW YOLO BOXES
// ============================================================================
void DisplayWorker::draw_yolo_overlay(cv::Mat &img, const YoloOutput *yolo) {
    if (!yolo) return;

    for (const auto &det : yolo->dets) {
        cv::rectangle(img, det.bbox, cv::Scalar(255,0,0), 2);

        for (auto &kp : det.keypoints)
            cv::circle(img, kp, 4, cv::Scalar(0,255,0), -1);

        char txt[32];
        std::sprintf(txt, "id=%d", det.class_id);
        cv::putText(img, txt,
            cv::Point(det.bbox.x, det.bbox.y - 4),
            cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(0,255,255), 2);
    }
}



// ============================================================================
//  DRAW INFO PANEL (yaw/pitch/fire + FPS)
// ============================================================================
void DisplayWorker::draw_info_panel(cv::Mat &img, const PredictionOut *pred, double fps) {
    cv::Scalar col(0,255,255);
    int y = 30;

    char buf[128];
    if (pred) {
        std::snprintf(buf, sizeof(buf),
                      "Yaw=%.2f  Pitch=%.2f  Aim=%d  Fire=%d",
                      pred->yaw, pred->pitch,
                      pred->aim, pred->fire);
        cv::putText(img, buf, cv::Point(10, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, col, 2);
        y += 30;
    }

    std::snprintf(buf, sizeof(buf), "FPS = %.1f", fps);
    cv::putText(img, buf, cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0,255,0), 2);
}
