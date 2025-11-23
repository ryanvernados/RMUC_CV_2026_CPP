#include <cstring>
#include <thread>
#include <opencv2/opencv.hpp>  // Required for video file reading
#ifdef USE_VIDEO_FILE
#include <opencv2/videoio.hpp>
#endif
#include "types.hpp"
#include "workers.hpp"



CameraWorker::CameraWorker(void* cam_handle,
                           SharedLatest& shared,
                           std::atomic<bool>& stop_flag)
    : cam_(cam_handle), shared_(shared), stop_(stop_flag)
{
#ifdef USE_VIDEO_FILE
    cap_.open(VIDEO_PATH);
    if (!cap_.isOpened()) {
        std::cerr << "[CameraWorker] ERROR: Failed to open video file: " << VIDEO_PATH << std::endl;
        use_stub_ = true;
    } else {
        std::cout << "[CameraWorker] Reading from video file: " << VIDEO_PATH << std::endl;
        use_stub_ = false;
    }
#endif
}

void CameraWorker::operator()() {
#ifdef USE_VIDEO_FILE
    // No need for SDK start call
#else
    // MV_CC_StartGrabbing(cam_);
#endif

    while (!stop_.load(std::memory_order_relaxed)) {
        CameraFrame frame;

#ifdef USE_VIDEO_FILE
        if (!use_stub_) {
            grab_frame_from_video(frame);
        } else {
            grab_frame_stub(frame);
        }
#else
        // TODO: Real Hikvision SDK grab (e.g., MV_CC_GetImageBuffer + memcpy)
        grab_frame_stub(frame);  // placeholder until SDK integrated
#endif

        auto ptr = std::make_shared<CameraFrame>(std::move(frame));
        std::atomic_store(&shared_.camera, ptr);
        shared_.camera_ver.fetch_add(1, std::memory_order_relaxed);

        // Optional: throttle loop if needed (e.g., std::this_thread::sleep_for)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

#ifdef USE_VIDEO_FILE
    cap_.release();
#else
    // MV_CC_StopGrabbing(cam_);
#endif
}

#ifdef USE_VIDEO_FILE
void CameraWorker::grab_frame_from_video(CameraFrame &frame) {
    cv::Mat bgr_frame;
    if (!cap_.read(bgr_frame) || bgr_frame.empty()) {
        // Rewind on EOF (loop playback) — optional
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        if (!cap_.read(bgr_frame) || bgr_frame.empty()) {
            std::cerr << "[CameraWorker] Video read failed even after rewind. Using stub." << std::endl;
            use_stub_ = true;
            grab_frame_stub(frame);
            return;
        }
    }

    // Convert BGR (OpenCV default) to RGB (common for inference)
    cv::Mat rgb_frame;
    cv::cvtColor(bgr_frame, rgb_frame, cv::COLOR_BGR2RGB);

    frame.timestamp = Clock::now();
    frame.width  = rgb_frame.cols;
    frame.height = rgb_frame.rows;

    // Store directly as cv::Mat
    frame.raw_data = rgb_frame.clone();  // or rgb_frame.copyTo(frame.raw_data);
}
#endif

void CameraWorker::grab_frame_stub(CameraFrame &frame) {
    frame.timestamp = Clock::now();
    frame.width  = 640;
    frame.height = 640;

    // Create a dummy gray image (RGB=128,128,128)
    frame.raw_data = cv::Mat(frame.height,
                             frame.width,
                             CV_8UC3,
                             cv::Scalar(128, 128, 128));
}
