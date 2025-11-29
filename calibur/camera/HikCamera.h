// calibur/camera/HikCamera.h
#pragma once

#include "calibur/camera/MvCameraControl.h"
#include <opencv2/opencv.hpp>

class CameraWorker {
public:
    CameraWorker(void* cam_handle,
                 SharedLatest& shared,
                 std::atomic<bool>& stop_flag);

    void operator()();  // For thread pool execution

private:
    void* cam_;
    SharedLatest& shared_;
    std::atomic<bool>& stop_;

#ifdef USE_VIDEO_FILE
    cv::VideoCapture cap_;
    bool use_stub_ = false;
#endif

    void grab_frame_stub(CameraFrame& frame);
#ifdef USE_VIDEO_FILE
    void grab_frame_from_video(CameraFrame& frame);
#endif
};
