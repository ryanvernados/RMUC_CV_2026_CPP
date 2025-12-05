#include "workers.hpp"
#include "../camera/MvCameraControl.h"

#include <iostream>
#include <thread>
#include <cstring>
#include <opencv2/highgui.hpp>



// -------------------------------- CameraWorker --------------------------------

CameraWorker::CameraWorker(void* cam_handle,
                           SharedLatest& shared,
                           std::atomic<bool>& stop_flag,
                           CameraMode mode)
    : cam_(cam_handle),
      shared_(shared),
      stop_(stop_flag),
      mode_(mode)
{
    if (mode_ == CameraMode::VIDEO_FILE) {
        // Open debug video file
        cap_.open(VIDEO_PATH);
        if (!cap_.isOpened()) {
            std::cerr << "[CameraWorker] ERROR: Failed to open video file: "
                      << VIDEO_PATH << std::endl;
            use_stub_ = true;
        } else {
            std::cout << "[CameraWorker] Using video file: "
                      << VIDEO_PATH << std::endl;
            use_stub_ = false;
        }
    } else {
        // HIK_USB mode: assume handle is created & device opened in main()
        std::cout << "[CameraWorker] Using Hikrobot camera handle: " << cam_
                  << std::endl;
        use_stub_ = (cam_ == nullptr);
        if (use_stub_) {
            std::cerr << "[CameraWorker] Null camera handle! Falling back to stub.\n";
        }
    }
}


void CameraWorker::operator()() {
    int nRet = MV_OK;


    // Start grabbing for Hik mode
    if (!use_stub_ && mode_ == CameraMode::HIK_USB) {
        nRet = MV_CC_StartGrabbing(cam_);
        if (nRet != MV_OK) {
            std::cerr << "[CameraWorker] MV_CC_StartGrabbing failed: " << nRet
                      << ". Using stub frames instead.\n";
            use_stub_ = true;
        }
    }

    while (!stop_.load(std::memory_order_relaxed)) {
        CameraFrame frame;

        // 1) Acquire frame
        if (use_stub_) {
            grab_frame_stub(frame);
        } else if (mode_ == CameraMode::HIK_USB) {
            grab_frame_from_hik(frame);
        } else { // VIDEO_FILE
            grab_frame_from_video(frame);
        }

        // 2) Publish raw frame to shared
        auto ptr = std::make_shared<CameraFrame>(std::move(frame));
        std::atomic_store(&shared_.camera, ptr);
        shared_.camera_ver.fetch_add(1, std::memory_order_relaxed);


        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!use_stub_ && mode_ == CameraMode::HIK_USB) {
        MV_CC_StopGrabbing(cam_);
    }

    if (mode_ == CameraMode::VIDEO_FILE && cap_.isOpened()) {
        cap_.release();
    }

}

void CameraWorker::grab_frame_stub(CameraFrame &frame) {
    frame.timestamp = Clock::now();
    frame.width  = 640;
    frame.height = 640;

    frame.raw_data = cv::Mat(
        frame.height,
        frame.width,
        CV_8UC3,
        cv::Scalar(128, 128, 128));
}

// ---------- HIK camera grab ----------
void CameraWorker::grab_frame_from_hik(CameraFrame &frame) {
    static const unsigned int MAX_SRC_BUF = 1080 * 1080 * 3;
    static unsigned char* pSrcData = [](){
        auto ptr = new unsigned char[MAX_SRC_BUF];
        return ptr;
    }();

    // Big enough for RGB/BGR8
    static const unsigned int MAX_DST_BUF = 1080 * 1080 * 3;
    static unsigned char* pDstData = [](){
        auto ptr = new unsigned char[MAX_DST_BUF];
        return ptr;
    }();

    MV_FRAME_OUT_INFO_EX frameInfo;
    memset(&frameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    int nRet = MV_CC_GetOneFrameTimeout(
        cam_,
        pSrcData,
        MAX_SRC_BUF,
        &frameInfo,
        1000  // timeout ms
    );

    if (nRet != MV_OK) {
        std::cerr << "[CameraWorker] MV_CC_GetOneFrameTimeout failed: "
                  << nRet << std::endl;
        grab_frame_stub(frame);
        return;
    }

    frame.timestamp = Clock::now();
    frame.width  = frameInfo.nWidth;
    frame.height = frameInfo.nHeight;

    // --- If already BGR8/RGB8, we can skip conversion ---
    if (frameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
        cv::Mat bgr(frameInfo.nHeight,
                    frameInfo.nWidth,
                    CV_8UC3,
                    pSrcData);
        frame.raw_data = bgr.clone();
        return;
    }
    if (frameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
        cv::Mat rgb(frameInfo.nHeight,
                    frameInfo.nWidth,
                    CV_8UC3,
                    pSrcData);
        // Convert RGB to BGR for OpenCV
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        frame.raw_data = bgr;
        return;
    }

    // --- Otherwise, convert to BGR8 using MV_CC_ConvertPixelType ---
    MV_CC_PIXEL_CONVERT_PARAM convParam;
    memset(&convParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
    convParam.nWidth          = frameInfo.nWidth;
    convParam.nHeight         = frameInfo.nHeight;
    convParam.pSrcData        = pSrcData;
    convParam.nSrcDataLen     = frameInfo.nFrameLen;
    convParam.enSrcPixelType  = frameInfo.enPixelType;
    convParam.enDstPixelType  = PixelType_Gvsp_BGR8_Packed;
    convParam.pDstBuffer      = pDstData;
    convParam.nDstBufferSize  = MAX_DST_BUF;

    nRet = MV_CC_ConvertPixelType(cam_, &convParam);
    if (nRet != MV_OK) {
        std::cerr << "[CameraWorker] MV_CC_ConvertPixelType failed: "
                  << nRet << " (src pixel type 0x"
                  << std::hex << frameInfo.enPixelType << std::dec << ")\n";
        grab_frame_stub(frame);
        return;
    }

    cv::Mat bgr(frameInfo.nHeight,
                frameInfo.nWidth,
                CV_8UC3,
                pDstData);
    frame.raw_data = bgr.clone();
}


// ---------- Video file grab ----------
void CameraWorker::grab_frame_from_video(CameraFrame &frame) {
    cv::Mat bgr;
    if (!cap_.read(bgr) || bgr.empty()) {
        // Loop when reaching EOF
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);

        if (!cap_.read(bgr) || bgr.empty()) {
            std::cerr << "[CameraWorker] Video read failed after rewind. "
                         "Switching to stub.\n";
            use_stub_ = true;
            grab_frame_stub(frame);
            return;
        }
    }

    frame.timestamp = Clock::now();
    frame.width  = bgr.cols;
    frame.height = bgr.rows;
    frame.raw_data = bgr.clone();
}
