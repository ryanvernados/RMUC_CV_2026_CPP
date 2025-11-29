// calibur/camera/HikCamera.cpp
#include "calibur/camera/HikCamera.h"
#include <iostream>
#include <cstring>

HikCamera::HikCamera() = default;

HikCamera::~HikCamera() {
    close();
}

bool HikCamera::open() {
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST deviceList;
    memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        std::cerr << "[HikCamera] No camera found!" << std::endl;
        return false;
    }

    nRet = MV_CC_CreateHandle(&handle_, deviceList.pDeviceInfo[0]);
    if (nRet != MV_OK) {
        std::cerr << "[HikCamera] Create handle failed!" << std::endl;
        handle_ = nullptr;
        return false;
    }

    nRet = MV_CC_OpenDevice(handle_);
    if (nRet != MV_OK) {
        std::cerr << "[HikCamera] Open device failed!" << std::endl;
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return false;
    }

    MV_CC_SetEnumValue(handle_, "AcquisitionMode", 2);
    MV_CC_SetEnumValue(handle_, "TriggerMode", 0);

    nRet = MV_CC_StartGrabbing(handle_);
    if (nRet != MV_OK) {
        std::cerr << "[HikCamera] Start grabbing failed!" << std::endl;
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return false;
    }

    // same buffer as your working code
    bufferSize_ = 640 * 480 * 3;
    buffer_ = new unsigned char[bufferSize_];
    grabbing_ = true;

    return true;
}

bool HikCamera::grab(cv::Mat& frame) {
    if (!handle_ || !grabbing_) return false;

    MV_FRAME_OUT_INFO_EX frameInfo;
    memset(&frameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    int nRet = MV_CC_GetOneFrameTimeout(handle_, buffer_, bufferSize_, &frameInfo, 1000);
    if (nRet != MV_OK) {
        // timeout or error; just report failure, caller can continue loop
        return false;
    }

    // same as your code, but we clone so buffer can be reused
    cv::Mat img(frameInfo.nHeight, frameInfo.nWidth, CV_8UC3, buffer_);
    frame = img.clone();
    return true;
}

void HikCamera::close() {
    if (handle_) {
        if (grabbing_) {
            MV_CC_StopGrabbing(handle_);
            grabbing_ = false;
        }
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }

    delete[] buffer_;
    buffer_ = nullptr;
    bufferSize_ = 0;
}
