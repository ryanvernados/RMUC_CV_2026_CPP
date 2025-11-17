#include "calibur/camera/MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>

int main() {
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST deviceList;
    memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        std::cerr << "No camera found!" << std::endl;
        return -1;
    }

    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, deviceList.pDeviceInfo[0]);
    if (nRet != MV_OK) {
        std::cerr << "Create handle failed!" << std::endl;
        return -1;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        std::cerr << "Open device failed!" << std::endl;
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);
    MV_CC_SetEnumValue(handle, "TriggerMode", 0);

    nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        std::cerr << "Start grabbing failed!" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    std::cout << "Press ESC to exit." << std::endl;

    MV_FRAME_OUT_INFO_EX frameInfo;
    memset(&frameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    const unsigned int maxBuffer = 640 * 480 * 3;
    unsigned char* pData = new unsigned char[maxBuffer];

    while (true) {
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, maxBuffer, &frameInfo, 1000);
        if (nRet == MV_OK) {
            // std::cout << "Frame " << frameInfo.nFrameNum
            //   << ": " << frameInfo.nWidth << "x" << frameInfo.nHeight
            //   << ", PixelType=0x" << std::hex << frameInfo.enPixelType << std::dec
            //   << ", FrameLen=" << frameInfo.nFrameLen
            //   << std::endl;
            cv::Mat img(frameInfo.nHeight, frameInfo.nWidth, CV_8UC3, pData);
            cv::imshow("HIKRobot MV-CS016-10UC", img);
            if (cv::waitKey(1) == 27) break;
        }
    }

    delete[] pData;
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 0;
}
