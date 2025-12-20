#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include "./calibur/worker/types.hpp"
#include "./calibur/worker/thread_pool.hpp"
#include "./calibur/worker/workers.hpp"
#include "../camera/MvCameraControl.h"

static std::atomic<bool> g_stop_flag{false};

void signal_handler(int) {
    g_stop_flag.store(true, std::memory_order_relaxed);
}

void* init_camera_stub() { 
    int nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST deviceList;
    memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        std::cerr << "[CameraWorker] No camera found! nRet=" << nRet << std::endl;
        return nullptr;
    }
    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, deviceList.pDeviceInfo[0]);
    if (nRet != MV_OK) {
        std::cerr << "[CameraWorker] Create handle failed: " << nRet << std::endl;
        return nullptr;
    }
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        std::cerr << "[CameraWorker] Open device failed: " << nRet << std::endl;
        MV_CC_DestroyHandle(handle);
        return nullptr;
    }
    // MV_CC_SetIntValue(handle, "OffsetX", 0);
    // MV_CC_SetIntValue(handle, "OffsetY", 0);
    MV_CC_SetIntValue(handle, "Height", 1080);
    MV_CC_SetIntValue(handle, "Width", 1080);
    MV_CC_SetEnumValue(handle, "ExposureAuto", 2); 
    MV_CC_SetFloatValue(handle, "ExposureTime", 7000.0f);
    MV_CC_SetEnumValue(handle, "GainAuto", 2);
    MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);
    MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    std::cout << "[CameraWorker] Hik camera initialized.\n";
    return handle;
}

void shutdown_camera_stub(void* handle) {
    if (!handle) return;
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    std::cout << "[CameraWorker] Hik camera closed.\n";
}

int main() {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);
    SharedLatest  shared;
    SharedScalars scalars;
    auto usb_comm = std::make_shared<calibur::USBCommunication>("/dev/ttyACM1");
    
    // Initialize refined_dets to empty
    shared.refined_dets = std::make_shared<std::vector<DetectionResult>>();
    
    void* cam_handle = init_camera_stub();
    ThreadPool pool(7);
    CameraMode mode = CameraMode::HIK_USB; 
    pool.submit(CameraWorker(cam_handle, shared, g_stop_flag, mode));
    pool.submit([&shared, &g_stop_flag]() {
        IMUWorker worker(shared, g_stop_flag);
        worker();
    });
    pool.submit(YoloWorker(std::ref(shared), std::ref(g_stop_flag), YOLO_MODEL_PATH));
    pool.submit(DetectionWorker(std::ref(shared), std::ref(scalars), std::ref(g_stop_flag)));
    pool.submit(PredictionWorker(std::ref(shared), std::ref(scalars), std::ref(g_stop_flag)));
    pool.submit(USBWorker(std::ref(shared), std::ref(scalars), std::ref(g_stop_flag), std::ref(usb_comm)));
    pool.submit([&shared, &g_stop_flag]() {
        DisplayWorker worker(shared, g_stop_flag);
        worker();
    });
    PFWorker pf_worker(shared, g_stop_flag);
    std::thread pf_thread(std::ref(pf_worker));
    while (!g_stop_flag.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (pf_thread.joinable()) pf_thread.join();
    shutdown_camera_stub(cam_handle);
    return 0;
}