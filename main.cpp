#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include "./calibur/worker/types.hpp"
#include "./calibur/worker/thread_pool.hpp"
#include "./calibur/worker/workers.hpp"

static std::atomic<bool> g_stop_flag{false};

void signal_handler(int) {
    g_stop_flag.store(true, std::memory_order_relaxed);
}

void* init_camera_stub() { return nullptr; }
void  shutdown_camera_stub(void*) {}

int main() {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    SharedLatest  shared;
    SharedScalars scalars;

    void* cam_handle = init_camera_stub(); // TODO: Hikvision init

    ThreadPool pool(6); // Camera, IMU, Detection, Prediction, USB

    pool.submit(CameraWorker(cam_handle, std::ref(shared), std::ref(g_stop_flag)));
    //pool.submit(IMUWorker(std::ref(shared), std::ref(g_stop_flag)));
    //pool.submit(YoloWorker(std::ref(shared), std::ref(g_stop_flag), YOLO_MODEL_PATH));
    //pool.submit(DetectionWorker(std::ref(shared), std::ref(g_stop_flag)));
    pool.submit(PredictionWorker(std::ref(shared), std::ref(scalars), std::ref(g_stop_flag)));
    //pool.submit(USBWorker(std::ref(shared), std::ref(scalars), std::ref(g_stop_flag)));

    //PFWorker pf_worker(shared, g_stop_flag);
    //std::thread pf_thread(std::ref(pf_worker));

    while (!g_stop_flag.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    //if (pf_thread.joinable()) pf_thread.join();

    shutdown_camera_stub(cam_handle);
    return 0;
}
