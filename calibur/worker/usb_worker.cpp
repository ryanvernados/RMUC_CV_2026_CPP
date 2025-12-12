#include "types.hpp"
#include <thread>
#include "workers.hpp"


USBWorker::USBWorker(SharedLatest &shared,
            SharedScalars &scalars,
            std::atomic<bool> &stop_flag)
    : shared_(shared), scalars_(scalars), stop_(stop_flag), last_pred_ver_(0) {}

void USBWorker::operator()() {
    while (!stop_.load(std::memory_order_relaxed)) {
        process_usb_rx(); // updates scalars_.bullet_speed etc.

        uint64_t cur_ver = shared_.prediction_ver.load(std::memory_order_relaxed);
        if (cur_ver == last_pred_ver_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue; // no new prediction
        }
        last_pred_ver_ = cur_ver;

        auto pred = std::atomic_load(&shared_.prediction_out);
        if (pred) {
            usb_send_tx(*pred);
        }
    }
}

void USBWorker::process_usb_rx() {
    // parse incoming packets
    // e.g. update scalars_.bullet_speed.store(new_speed);
}

void USBWorker::usb_send_tx(const PredictionOut &out) {
    // encode yaw, pitch, aim, fire, chase to COM
    std::cout << "aim: " << out.aim << "fire: " << out.fire 
            << "yaw: " << out.yaw << "pitch: " << out.pitch 
            << "chase: " << out.chase << std::endl;
    
    
}

