#include "types.hpp"
#include <thread>
#include "workers.hpp"
#include "usb_communication.h"
#include "protocol_data.hpp"
#include "../log.h"

// static calibur::Logger::ptr g_logger = CALIBUR_LOG_NAME("usb");

USBWorker::USBWorker(SharedLatest &shared,
            SharedScalars &scalars,
            std::atomic<bool> &stop_flag,
            std::shared_ptr<calibur::USBCommunication> usb_comm)
    : shared_(shared), scalars_(scalars), stop_(stop_flag), last_pred_ver_(0), usb_comm_(std::move(usb_comm)) {}

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
         
        Protocol::AngleData ang{.pitch = 10.5f, .yaw = -22.0f};
        usb_comm_->sendData(ang);
        Protocol::CommandData cmd{.fire = 1, .aim = 0, ..chase = 1};
        usb_comm_->sendData(ang);
        
        // Switch off for now, uncomment when pipeline is stable to send data
        // if (pred) {
        //     usb_send_tx(*pred);
        // }

    }
}

void USBWorker::process_usb_rx() {
    // parse incoming packets
    // e.g. update scalars_.bullet_speed.store(new_speed);
}

void USBWorker::usb_send_tx(const PredictionOut &out) {
    // encode yaw, pitch, aim, fire, chase to COM
    usb_comm_->sendData(out.yaw, out.pitch, out.aim, out.fire, out.chase);
}

