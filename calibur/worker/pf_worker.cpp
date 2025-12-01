#include "types.hpp"
#include <thread>
#include "workers.hpp"
#include "rbpf.cuh"


using timestamp_clock_t= std::chrono::steady_clock;

PFWorker::PFWorker(SharedLatest &shared,
                   std::atomic<bool> &stop_flag)
    : shared_(shared),
      stop_(stop_flag),
      last_det_ver_(0),
      g_pf(nullptr)
{
}

void PFWorker::gpu_pf_init() {
    if (!g_pf) {
        // Use your existing factory, but hand ownership to unique_ptr
        g_pf.reset(rbpf_create(NUM_PARTICLES));

        // Optionally reset from some initial guess
        RobotState init{};
        rbpf_reset_from_meas(g_pf.get(), init);
    }
}

void PFWorker::gpu_pf_reset(const RobotState &meas) {
    rbpf_reset_from_meas(g_pf.get(), meas);
}

void PFWorker::gpu_pf_predict_only() {
    rbpf_predict(g_pf.get(), kDt);
}

void PFWorker::gpu_pf_step(const RobotState &meas) {
    rbpf_step(g_pf.get(), meas, kDt);
}

RobotState PFWorker::gpu_return_result(){
    return rbpf_get_mean(g_pf.get());
}


void PFWorker::operator()() {
    gpu_pf_init();
    auto next_tick = timestamp_clock_t::now();

    while (!stop_.load(std::memory_order_acquire)) {
        next_tick += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next_tick);

        bool       has_meas = false;
        RobotState meas;
        uint64_t   det_ver = shared_.detection_ver.load(std::memory_order_acquire);

        if (det_ver != last_det_ver_) {
            auto det_ptr = shared_.detection_out;
            if (det_ptr) {
                meas          = *det_ptr;
                last_det_ver_ = det_ver;
                has_meas      = true;
            }
        }
#ifdef PERFORMANCE_BENCHMARK
        auto t0 = std::chrono::high_resolution_clock::now();
#endif
        RobotState pf_state;
        if (has_meas) {
            gpu_pf_step(meas);
        } else {
            gpu_pf_predict_only();
        }
        pf_state = gpu_return_result();
        
        // std::cout << "PF out: ";
        // for (int i = 0; i < 15; i++){
        //     std::cout << pf_state.state[i] << ", " << std::endl;
        // }

#ifdef PERFORMANCE_BENCHMARK
        //auto t1 = std::chrono::high_resolution_clock::now();
        //double infer_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        //std::cout << "[PF] inference time = " << infer_ms << " ms\n";
#endif
        shared_.pf_out = std::make_shared<RobotState>(pf_state);
        shared_.pf_ver.fetch_add(1, std::memory_order_acq_rel);
    }
}


