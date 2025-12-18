// rbpf.cuh
#pragma once

#include <curand_kernel.h>
#include "workers.hpp"
#include "types.hpp"

constexpr int CUDA_BLOCK_SIZE = 256;

// ===================== Process Noise (PF) =====================
constexpr float Q_POS_DIFFUSION     = 1.2e-3f;
constexpr float Q_YAW_DIFFUSION     = 4.0e-3f; 
constexpr float Q_ACC_RANDOMWALK    = 6.0e-3f;  
constexpr float Q_YAWACC_RANDOMWALK = 8.0e-4f;   

// ===================== Process Noise (KF) =====================
constexpr float Q_VEL_DIFFUSION     = 1.5e-3f;
constexpr float Q_YAWRATE_DIFFUSION = 5.0e-4f;
constexpr float Q_GEOM_DRIFT        = 1.0e-6f;    

// ===================== Measurement Noise =====================
constexpr float RZ_POS_NOISE        = 9.0e-2f;
constexpr float RZ_YAW_NOISE        = 1.5e-1f;  

// ===================== Pseudo-measurements ===================
constexpr float RY_VEL_NOISE        = 3.0e-1f;
constexpr float RY_YAWR_NOISE       = 1.8e-1f;  

// ===================== Geometry direct measurement ==========
constexpr float RC_GEOM_NOISE       = 5e-3f;   


// ===================== Initialization spreads ================
constexpr float INIT_VEL_STD       = 0.2f;
constexpr float INIT_GEOM_MEAN_R   = 0.30f;
constexpr float INIT_GEOM_MEAN_H   = 0.0f;
constexpr float INIT_GEOM_STD_R    = 0.05f;
constexpr float INIT_GEOM_STD_H    = 0.05f;

// ======================= CONFIG ==========================
constexpr int D    = 15;  // PF state dim
constexpr int KF_D = 7;   // KF state dim: [vx,vy,vz,yaw_rate,r1,r2,h]

// ======================= PARAMS ==========================

struct RBPFParams {
    // Process noise (PF)
    float Q_pos_diag[3];
    float Q_yaw;
    float Q_acc_diag[3];
    float Q_yawalpha;

    // KF process noise
    float Q_vel_diag[4];    // [vx,vy,vz,yaw_rate]
    float Q_geom_diag[3];   // [r1,r2,h]

    // Measurement covariances
    float Rz_pos_diag[3];
    float Rz_yaw;

    float Ry_vel_diag[3];
    float Ry_yawr;

    float Rc_geom_diag[3];

    // Init spreads
    float init_vel_std[4];
    float init_geom_mean[3];
    float init_geom_std[3];
};

// ======================= DEVICE STATE ====================

struct RBPFDevice {
    int N;

    float *X;           // [N * D]
    float *kf_mean;     // [N * KF_D]
    float *P_vel_diag;  // [N * 4]
    float *P_geom_diag; // [N * 3]

    float *prev_pos;    // [N * 3]
    float *prev_yaw;    // [N]

    curandState *rng_states; // [N]
};

// Forward declaration of default params
RBPFParams default_params();

// ======================= MAIN GPU OBJECT =================

struct RBPFPosYawModelGPU {
    int N;
    RBPFParams params;
    RBPFDevice dev;

    float *d_W;
    float *d_loglik;
    float *d_mean;
    float *d_obs;
    float *d_cdf;

    // scratch
    float *d_max = nullptr;
    float *d_sum = nullptr;
    float *X_new = nullptr;
    float *kf_new = nullptr;
    float *Pvel_new = nullptr;
    float *Pgeom_new = nullptr;

    float *d_ess_inv = nullptr;

    float z_yaw_prev;
    cudaStream_t stream;

    // ctor / dtor
    RBPFPosYawModelGPU(int N_, const RBPFParams &p = default_params());
    ~RBPFPosYawModelGPU();

    // host->device state init
    void set_state_from_host(const float *h_X);
    void set_state_single(const float *X0);

    // device-side steps (no host pointers)
    void predict_device(float dt);
    void loglik_device(); // uses d_obs -> d_loglik
    void kf_update_device(float dt,
                          bool have_yaw_obs, float y_obs, float R_obs_yawr,
                          bool have_geom_obs, float g0, float g1, float g2);
    void mean_device();
};

// ======================= C API WRAPPERS ==================

RBPFPosYawModelGPU *rbpf_create(int N);
void rbpf_destroy(RBPFPosYawModelGPU *pf);

void rbpf_reset_from_meas(RBPFPosYawModelGPU *pf, const RobotState &meas);
void rbpf_predict(RBPFPosYawModelGPU *pf, float dt);
void rbpf_step(RBPFPosYawModelGPU *pf, const RobotState &meas, float dt);
RobotState rbpf_get_mean(RBPFPosYawModelGPU *pf);
void gpu_update_and_normalize_weights(
        const float *d_loglik,
        float *d_W,
        int N,
        float *d_max,
        float *d_sum,
        float *d_ess_inv,
        cudaStream_t stream);
void gpu_set_uniform_weights(float *d_W, int N, cudaStream_t stream);
void gpu_resample_particles(
        RBPFDevice dev,
        float *d_W,
        float *d_cdf,
        float *X_new,
        float *kf_new,
        float *Pvel_new,
        float *Pgeom_new,
        int N,
        cudaStream_t stream);
