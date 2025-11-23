// rbpf_posyaw.cu
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <cmath>
#include <cstdio>
#include "rbpf.cuh"



__host__ RBPFParams default_params() {
    RBPFParams p{};

    // PF diffusion
    p.Q_pos_diag[0] = p.Q_pos_diag[1] = p.Q_pos_diag[2] = Q_POS_DIFFUSION;
    p.Q_yaw = Q_YAW_DIFFUSION;
    p.Q_acc_diag[0] = p.Q_acc_diag[1] = p.Q_acc_diag[2] = Q_ACC_RANDOMWALK;
    p.Q_yawalpha = Q_YAWACC_RANDOMWALK;

    // KF diffusion
    p.Q_vel_diag[0] = p.Q_vel_diag[1] = p.Q_vel_diag[2] = Q_VEL_DIFFUSION;
    p.Q_vel_diag[3] = Q_YAWRATE_DIFFUSION;
    p.Q_geom_diag[0] = p.Q_geom_diag[1] = p.Q_geom_diag[2] = Q_GEOM_DRIFT;

    // Measurement noise
    p.Rz_pos_diag[0] = p.Rz_pos_diag[1] = p.Rz_pos_diag[2] = RZ_POS_NOISE;
    p.Rz_yaw = RZ_YAW_NOISE;

    p.Ry_vel_diag[0] = p.Ry_vel_diag[1] = p.Ry_vel_diag[2] = RY_VEL_NOISE;
    p.Ry_yawr       = RY_YAWR_NOISE;

    p.Rc_geom_diag[0] = p.Rc_geom_diag[1] = p.Rc_geom_diag[2] = RC_GEOM_NOISE;

    // Init spreads
    p.init_vel_std[0] = p.init_vel_std[1] = p.init_vel_std[2] = INIT_VEL_STD;
    p.init_vel_std[3] = INIT_VEL_STD;

    p.init_geom_mean[0] = p.init_geom_mean[1] = INIT_GEOM_MEAN_R;
    p.init_geom_std[0]  = p.init_geom_std[1]  = INIT_GEOM_STD_R;
    p.init_geom_mean[2] = INIT_GEOM_MEAN_H;
    p.init_geom_std[2]  = INIT_GEOM_STD_H;

    return p;
}

// ======================= DEVICE HELPERS ==================

__device__ inline float wrap_to_pi(float a) {
    float twopi = 2.0f * M_PI;
    a = fmodf(a + M_PI, twopi);
    if (a < 0.0f) a += twopi;
    return a - M_PI;
}

__device__ float gaussian(curandState *state) {
    return curand_normal(state);
}

// ======================= KERNELS =========================

// RNG init
__global__ void init_rng_kernel(curandState *rng_states, int N, unsigned long long seed) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;
    curand_init(seed, i, 0, &rng_states[i]);
}

// KF attach/init (similar to Python attach)
__global__ void kf_attach_kernel(RBPFDevice dev, RBPFParams params) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= dev.N) return;

    curandState local = dev.rng_states[i];

    float *mean = &dev.kf_mean[i * KF_D];
    float *Pvel = &dev.P_vel_diag[i * 4];
    float *Pgeom = &dev.P_geom_diag[i * 3];

    // vel0 = N(0, init_vel_std)
    for (int k = 0; k < 4; ++k) {
        float z = gaussian(&local);
        mean[k] = z * params.init_vel_std[k];
        Pvel[k] = params.init_vel_std[k] * params.init_vel_std[k];
    }

    // geom0 = init_geom_mean + N(0, init_geom_std)
    for (int k = 0; k < 3; ++k) {
        float z = gaussian(&local);
        mean[4 + k] = params.init_geom_mean[k] + z * params.init_geom_std[k];
        Pgeom[k] = params.init_geom_std[k] * params.init_geom_std[k];
    }

    // prev_pos, prev_yaw start at 0
    dev.prev_pos[i * 3 + 0] = 0.0f;
    dev.prev_pos[i * 3 + 1] = 0.0f;
    dev.prev_pos[i * 3 + 2] = 0.0f;
    dev.prev_yaw[i] = 0.0f;

    dev.rng_states[i] = local;
}

// on_before_predict: cache current pos and yaw
__global__ void on_before_predict_kernel(RBPFDevice dev) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= dev.N) return;

    float *Xi = &dev.X[i * D];

    dev.prev_pos[i * 3 + 0] = Xi[IDX_TX];
    dev.prev_pos[i * 3 + 1] = Xi[IDX_TY];
    dev.prev_pos[i * 3 + 2] = Xi[IDX_TZ];

    dev.prev_yaw[i] = Xi[IDX_YAW];
}

// predict kernel (PF + KF cov predict)
__global__ void predict_kernel(RBPFDevice dev, RBPFParams params, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= dev.N) return;
    if (dt <= 0.0f) return;

    curandState local = dev.rng_states[i];
    float *Xi    = &dev.X[i * D];
    float *mean  = &dev.kf_mean[i * KF_D];
    float *Pvel  = &dev.P_vel_diag[i * 4];
    float *Pgeom = &dev.P_geom_diag[i * 3];

    // === KF covariance predict (diag) ===
    for (int k = 0; k < 4; ++k) {
        Pvel[k]  += params.Q_vel_diag[k]  * dt;
    }
    for (int k = 0; k < 3; ++k) {
        Pgeom[k] += params.Q_geom_diag[k] * dt;
    }

    // === Precompute std for PF noises ===
    float qpos_std[3];
    float qacc_std[3];
    for (int k = 0; k < 3; ++k) {
        qpos_std[k] = sqrtf(params.Q_pos_diag[k] * dt);
        qacc_std[k] = sqrtf(params.Q_acc_diag[k] * dt);
    }
    float qyaw_std  = sqrtf(params.Q_yaw      * dt);
    float qyawa_std = sqrtf(params.Q_yawalpha * dt);

    float qgeom_std[3];
    for (int k = 0; k < 3; ++k) {
        qgeom_std[k] = sqrtf(params.Q_geom_diag[k] * dt);
    }

    // === State views ===
    float &x = Xi[IDX_TX];
    float &y = Xi[IDX_TY];
    float &z = Xi[IDX_TZ];

    float &vx = Xi[IDX_VX];
    float &vy = Xi[IDX_VY];
    float &vz = Xi[IDX_VZ];

    float &ax = Xi[IDX_AX];
    float &ay = Xi[IDX_AY];
    float &az = Xi[IDX_AZ];

    float &yaw  = Xi[IDX_YAW];
    float &yawr = Xi[IDX_OMEGA];
    float &yawa = Xi[IDX_ALPHA];

    float &r1 = Xi[IDX_R1];
    float &r2 = Xi[IDX_R2];
    float &h  = Xi[IDX_H];

    // Pull KF mean velocities / yaw_rate into PF state
    vx   = mean[0];
    vy   = mean[1];
    vz   = mean[2];
    yawr = mean[3];
    r1   = mean[4];
    r2   = mean[5];
    h    = mean[6];

    // --- draw noises ---
    float n_pos[3];
    float n_acc[3];
    for (int k = 0; k < 3; ++k) {
        n_pos[k] = gaussian(&local) * qpos_std[k];
        n_acc[k] = gaussian(&local) * qacc_std[k];
    }
    float n_yaw  = gaussian(&local) * qyaw_std;
    float n_yawa = gaussian(&local) * qyawa_std;
    float n_geom[3];
    for (int k = 0; k < 3; ++k) {
        n_geom[k] = gaussian(&local) * qgeom_std[k];
    }

    float dt2 = dt * dt;

    // --- linear CA integration ---
    x += vx * dt + 0.5f * ax * dt2 + n_pos[0];
    y += vy * dt + 0.5f * ay * dt2 + n_pos[1];
    z += vz * dt + 0.5f * az * dt2 + n_pos[2];

    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;

    ax += n_acc[0];
    ay += n_acc[1];
    az += n_acc[2];

    // --- yaw CA integration ---
    yaw  = wrap_to_pi(yaw + yawr * dt + 0.5f * yawa * dt2 + n_yaw);
    yawr += yawa * dt;
    yawa += n_yawa;

    // --- geom slow drift ---
    r1 += n_geom[0];
    r2 += n_geom[1];
    h  += n_geom[2];

    dev.rng_states[i] = local;
}

// log-likelihood kernel: pos(3) + yaw
__global__ void loglik_kernel(
        RBPFDevice dev,
        RBPFParams params,
        const float *z,  // obs vector, layout same as state: z[0..2]=tvec, z[9]=yaw
        float *out_loglik)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= dev.N) return;

    const float *Xi = &dev.X[i * D];

    // Position diff
    float diff_p[3];
    diff_p[0] = Xi[IDX_TX] - z[IDX_TX];
    diff_p[1] = Xi[IDX_TY] - z[IDX_TY];
    diff_p[2] = Xi[IDX_TZ] - z[IDX_TZ];

    float quad_p = 0.0f;
    float const_p = 0.0f;
    for (int k = 0; k < 3; ++k) {
        float invR = 1.0f / params.Rz_pos_diag[k];
        quad_p += diff_p[k] * diff_p[k] * invR;
        const_p += logf(2.0f * M_PI * params.Rz_pos_diag[k]);
    }
    const_p *= -0.5f;

    // Yaw diff
    float diff_y = wrap_to_pi(Xi[IDX_YAW] - z[IDX_YAW]);
    float invR_y = 1.0f / params.Rz_yaw;
    float quad_y = diff_y * diff_y * invR_y;
    float const_y = -0.5f * logf(2.0f * M_PI * params.Rz_yaw);

    out_loglik[i] = (const_p - 0.5f * quad_p) + (const_y - 0.5f * quad_y);
}

// KF update kernel
// - dt: timestep
// - z:  obs vector, layout as state
// - have_yaw_obs, y_obs, R_obs_yawr: for strong yaw-rate from obs yaw
// - have_geom_obs, y_geom[3]: direct measurement of [r1,r2,h] (same for all particles)
__global__ void kf_update_kernel(
    RBPFDevice dev,
    RBPFParams params,
    float dt,
    const float *z,
    bool have_yaw_obs,
    float y_obs,
    float R_obs_yawr,
    bool have_geom_obs,
    float y_geom0, float y_geom1, float y_geom2)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= dev.N) return;
    if (dt <= 0.0f) return;

    float *Xi    = &dev.X[i * D];
    float *mean  = &dev.kf_mean[i * KF_D];
    float *Pvel  = &dev.P_vel_diag[i * 4];
    float *Pgeom = &dev.P_geom_diag[i * 3];

    // ---------- (A) Velocity from finite-diff pos ----------
    float y_vel[3];
    float prev_x = dev.prev_pos[i * 3 + 0];
    float prev_y = dev.prev_pos[i * 3 + 1];
    float prev_z = dev.prev_pos[i * 3 + 2];
    y_vel[0] = (Xi[IDX_TX] - prev_x) / dt;
    y_vel[1] = (Xi[IDX_TY] - prev_y) / dt;
    y_vel[2] = (Xi[IDX_TZ] - prev_z) / dt;

    // mu_v in KF mean: [0..2]
    for (int k = 0; k < 3; ++k) {
        float mu = mean[k];
        float P  = Pvel[k];
        float R  = params.Ry_vel_diag[k];
        float S  = P + R;
        float K  = (S > 0.0f) ? (P / S) : 0.0f;
        float innov = y_vel[k] - mu;
        mean[k] = mu + K * innov;
        Pvel[k] = (1.0f - K) * P;
    }

    // ---------- (B) Yaw-rate from finite-diff yaw (weak pseudo) ----------
    float prev_yaw = dev.prev_yaw[i];
    float y_pseudo = wrap_to_pi(Xi[IDX_YAW] - prev_yaw) / dt;
    {
        int k = 3; // yaw_rate index in KF
        float mu = mean[k];
        float P  = Pvel[k];
        float R  = params.Ry_yawr * 10.0f; // inflated
        float S  = P + R;
        float K  = (S > 0.0f) ? (P / S) : 0.0f;
        float innov = y_pseudo - mu;
        mean[k] = mu + K * innov;
        Pvel[k] = (1.0f - K) * P;
    }

    // ---------- (C) Stronger yaw-rate from obs yaw (if provided) ----------
    if (have_yaw_obs) {
        int k = 3;
        float mu = mean[k];
        float P  = Pvel[k];
        float R  = R_obs_yawr;
        float S  = P + R;
        float K  = (S > 0.0f) ? (P / S) : 0.0f;
        float innov = y_obs - mu;
        mean[k] = mu + K * innov;
        Pvel[k] = (1.0f - K) * P;
    }

    // ---------- (D) Geometry update r1,r2,h (if provided) ----------
    if (have_geom_obs) {
        for (int k = 0; k < 3; ++k) {
            float mu = mean[4 + k];
            float P  = Pgeom[k];
            float R  = params.Rc_geom_diag[k];
            float S  = P + R;
            float K  = (S > 0.0f) ? (P / S) : 0.0f;
            float yg[3] = { y_geom0, y_geom1, y_geom2 };
            float innov = yg[k] - mu;
            mean[4 + k] = mu + K * innov;
            Pgeom[k] = (1.0f - K) * P;
        }
    }

    // ---------- (E) Write back KF means into PF state ----------
    Xi[IDX_VX] = mean[0];
    Xi[IDX_VY] = mean[1];
    Xi[IDX_VZ] = mean[2];
    Xi[IDX_OMEGA] = mean[3];      // yaw_rate
    Xi[IDX_R1] = mean[4];
    Xi[IDX_R2] = mean[5];
    Xi[IDX_H]  = mean[6];
}

__global__ void mean_kernel(const float *X, int N, float *out_mean) {
    __shared__ float buf[D];
    int tid = threadIdx.x;

    if (tid < D) buf[tid] = 0.0f;
    __syncthreads();

    // simple striped accumulation
    for (int idx = tid; idx < N; idx += blockDim.x) {
        const float *Xi = &X[idx * D];
        for (int k = 0; k < D; ++k) {
            atomicAdd(&buf[k], Xi[k]);
        }
    }

    __syncthreads();
    if (tid < D) {
        out_mean[tid] = buf[tid] / float(N);
    }
}

// ======================= HOST WRAPPER ====================

RBPFPosYawModelGPU::RBPFPosYawModelGPU(int N_, const RBPFParams &p)
    : N(N_), params(p), z_yaw_prev(NAN)
{
    dev.N = N;

    size_t szX       = N * D    * sizeof(float);
    size_t szMean    = N * KF_D * sizeof(float);
    size_t szPvel    = N * 4    * sizeof(float);
    size_t szPgeom   = N * 3    * sizeof(float);
    size_t szPrevPos = N * 3    * sizeof(float);
    size_t szPrevYaw = N        * sizeof(float);
    size_t szRng     = N        * sizeof(curandState);
    size_t szLoglik  = N        * sizeof(float);
    size_t szObs     = D        * sizeof(float);
    size_t szW       = N        * sizeof(float);
    size_t szCDF     = N        * sizeof(float);
    size_t szMeanVec = D        * sizeof(float);

    cudaStreamCreate(&stream);

    cudaMalloc(&dev.X,          szX);
    cudaMalloc(&dev.kf_mean,    szMean);
    cudaMalloc(&dev.P_vel_diag, szPvel);
    cudaMalloc(&dev.P_geom_diag,szPgeom);
    cudaMalloc(&dev.prev_pos,   szPrevPos);
    cudaMalloc(&dev.prev_yaw,   szPrevYaw);
    cudaMalloc(&dev.rng_states, szRng);

    cudaMalloc(&d_loglik, szLoglik);
    cudaMalloc(&d_obs,    szObs);
    cudaMalloc(&d_W,      szW);
    cudaMalloc(&d_cdf,    szCDF);
    cudaMalloc(&d_mean,   szMeanVec);

    // init RNG
    int block = CUDA_BLOCK_SIZE;
    int grid  = (N + block - 1) / block;
    init_rng_kernel<<<grid, block, 0, stream>>>(dev.rng_states, N, 1234ULL);

    // attach/init KF
    kf_attach_kernel<<<grid, block, 0, stream>>>(dev, params);

    // init weights uniform
    float w0 = 1.0f / float(N);
    // simple kernel or thrust; for now assume kernel exists
    // gpu_set_uniform_weights(d_W, N, stream);

    cudaStreamSynchronize(stream);
}

RBPFPosYawModelGPU::~RBPFPosYawModelGPU() {
    cudaFree(dev.X);
    cudaFree(dev.kf_mean);
    cudaFree(dev.P_vel_diag);
    cudaFree(dev.P_geom_diag);
    cudaFree(dev.prev_pos);
    cudaFree(dev.prev_yaw);
    cudaFree(dev.rng_states);

    cudaFree(d_W);
    cudaFree(d_loglik);
    cudaFree(d_mean);
    cudaFree(d_obs);
    cudaFree(d_cdf);

    cudaStreamDestroy(stream);
}


void RBPFPosYawModelGPU::set_state_from_host(const float *h_X) {
    cudaMemcpyAsync(dev.X, h_X, N * D * sizeof(float),
                    cudaMemcpyHostToDevice, stream);
}

void RBPFPosYawModelGPU::predict_device(float dt) {
    int block = CUDA_BLOCK_SIZE;
    int grid  = (N + block - 1) / block;
    on_before_predict_kernel<<<grid, block, 0, stream>>>(dev);
    predict_kernel<<<grid, block, 0, stream>>>(dev, params, dt);
}

void RBPFPosYawModelGPU::loglik_device() {
    int block = CUDA_BLOCK_SIZE;
    int grid  = (N + block - 1) / block;
    loglik_kernel<<<grid, block, 0, stream>>>(dev, params, d_obs, d_loglik);
}

void RBPFPosYawModelGPU::kf_update_device(
        float dt,
        bool have_yaw_obs, float y_obs, float R_obs_yawr,
        bool have_geom_obs, float g0, float g1, float g2)
{
    int block = CUDA_BLOCK_SIZE;
    int grid  = (N + block - 1) / block;
    kf_update_kernel<<<grid, block, 0, stream>>>(
        dev, params, dt,
        d_obs,
        have_yaw_obs, y_obs, R_obs_yawr,
        have_geom_obs, g0, g1, g2
    );
}

void RBPFPosYawModelGPU::mean_device() {
    int block = D;  // e.g. 32
    int grid  = 1;
    mean_kernel<<<grid, block, 0, stream>>>(dev.X, N, d_mean);
}

// ======================= C API WRAPPERS ====================


// Map RobotState -> obs[15]
static inline void robotStateToObs(const RobotState &meas, float *z) {
    // fill z[0..14] from meas.tvec, meas.vel, meas.acc, meas.yaw, omega, alpha, r1,r2,h
    // TODO: implement based on your RobotState layout
}

RBPFPosYawModelGPU *rbpf_create(int N) {
    return new RBPFPosYawModelGPU(N);
}

void rbpf_destroy(RBPFPosYawModelGPU *pf) {
    delete pf;
}

void rbpf_predict(RBPFPosYawModelGPU *pf, float dt) {
    pf->predict_device(dt);
}

void rbpf_reset_from_meas(RBPFPosYawModelGPU *pf, const RobotState &meas) {
    float X0[D];
    // TODO: build initial state X0 from meas and copy with set_state_from_host
    pf->set_state_from_host(X0);
}

void rbpf_step(RBPFPosYawModelGPU *pf, const RobotState &meas, float dt) {
    // 1) H2D: obs (only transfer this)
    float h_obs[D];
    robotStateToObs(meas, h_obs);
    cudaMemcpyAsync(pf->d_obs, h_obs, D*sizeof(float),
                    cudaMemcpyHostToDevice, pf->stream);

    //int block = CUDA_BLOCK_SIZE;
    //int grid  = (pf->N + block - 1) / block;

    // 2) predict
    pf->predict_device(dt);

    // 3) loglik
    pf->loglik_device();

    // 4) update + normalize weights, resample (device-only; TODO implement)
    gpu_update_and_normalize_weights(pf->d_loglik, pf->d_W, pf->N, pf->stream);
    gpu_resample_particles(pf->dev, pf->d_W, pf->d_cdf, pf->N, pf->stream);
    gpu_set_uniform_weights(pf->d_W, pf->N, pf->stream);

    // 5) KF update – compute y_obs, R_obs on host
    bool  have_yaw_obs = false;
    float y_obs = 0.0f;
    float R_obs = 0.0f;
    float z_yaw = h_obs[IDX_YAW];

    if (!std::isnan(pf->z_yaw_prev)) {
        float dy = std::fmod(z_yaw - pf->z_yaw_prev + M_PI, 2.0f * M_PI);
        if (dy < 0.0f) dy += 2.0f * M_PI;
        dy -= M_PI;
        y_obs = dy / dt;
        float gain = 2.0f;
        R_obs = gain * pf->params.Rz_yaw / (dt * dt);
        have_yaw_obs = true;
    }
    pf->z_yaw_prev = z_yaw;

    bool  have_geom = true;
    float g0 = meas.state[12], g1 = meas.state[13], g2 = meas.state[14];

    pf->kf_update_device(dt, have_yaw_obs, y_obs, R_obs, have_geom, g0, g1, g2);
}

RobotState rbpf_get_mean(RBPFPosYawModelGPU *pf) {
    pf->mean_device();

    float host_mean[D];
    cudaMemcpyAsync(host_mean, pf->d_mean, D*sizeof(float),
                    cudaMemcpyDeviceToHost, pf->stream);
    cudaStreamSynchronize(pf->stream);

    RobotState rs{};
    // TODO: map host_mean[0..14] -> rs fields
    return rs;
}

// =================== WEIGHT UPDATE / RESAMPLE HELPERS ===================

// Atomic max for float using CAS (device-only, no host transfers)
__device__ float atomicMaxFloat(float* addr, float value) {
    int* addr_i = reinterpret_cast<int*>(addr);
    int old = *addr_i, assumed;

    do {
        assumed = old;
        float old_val = __int_as_float(assumed);
        float max_val = fmaxf(value, old_val);
        int max_i = __float_as_int(max_val);
        old = atomicCAS(addr_i, assumed, max_i);
    } while (assumed != old);

    return __int_as_float(old);
}

// init max/sum scalars on device
__global__ void init_max_sum_kernel(float *d_max, float *d_sum) {
    if (blockIdx.x == 0 && threadIdx.x == 0) {
        *d_max = -1e30f;  // effectively -inf
        *d_sum = 0.0f;
    }
}

// 1) compute log-weights and global max(loglik)
__global__ void compute_logw_and_max_kernel(
        const float *d_loglik,
        float *d_W,
        int N,
        float *d_max)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float lw = d_loglik[i];   // log-likelihood of particle i
    d_W[i] = lw;              // temporarily store log-w in W

    // update global max logw
    atomicMaxFloat(d_max, lw);
}

// 2) exponentiate and accumulate sum of weights
__global__ void exp_and_sum_kernel(
        float *d_W,
        int N,
        const float *d_max,
        float *d_sum)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float lw = d_W[i];
    float w = __expf(lw - *d_max);  // exp(logw - max)
    d_W[i] = w;

    atomicAdd(d_sum, w);
}

// 3) normalize weights by sum
__global__ void normalize_weights_kernel(
        float *d_W,
        int N,
        const float *d_sum)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float s = *d_sum;
    if (s > 0.0f) {
        d_W[i] /= s;
    }
}

// set all weights to uniform 1/N
__global__ void set_uniform_weights_kernel(float *d_W, int N, float w0) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;
    d_W[i] = w0;
}

// build CDF (prefix sum) of weights in a single thread (simple & safe)
__global__ void build_cdf_kernel(const float *d_W, float *d_cdf, int N) {
    if (blockIdx.x == 0 && threadIdx.x == 0) {
        float c = 0.0f;
        for (int i = 0; i < N; ++i) {
            c += d_W[i];
            d_cdf[i] = c;
        }
        // small numerical fix: ensure last element is exactly 1.0
        if (N > 0) d_cdf[N - 1] = 1.0f;
    }
}

// resample particles (systematic, deterministic u0 = 0.5/N)
__global__ void resample_kernel(
        RBPFDevice dev,
        const float *d_cdf,
        float *X_new,
        float *kf_new,
        float *Pvel_new,
        float *Pgeom_new,
        int N)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    // systematic resampling with fixed offset (no RNG needed)
    float u0 = 0.5f / float(N);
    float u  = u0 + float(i) / float(N);

    // binary search in CDF
    int lo = 0, hi = N - 1;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (d_cdf[mid] >= u) hi = mid;
        else                  lo = mid + 1;
    }
    int idx = lo;  // chosen ancestor index

    // copy PF state
    const float *X_src = &dev.X[idx * D];
    float       *X_dst = &X_new[i * D];
    for (int k = 0; k < D; ++k) {
        X_dst[k] = X_src[k];
    }

    // copy KF mean
    const float *kf_src = &dev.kf_mean[idx * KF_D];
    float       *kf_dst = &kf_new[i * KF_D];
    for (int k = 0; k < KF_D; ++k) {
        kf_dst[k] = kf_src[k];
    }

    // copy KF cov diagonals
    const float *Pvel_src  = &dev.P_vel_diag[idx * 4];
    float       *Pvel_dst  = &Pvel_new[i * 4];
    for (int k = 0; k < 4; ++k) {
        Pvel_dst[k] = Pvel_src[k];
    }

    const float *Pgeom_src = &dev.P_geom_diag[idx * 3];
    float       *Pgeom_dst = &Pgeom_new[i * 3];
    for (int k = 0; k < 3; ++k) {
        Pgeom_dst[k] = Pgeom_src[k];
    }
}
void gpu_update_and_normalize_weights(
        const float *d_loglik,
        float *d_W,
        int N,
        cudaStream_t stream)
{
    // scratch scalars on device
    float *d_max = nullptr;
    float *d_sum = nullptr;
    cudaMalloc(&d_max, sizeof(float));
    cudaMalloc(&d_sum, sizeof(float));

    init_max_sum_kernel<<<1, 1, 0, stream>>>(d_max, d_sum);

    int block = CUDA_BLOCK_SIZE;
    int grid  = (N + block - 1) / block;

    // 1) compute logw and global max logw
    compute_logw_and_max_kernel<<<grid, block, 0, stream>>>(
        d_loglik, d_W, N, d_max);

    // 2) exponentiate and accumulate sum
    exp_and_sum_kernel<<<grid, block, 0, stream>>>(
        d_W, N, d_max, d_sum);

    // 3) normalize weights
    normalize_weights_kernel<<<grid, block, 0, stream>>>(
        d_W, N, d_sum);

    cudaFree(d_max);
    cudaFree(d_sum);
}
void gpu_set_uniform_weights(float *d_W, int N, cudaStream_t stream) {
    int block = CUDA_BLOCK_SIZE;
    int grid  = (N + block - 1) / block;
    float w0 = 1.0f / float(N);
    set_uniform_weights_kernel<<<grid, block, 0, stream>>>(d_W, N, w0);
}
void gpu_resample_particles(
        RBPFDevice dev,
        float *d_W,
        float *d_cdf,
        int N,
        cudaStream_t stream)
{
    // 1) build CDF of weights (device-only)
    build_cdf_kernel<<<1, 1, 0, stream>>>(d_W, d_cdf, N);

    // 2) allocate temporary arrays for resampled state + KF stuff
    size_t szX     = size_t(N) * D    * sizeof(float);
    size_t szMean  = size_t(N) * KF_D * sizeof(float);
    size_t szPvel  = size_t(N) * 4    * sizeof(float);
    size_t szPgeom = size_t(N) * 3    * sizeof(float);

    float *X_new     = nullptr;
    float *kf_new    = nullptr;
    float *Pvel_new  = nullptr;
    float *Pgeom_new = nullptr;

    cudaMalloc(&X_new,     szX);
    cudaMalloc(&kf_new,    szMean);
    cudaMalloc(&Pvel_new,  szPvel);
    cudaMalloc(&Pgeom_new, szPgeom);

    int block = CUDA_BLOCK_SIZE;
    int grid  = (N + block - 1) / block;

    // 3) resample into new arrays (systematic with fixed u0)
    resample_kernel<<<grid, block, 0, stream>>>(
        dev, d_cdf, X_new, kf_new, Pvel_new, Pgeom_new, N);

    // 4) overwrite original device arrays with resampled ones
    cudaMemcpyAsync(dev.X,          X_new,     szX,
                    cudaMemcpyDeviceToDevice, stream);
    cudaMemcpyAsync(dev.kf_mean,    kf_new,    szMean,
                    cudaMemcpyDeviceToDevice, stream);
    cudaMemcpyAsync(dev.P_vel_diag, Pvel_new,  szPvel,
                    cudaMemcpyDeviceToDevice, stream);
    cudaMemcpyAsync(dev.P_geom_diag,Pgeom_new, szPgeom,
                    cudaMemcpyDeviceToDevice, stream);

    // 5) cleanup
    cudaFree(X_new);
    cudaFree(kf_new);
    cudaFree(Pvel_new);
    cudaFree(Pgeom_new);
}



