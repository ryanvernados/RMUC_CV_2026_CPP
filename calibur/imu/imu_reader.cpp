// calibur/imu/imu_reader.cpp
#include "imu_reader.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <vector>

// ---- Protocol constants ----
static constexpr uint8_t HDR0        = 0x59;
static constexpr uint8_t HDR1        = 0x53;
static constexpr uint8_t MAX_PAYLOAD = 255;

static constexpr uint8_t ID_TEMP          = 0x01;
static constexpr uint8_t ID_ACCEL         = 0x10;
static constexpr uint8_t ID_GYRO          = 0x20;
static constexpr uint8_t ID_MAGN_NORM     = 0x30;
static constexpr uint8_t ID_MAGN_RAW      = 0x31;
static constexpr uint8_t ID_EULER         = 0x40;
static constexpr uint8_t ID_QUAT          = 0x41;
static constexpr uint8_t ID_TS_SAMPLE     = 0x51;
static constexpr uint8_t ID_TS_DATAREADY  = 0x52;
static constexpr uint8_t ID_POS           = 0x68;
static constexpr uint8_t ID_VEL           = 0x70;
static constexpr uint8_t ID_FUSION_STATE  = 0x80;

// ---- Little-endian helpers ----
static inline uint16_t read_u16_le(const uint8_t *p) {
    return static_cast<uint16_t>(p[0]) |
           static_cast<uint16_t>(p[1]) << 8;
}

static inline int32_t read_i32_le(const uint8_t *p) {
    uint32_t v = static_cast<uint32_t>(p[0])
               | (static_cast<uint32_t>(p[1]) << 8)
               | (static_cast<uint32_t>(p[2]) << 16)
               | (static_cast<uint32_t>(p[3]) << 24);
    return static_cast<int32_t>(v);
}

static inline uint32_t read_u32_le(const uint8_t *p) {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

// ---- checksum (Fletcher-like) ----
static void checksum(const std::vector<uint8_t> &data,
                     uint8_t &ck1, uint8_t &ck2)
{
    ck1 = 0;
    ck2 = 0;
    for (uint8_t b : data) {
        ck1 = static_cast<uint8_t>((ck1 + b) & 0xFF);
        ck2 = static_cast<uint8_t>((ck2 + ck1) & 0xFF);
    }
}

// ---- Parse a single TLV block into IMUData ----
static void parse_block(uint8_t data_id,
                        const uint8_t *payload,
                        uint8_t length,
                        IMUData &out)
{
    switch (data_id) {
    case ID_TEMP:
        if (length == 2) {
            int16_t raw = static_cast<int16_t>(read_u16_le(payload));
            out.temp_c = raw * 0.01f;
            out.has_temp = true;
        }
        break;

    case ID_ACCEL:
        if (length == 12) {
            int32_t ax = read_i32_le(payload + 0);
            int32_t ay = read_i32_le(payload + 4);
            int32_t az = read_i32_le(payload + 8);
            out.accel_mps2[0] = ax * 1e-6f;
            out.accel_mps2[1] = ay * 1e-6f;
            out.accel_mps2[2] = az * 1e-6f;
            out.has_accel = true;
        }
        break;

    case ID_GYRO:
        if (length == 12) {
            int32_t wx = read_i32_le(payload + 0);
            int32_t wy = read_i32_le(payload + 4);
            int32_t wz = read_i32_le(payload + 8);
            out.gyro_dps[0] = wx * 1e-6f;
            out.gyro_dps[1] = wy * 1e-6f;
            out.gyro_dps[2] = wz * 1e-6f;
            out.has_gyro = true;
        }
        break;

    case ID_MAGN_NORM:
        if (length == 12) {
            int32_t mx = read_i32_le(payload + 0);
            int32_t my = read_i32_le(payload + 4);
            int32_t mz = read_i32_le(payload + 8);
            out.mag_norm[0] = mx * 1e-6f;
            out.mag_norm[1] = my * 1e-6f;
            out.mag_norm[2] = mz * 1e-6f;
            out.has_mag_norm = true;
        }
        break;

    case ID_MAGN_RAW:
        if (length == 12) {
            int32_t mx = read_i32_le(payload + 0);
            int32_t my = read_i32_le(payload + 4);
            int32_t mz = read_i32_le(payload + 8);
            out.mag_mg[0] = mx * 1e-3f;
            out.mag_mg[1] = my * 1e-3f;
            out.mag_mg[2] = mz * 1e-3f;
            out.has_mag_mg = true;
        }
        break;

    case ID_EULER:
        if (length == 12) {
            int32_t pitch = read_i32_le(payload + 0);
            int32_t roll  = read_i32_le(payload + 4);
            int32_t yaw   = read_i32_le(payload + 8);
            out.euler_deg[0] = pitch * 1e-6f;
            out.euler_deg[1] = roll  * 1e-6f;
            out.euler_deg[2] = yaw   * 1e-6f;
            out.has_euler = true;
        }
        break;

    case ID_QUAT:
        if (length == 16) {
            int32_t q0 = read_i32_le(payload + 0);
            int32_t q1 = read_i32_le(payload + 4);
            int32_t q2 = read_i32_le(payload + 8);
            int32_t q3 = read_i32_le(payload + 12);
            out.quat[0] = q0 * 1e-6f;
            out.quat[1] = q1 * 1e-6f;
            out.quat[2] = q2 * 1e-6f;
            out.quat[3] = q3 * 1e-6f;
            out.has_quat = true;
        }
        break;

    case ID_TS_SAMPLE:
        if (length == 4) {
            out.ts_sample_us = read_u32_le(payload);
            out.has_ts_sample = true;
        }
        break;

    case ID_TS_DATAREADY:
        if (length == 4) {
            out.ts_dataready_us = read_u32_le(payload);
            out.has_ts_dataready = true;
        }
        break;

    case ID_FUSION_STATE:
        if (length == 1) {
            uint8_t s = payload[0];
            out.fusion_state = s & 0x0F;
            out.gnss_state   = (s >> 4) & 0x0F;
            out.has_fusion = true;
        }
        break;

    default:
        // ignore others for now
        break;
    }
}

// ---- Parse full payload (list of TLVs) ----
static void parse_payload(const std::vector<uint8_t> &buf,
                          IMUData &out)
{
    size_t i = 0;
    while (i + 2 <= buf.size()) {
        uint8_t id  = buf[i++];
        uint8_t len = buf[i++];
        if (i + len > buf.size()) break;
        parse_block(id, &buf[i], len, out);
        i += len;
    }
}

// =============================================================
// IMUReader implementation
// =============================================================

IMUReader::IMUReader(const std::string &device, int baud, float timeout_s)
    : device_(device),
      baud_(baud),
      timeout_s_(timeout_s),
      fd_(-1),
      running_(false),
      latest_(),
      has_data_(false)
{
}

IMUReader::~IMUReader() {
    stop();
}

int IMUReader::open_serial() {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::perror("open serial");
        return -1;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::perror("tcgetattr");
        close_serial();
        return -1;
    }

    cfmakeraw(&tio);

    speed_t speed = B460800; // you can switch on baud_ if needed
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cc[VTIME] = static_cast<cc_t>(timeout_s_ * 10.0f);
    tio.c_cc[VMIN]  = 0;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::perror("tcsetattr");
        close_serial();
        return -1;
    }

    return fd_;
}

void IMUReader::close_serial() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void IMUReader::start() {
    if (running_.load()) return;
    if (open_serial() < 0) {
        std::cerr << "[IMUReader] Failed to open " << device_ << "\n";
        return;
    }
    running_.store(true);
    thread_ = std::thread(&IMUReader::worker, this);
}

void IMUReader::stop() {
    running_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
    close_serial();
}

bool IMUReader::get_latest(IMUData &out) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_data_) return false;
    out = latest_;
    return true;
}

bool IMUReader::read_exact(uint8_t *buf, size_t n) {
    size_t total = 0;
    while (total < n) {
        ssize_t ret = ::read(fd_, buf + total, n - total);
        if (ret < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            std::perror("read");
            return false;
        } else if (ret == 0) {
            return false;
        }
        total += static_cast<size_t>(ret);
    }
    return true;
}

bool IMUReader::read_frame(IMUData &out) {
    // 1) sync on header
    uint8_t prev = 0;
    uint8_t b    = 0;
    bool synced  = false;

    while (running_.load()) {
        ssize_t ret = ::read(fd_, &b, 1);
        if (ret == 1) {
            if (prev == HDR0 && b == HDR1) {
                synced = true;
                break;
            }
            prev = b;
        } else if (ret < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            std::perror("read");
            return false;
        } else { // ret == 0
            return false;
        }
    }
    if (!synced) return false;

    // 2) read seq(2) + len(1)
    uint8_t head[3];
    if (!read_exact(head, 3)) return false;

    uint16_t seq    = read_u16_le(head);
    uint8_t  length = head[2];

    if (length > MAX_PAYLOAD) {
        std::vector<uint8_t> dump(length + 2);
        (void)::read(fd_, dump.data(), dump.size());
        return false;
    }

    // 3) payload
    std::vector<uint8_t> payload(length);
    if (!read_exact(payload.data(), length)) return false;

    // 4) checksum
    uint8_t ck[2];
    if (!read_exact(ck, 2)) return false;

    std::vector<uint8_t> chk_data;
    chk_data.reserve(3 + length);
    chk_data.push_back(static_cast<uint8_t>(seq & 0xFF));
    chk_data.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
    chk_data.push_back(length);
    chk_data.insert(chk_data.end(), payload.begin(), payload.end());

    uint8_t ck1, ck2;
    checksum(chk_data, ck1, ck2);
    if (ck1 != ck[0] || ck2 != ck[1]) {
        return false;
    }

    IMUData data;
    data.seq = seq;
    parse_payload(payload, data);
    out = data;
    return true;
}

void IMUReader::worker() {
    while (running_.load()) {
        IMUData d;
        if (!read_frame(d)) {
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_ = d;
            has_data_ = true;
        }
    }
}
