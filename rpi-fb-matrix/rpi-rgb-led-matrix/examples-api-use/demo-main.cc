// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
//
// This code is public domain
// (but note, once linked against the led-matrix library, this is
// covered by the GPL v2)
//
// This is a grab-bag of various demos and not very readable.
#include "led-matrix.h"
#include "threaded-canvas-manipulator.h"
#include "transformer.h"
#include "graphics.h"

#include <stdio.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <cstring>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include "linux/i2c.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <ostream>
#include <thread>
#include <vector>

using std::min;
using std::max;

#define TERM_ERR  "\033[1;31m"
#define TERM_NORM "\033[0m"

using namespace rgb_matrix;

#define MODE_READ 0
#define MODE_WRITE 1

#define MAX_LEN 32

char wbuf[MAX_LEN];

char buf[MAX_LEN];
int i;
uint8_t data;

volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
    interrupt_received = true;
}

static int i2c_bus = 1;
static int device_address = 0x68;

uint8_t reg = 0x68;
uint8_t chipID[1];

int i2c_init(int i2c_bus_num, int slave_address) {
    int file_i2c;
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", i2c_bus_num);

    if ((file_i2c = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        return -1;
    }

    if (ioctl(file_i2c, I2C_SLAVE, slave_address) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(file_i2c);
        return -1;
    }

    return file_i2c;
}
int file_descriptor;

void writeBytes(unsigned char* data_buffer, unsigned int length) {
    if (write(file_descriptor, data_buffer, length) != (ssize_t)length) {
        perror("Failed to write to the i2c bus");
    }
}

void readBytes(unsigned char* read_buffer, unsigned int length) {
    if (read(file_descriptor, read_buffer, length) != (ssize_t)length) {
        perror("Failed to read from the i2c bus");
    }
}

void readRegBytes(unsigned char reg_addr, unsigned char* read_buffer, unsigned int length) {
    if (write(file_descriptor, &reg_addr, 1) != 1) {
        // perror("Failed to write register address for reading");
        return;
    }
    if (read(file_descriptor, read_buffer, length) != (ssize_t)length) {
        perror("Failed to read data after addressing register");
    }
}

#define MPU9265_ACCEL_XOUT_H    0x3B
#define MPU9265_ACCEL_XOUT_L    0x3C
#define MPU9265_ACCEL_YOUT_H    0x3D
#define MPU9265_ACCEL_YOUT_L    0x3E
#define MPU9265_ACCEL_ZOUT_H    0x3F
#define MPU9265_ACCEL_ZOUT_L    0x40
#define MPU9265_PWR_MGMT_1      0x6B

#define MPU9265_GYRO_XOUT_H     0x43
#define MPU9265_GYRO_XOUT_L     0x44
#define MPU9265_GYRO_YOUT_H     0x45
#define MPU9265_GYRO_YOUT_L     0x46
#define MPU9265_GYRO_ZOUT_H     0x47
#define MPU9265_GYRO_ZOUT_L     0x48

#define GYRO_SENSITIVITY_250DPS 131.0

double offsetX_ms2 = 0, offsetY_ms2 = 0, offsetZ_ms2 = 0;

#define ACCEL_SENSITIVITY_2G 16384.0

double predictionTime = 0.025;
double LATENCY_SEC = 0.00001;

#define G_VALUE 9.81

struct Vector3 {
    double x = NAN;
    double y = NAN;
    double z = NAN;

    bool is_initialized() const {
        return !std::isnan(x);
    }

    double magnitude() {
        return x * x + y * y + z * z;
    }
};

struct Vector2 {
    double x = NAN;
    double y = NAN;

    bool is_initialized() const {
        return !std::isnan(x);
    }
};

Vector3 lerp(Vector3 a, Vector3 b, float t) {
    t = std::max(0.0f, std::min(1.0f, t));
    return {
      a.x + (b.x - a.x) * t,
      a.y + (b.y - a.y) * t,
      a.z + (b.z - a.z) * t
    };
}

void normalise(float x, float y, float z, float* nx, float* ny, float* nz) {
    float invNorm = 1.0f / sqrt(x * x + y * y + z * z);
    *nx = x * invNorm;
    *ny = y * invNorm;
    *nz = z * invNorm;
}

const float SMOOTHING_FACTOR = 0.1f;

Vector3 smoothed_acceleration = { 0.0, 0.0, 0.0 };
Vector3 gravity_vector = { 0.0, 0.0, 0.0 };

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

#define twoKp (2.0f * 0.05f)
float currentKp = 2.0f * 10.0f;
#define G_ACCEL_SENSITIVITY (G_VALUE / ACCEL_SENSITIVITY_2G);
#define DEG_TO_RAD (M_PI / 180.0f);
#define PI_GYRO_SENSITIVITY (M_PI / 180.0f / GYRO_SENSITIVITY_250DPS);

float invSqrt(float x) {
    const float xhalf = 0.5f * x;
    int i = *(int*)(&x);
    i = 0x5f3759df - (i >> 1);
    x = *(float*)(&i);
    x = x * (1.5f - xhalf * x * x);
    return x;
}

void MahonyAHRSupdateIMU(float gx_f, float gy_f, float gz_f, float ax_f, float ay_f, float az_f, double dt) {
    float recipNorm;

    if (!((ax_f == 0.0f) && (ay_f == 0.0f) && (az_f == 0.0f))) {

        recipNorm = invSqrt(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
        ax_f *= recipNorm;
        ay_f *= recipNorm;
        az_f *= recipNorm;

        const float halfvx = q1 * q3 - q0 * q2;
        const float halfvy = q0 * q1 + q2 * q3;
        const float halfvz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        const float halfex = (ay_f * halfvz - az_f * halfvy);
        const float halfey = (az_f * halfvx - ax_f * halfvz);
        const float halfez = (ax_f * halfvy - ay_f * halfvx);

        gx_f += currentKp * halfex;
        gy_f += currentKp * halfey;
        gz_f += currentKp * halfez;
    }

    gx_f *= 0.5f * static_cast<float>(dt);
    gy_f *= 0.5f * static_cast<float>(dt);
    gz_f *= 0.5f * static_cast<float>(dt);
    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx_f - qc * gy_f - q3 * gz_f);
    q1 += (qa * gx_f + qc * gz_f - q3 * gy_f);
    q2 += (qa * gy_f - qb * gz_f + q3 * gx_f);
    q3 += (qa * gz_f + qb * gy_f - qc * gx_f);

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

struct EulerAngles {
    double z;
    double x;
    double y;
};

EulerAngles QuatToEuler(double q0, double q1, double q2, double q3) {
    EulerAngles angles;

    double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    angles.z = std::atan2(sinr_cosp, cosr_cosp) * (180.0 / M_PI);

    double sinp = 2 * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1)
        angles.x = std::copysign(M_PI / 2, sinp) * (180.0 / M_PI);
    else
        angles.x = std::asin(sinp) * (180.0 / M_PI);

    double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    angles.y = std::atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);

    return angles;
}

class ObjModelRenderer : public ThreadedCanvasManipulator {
public:
    ObjModelRenderer(Canvas* m) : ThreadedCanvasManipulator(m) {}

    uint8_t scale_col(int val, int lo, int hi) {
        if (val < lo) return 0;
        if (val > hi) return 255;
        return 255 * (val - lo) / (hi - lo);
    }

    std::mutex config_mutex;

    struct Edge { int v1, v2; };
    std::vector<Vector3> modelVertices;
    std::vector<Edge> modelEdges;
    Vector3 minP;
    Vector3 maxP;

    std::vector<Vector2> lastDrawnPixels;

    EulerAngles offset_angles = { 0, 0, 0 };

    const int cent_x = canvas()->width() / 2;
    const int cent_y = canvas()->height() / 2;

    const int width = canvas()->width() - 1;
    const int height = canvas()->height() - 1;

    std::string last_model_id = "";
    float last_scale = -1.0f;
    Vector3 last_rot_speed = { 0, 0, 0 };
    Vector3 last_static_offset = { 0, 0, 0 };
    std::string model_path = "/opt/volumetrix/rpi-fb-matrix/rpi-rgb-led-matrix/examples-api-use/models/model.obj";
    std::string base_path = "/opt/volumetrix/rpi-fb-matrix/rpi-rgb-led-matrix/examples-api-use/";

    std::string reset_path = base_path + "reset.txt";

    std::string current_model_id;
    Vector3 current_static_offset;
    Vector3 current_rot_speed;
    float current_scale;

    std::future<void> configTask;

    bool resetMinMax = true;

    double angle_multiplier = 3;

    void Run() {

        std::ifstream video_stream;
        std::vector<uint8_t> frame_buf(width * height);
        auto next_frame_time = std::chrono::high_resolution_clock::now();

        static auto initial_time = std::chrono::high_resolution_clock::now();
        static auto last_time = std::chrono::high_resolution_clock::now();

        auto fps_timer = std::chrono::high_resolution_clock::now();
        int frames = 0;

        RequestConfig();

        while (running() && !interrupt_received) {
            frames++;
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - fps_timer).count() >= 1) {
                std::cout << "\r[REFRESH RATE]: " << frames << " FPS    " << std::flush;
                frames = 0;
                fps_timer = now;
            }

            if (!configTask.valid()) {
                configTask = std::async(
                    std::launch::async,
                    &ObjModelRenderer::RequestConfig,
                    this
                );
            }
            else if (configTask.valid() && configTask.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                configTask.get();
                configTask = std::future<void>();
            }

            bool is_bin_video = current_model_id.find(".bin") != std::string::npos;
            bool is_3d_video = current_model_id.find(".bin3d") != std::string::npos;

            if (is_bin_video) {
                constexpr size_t bytes_per_pixel = 3;
                constexpr int vid_w = 64;
                constexpr int vid_h = 32;
                constexpr size_t color_frame_size = vid_w * vid_h * bytes_per_pixel;

                if (!video_stream.is_open() || current_model_id != last_model_id) {
                    if (video_stream.is_open()) video_stream.close();

                    std::string filename = current_model_id;
                    size_t pos = filename.find(".bin3d");

                    if (pos != std::string::npos) {
                        filename.replace(pos, 6, ".bin");
                    }

                    video_stream.open(base_path + "models/" + filename, std::ios::binary);
                    video_stream.open(base_path + "models/" + filename, std::ios::binary);

                    frame_buf.resize(color_frame_size);
                    next_frame_time = std::chrono::high_resolution_clock::now();
                    last_time = now;
                    last_model_id = current_model_id;
                }

                if (is_3d_video) {
                    auto current_time = std::chrono::high_resolution_clock::now();
                    double dt = std::chrono::duration<double>(current_time - last_time).count();
                    last_time = current_time;

                    unsigned char buf[6], gyro_buf[6];
                    readRegBytes(MPU9265_ACCEL_XOUT_H, buf, 6);
                    readRegBytes(MPU9265_GYRO_XOUT_H, gyro_buf, 6);

                    Vector3 raw_accel_ms2 = {
                      ((double)((int16_t)(buf[0] << 8 | buf[1])) / ACCEL_SENSITIVITY_2G) * G_VALUE,
                      ((double)((int16_t)(buf[2] << 8 | buf[3])) / ACCEL_SENSITIVITY_2G) * G_VALUE,
                      ((double)((int16_t)(buf[4] << 8 | buf[5])) / ACCEL_SENSITIVITY_2G) * G_VALUE
                    };

                    Vector3 raw_gyro = {
                      ((float)((int16_t)(gyro_buf[0] << 8 | gyro_buf[1])) / GYRO_SENSITIVITY_250DPS) * (float)(M_PI / 180.0f),
                      ((float)((int16_t)(gyro_buf[2] << 8 | gyro_buf[3])) / GYRO_SENSITIVITY_250DPS) * (float)(M_PI / 180.0f),
                      ((float)((int16_t)(gyro_buf[4] << 8 | gyro_buf[5])) / GYRO_SENSITIVITY_250DPS) * (float)(M_PI / 180.0f)
                    };

                    MahonyAHRSupdateIMU(raw_gyro.x, raw_gyro.y, raw_gyro.z, raw_accel_ms2.x, raw_accel_ms2.y, raw_accel_ms2.z, dt);

                    if (now >= next_frame_time) {
                        if (!video_stream.read(reinterpret_cast<char*>(frame_buf.data()), color_frame_size)) {
                            video_stream.clear();
                            video_stream.seekg(0, std::ios::beg);
                            if (!video_stream.read(reinterpret_cast<char*>(frame_buf.data()), color_frame_size)) continue;
                        }

                        canvas()->Fill(0, 0, 0);

                        EulerAngles g_angs = QuatToEuler(q0, q1, q2, q3);
                        float pitch_rad = -(g_angs.x + current_static_offset.x) * (M_PI / 180.0f);
                        float yaw_rad = -(g_angs.y + current_static_offset.y) * (M_PI / 180.0f);
                        float roll_rad = -(g_angs.z + current_static_offset.z) * (M_PI / 180.0f);

                        float cy = cos(yaw_rad), sy = sin(yaw_rad);
                        float cp = cos(pitch_rad), sp = sin(pitch_rad);
                        float cr = cos(roll_rad), sr = sin(roll_rad);

                        float centerX = canvas()->width() / 2.0f;
                        float centerY = canvas()->height() / 2.0f;

                        for (int y = 0; y < vid_h; ++y) {
                            for (int x = 0; x < vid_w; ++x) {
                                int offset = (y * vid_w + x) * 3;

                                float lx = x - (vid_w / 2.0f);
                                float ly = (vid_h / 2.0f) - y;
                                float lz = 0;

                                float x1 = lx * cy + lz * sy;
                                float z1 = -lx * sy + lz * cy;
                                float y2 = ly * cp - z1 * sp;
                                float z2 = ly * sp + z1 * cp;
                                float rx = x1 * cr - y2 * sr;
                                float ry = x1 * sr + y2 * cr;

                                int tx = (int)std::round(centerX + rx);
                                int ty = (int)std::round(centerY + ry);

                                if (tx >= 0 && tx < canvas()->width() && ty >= 0 && ty < canvas()->height()) {
                                    canvas()->SetPixel(tx, ty, frame_buf[offset], frame_buf[offset + 1], frame_buf[offset + 2]);
                                }
                            }
                        }
                        next_frame_time = now + std::chrono::milliseconds(33);
                    }
                }
                else {
                    if (now >= next_frame_time) {
                        if (!video_stream.read(reinterpret_cast<char*>(frame_buf.data()), color_frame_size)) {
                            video_stream.clear();
                            video_stream.seekg(0, std::ios::beg);
                            if (!video_stream.read(reinterpret_cast<char*>(frame_buf.data()), color_frame_size)) return;
                        }

                        canvas()->Fill(0, 0, 0);
                        for (int y = 0; y < vid_h; ++y) {
                            for (int x = 0; x < vid_w; ++x) {
                                int offset = (y * vid_w + x) * 3;

                                int targetX = y;
                                int targetY = x;

                                if (targetX < canvas()->width() && targetY < canvas()->height()) {
                                    canvas()->SetPixel(targetX, targetY, frame_buf[offset], frame_buf[offset + 1], frame_buf[offset + 2]);
                                }
                            }
                        }
                        next_frame_time = now + std::chrono::milliseconds(33);
                    }
                }
            }
            else {
                if (video_stream.is_open()) {
                    video_stream.close();
                }

                auto current_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_time);
                double dt = time_span.count();
                last_time = current_time;

                unsigned char buf[6];
                unsigned char gyro_buf[6];

                readRegBytes(MPU9265_ACCEL_XOUT_H, buf, 6);
                int16_t rawAX = (buf[0] << 8) | buf[1];
                int16_t rawAY = (buf[2] << 8) | buf[3];
                int16_t rawAZ = (buf[4] << 8) | buf[5];

                Vector3 raw_accel
                    = {
                             ((double)rawAX / ACCEL_SENSITIVITY_2G) * G_VALUE,
                             ((double)rawAY / ACCEL_SENSITIVITY_2G) * G_VALUE,
                             ((double)rawAZ / ACCEL_SENSITIVITY_2G) * G_VALUE
                };

                readRegBytes(MPU9265_GYRO_XOUT_H, gyro_buf, 6);
                int16_t rawGX = (gyro_buf[0] << 8) | gyro_buf[1];
                int16_t rawGY = (gyro_buf[2] << 8) | gyro_buf[3];
                int16_t rawGZ = (gyro_buf[4] << 8) | gyro_buf[5];

                Vector3 raw_gyro = {
                  ((float)rawGX / GYRO_SENSITIVITY_250DPS) * (M_PI / 180.0f),
                  ((float)rawGY / GYRO_SENSITIVITY_250DPS) * (M_PI / 180.0f),
                  ((float)rawGZ / GYRO_SENSITIVITY_250DPS) * (M_PI / 180.0f),
                };

                std::cout << "Gyro: " << raw_gyro.magnitude() << ", " << raw_gyro.x << ", " << raw_gyro.y << ", " << raw_gyro.z << std::endl;

                const float GYRO_DEADZONE = 0.05f;

                if (std::abs(raw_gyro.x) < GYRO_DEADZONE) raw_gyro.x = 0;
                if (std::abs(raw_gyro.y) < GYRO_DEADZONE) raw_gyro.y = 0;
                if (std::abs(raw_gyro.z) < GYRO_DEADZONE) raw_gyro.z = 0;

                MahonyAHRSupdateIMU(raw_gyro.x, raw_gyro.y, raw_gyro.z, raw_accel.x, raw_accel.y, raw_accel.z, dt);

                if (currentKp > twoKp) {
                    float diff = currentKp - twoKp;

                    currentKp -= diff * 0.05f;

                    if (currentKp < twoKp) currentKp = twoKp;
                }

                float gx = raw_gyro.x;
                float gy = raw_gyro.y;
                float gz = raw_gyro.z;

                float pq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * predictionTime);
                float pq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * (0.5f * predictionTime);
                float pq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * (0.5f * predictionTime);
                float pq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * (0.5f * predictionTime);

                float pNorm = invSqrt(pq0 * pq0 + pq1 * pq1 + pq2 * pq2 + pq3 * pq3);
                pq0 *= pNorm; pq1 *= pNorm; pq2 *= pNorm; pq3 *= pNorm;

                EulerAngles gyro_angles = QuatToEuler(pq0, pq1, pq2, pq3);

                float invA = invSqrt(raw_accel.x * raw_accel.x + raw_accel.y * raw_accel.y + raw_accel.z * raw_accel.z);
                float upX = raw_accel.x * invA;
                float upY = raw_accel.y * invA;
                float upZ = raw_accel.z * invA;

                float world_yaw_rate = (raw_gyro.x * upX) + (raw_gyro.y * upY) + (raw_gyro.z * upZ);

                float prediction_angle = world_yaw_rate * 0.05 * angle_multiplier;

                LATENCY_SEC = std::chrono::duration_cast<std::chrono::milliseconds>(now - initial_time).count() / 1000.0f / 100.0f - 1.0f;

                offset_angles.x += current_rot_speed.x * dt;
                offset_angles.y += (current_rot_speed.y + (prediction_angle * (180.0f / M_PI))) * dt;
                offset_angles.z += current_rot_speed.z * dt;

                EulerAngles current_angles = {
                  gyro_angles.z + offset_angles.z + current_static_offset.z,
                  gyro_angles.x + offset_angles.x + current_static_offset.x,
                  gyro_angles.y + offset_angles.y + current_static_offset.y
                };

                // std::cout << "Angles: " << gyro_angles.x << ", " << gyro_angles.y << ", " << gyro_angles.z << std::endl;

                if (resetMinMax) {
                    minP = modelVertices[0], maxP = modelVertices[0];
                    for (const auto& v : modelVertices) {
                        minP.x = std::min(minP.x, v.x); maxP.x = std::max(maxP.x, v.x);
                        minP.y = std::min(minP.y, v.y); maxP.y = std::max(maxP.y, v.y);
                        minP.z = std::min(minP.z, v.z); maxP.z = std::max(maxP.z, v.z);
                    }
                    resetMinMax = false;
                }

                // std::cout << "LATENCY_SEC: " << LATENCY_SEC << std::endl;

                // canvas()->Fill(0,0,0);

                float predicted_pitch = current_angles.x /* + (raw_gyro.x * (180.0f / M_PI) * LATENCY_SEC)*/;
                float predicted_yaw = current_angles.y /* + (raw_gyro.y * (180.0f / M_PI) * LATENCY_SEC) + (prediction_angle * (180.0f / M_PI))*/;
                float predicted_roll = current_angles.z /* + (raw_gyro.z * (180.0f / M_PI) * LATENCY_SEC)*/;

                float pitch_rad = -predicted_pitch * DEG_TO_RAD;
                float yaw_rad = -predicted_yaw * DEG_TO_RAD;
                float roll_rad = -predicted_roll * DEG_TO_RAD;

                for (const auto& pixel : lastDrawnPixels) {
                    canvas()->SetPixel(pixel.x, pixel.y, 0, 0, 0);
                }
                lastDrawnPixels.clear();

                for (const auto& edge : modelEdges) {
                    Vector3 p[2] = { modelVertices[edge.v1], modelVertices[edge.v2] };
                    Vector3 rp[2];

                    float cy = cos(yaw_rad), sy = sin(yaw_rad);
                    float cp = cos(pitch_rad), sp = sin(pitch_rad);
                    float cr = cos(roll_rad), sr = sin(roll_rad);

                    for (int j = 0; j < 2; j++) {
                        float lx = p[j].x, ly = p[j].y, lz = p[j].z;

                        float x1 = lx * cy + lz * sy;
                        float z1 = -lx * sy + lz * cy;

                        float y2 = ly * cp - z1 * sp;
                        float z2 = ly * sp + z1 * cp;

                        rp[j].x = x1 * cr - y2 * sr;
                        rp[j].y = x1 * sr + y2 * cr;
                        rp[j].z = z2;
                    }

                    if ((rp[0].z > 0 && rp[1].z < 0) || (rp[0].z < 0 && rp[1].z > 0)) {
                        float t = -rp[0].z / (rp[1].z - rp[0].z);
                        float ix = rp[0].x + t * (rp[1].x - rp[0].x);
                        float iy = rp[0].y + t * (rp[1].y - rp[0].y);

                        Vector3 orig1 = modelVertices[edge.v1];
                        Vector3 orig2 = modelVertices[edge.v2];

                        float ox = orig1.x + t * (orig2.x - orig1.x);
                        float oy = orig1.y + t * (orig2.y - orig1.y);
                        float oz = orig1.z + t * (orig2.z - orig1.z);

                        int r = (int)(((ox - minP.x) / (maxP.x - minP.x)) * 255);
                        int g = (int)(((oy - minP.y) / (maxP.y - minP.y)) * 255);
                        int b = (int)(((oz - minP.z) / (maxP.z - minP.z)) * 255);

                        canvas()->SetPixel(cent_x + (int)ix, cent_y + (int)iy, r, g, b);
                        lastDrawnPixels.push_back({ cent_x + (int)ix, cent_y + (int)iy });
                    }
                }
            }
        }

        close(file_descriptor);
    }

private:
    void RequestConfig() {
        current_static_offset = {
          LoadConfig(base_path + "setX.txt", 0.0f),
          LoadConfig(base_path + "setY.txt", 0.0f),
          LoadConfig(base_path + "setZ.txt", 0.0f)
        };

        std::string new_id;

        if (std::ifstream id_file(base_path + "model_id.txt"); id_file.is_open()) {
            std::getline(id_file, new_id);
            id_file.close();
        }

        if (std::ifstream(reset_path)) {
            offset_angles = { 0, 0, 0 };

            std::remove(reset_path.c_str());
            std::cout << "[RESET] Orientation and Offsets cleared!" << std::endl;
        }

        current_scale = LoadConfig(base_path + "config.txt", 12.0f);
        current_rot_speed = {
          LoadConfig(base_path + "rotX.txt", 0.0f),
          LoadConfig(base_path + "rotY.txt", 0.0f),
          LoadConfig(base_path + "rotZ.txt", 0.0f)
        };

        if (new_id != last_model_id || current_scale != last_scale || current_rot_speed.x != last_rot_speed.x || current_rot_speed.y != last_rot_speed.y || current_rot_speed.z != last_rot_speed.z || last_static_offset.x != current_static_offset.x || last_static_offset.y != current_static_offset.y || last_static_offset.z != current_static_offset.z) {
            std::cout << "[UPDATE] Change detected! ID: " << new_id
                << " | Scale: " << current_scale << std::endl;

            model_path = base_path + "models/" + new_id + ".obj";

            last_model_id = new_id;
            last_scale = current_scale;
            last_rot_speed = current_rot_speed;
            last_static_offset = current_static_offset;

            if (new_id.find(".bin") != std::string::npos) {
                std::cout << "[VIDEO] Starting Playing Playback..." << std::endl;
            }
            else {
                LoadOBJ(model_path);
            }
        }

        {
            std::lock_guard<std::mutex> lock(config_mutex);
            current_model_id = new_id;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    void LoadOBJ(const std::string& filename) {
        canvas()->Fill(0, 0, 0);

        modelVertices.clear();
        modelEdges.clear();
        minP = { NAN, NAN, NAN };
        maxP = { NAN, NAN, NAN };

        resetMinMax = true;

        std::cout << "[OBJ] Attempting to open: " << filename << std::endl;

        std::ifstream file(filename);
        if (!file.is_open()) {
            perror(("[OBJ] Error opening " + filename).c_str());
        }
        std::cout << "[OBJ] File opened successfully!" << std::endl;

        std::string line;
        int vCount = 0;
        int fCount = 0;

        while (std::getline(file, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();

            std::istringstream iss(line);
            std::string type;
            if (!(iss >> type)) continue;

            if (type == "v") {
                Vector3 v;
                if (iss >> v.x >> v.y >> v.z) {
                    modelVertices.push_back(v);
                    vCount++;
                }
            }
            else if (type == "f") {
                std::vector<int> faceIndices;
                std::string vertexData;
                while (iss >> vertexData) {
                    try {
                        size_t slashPos = vertexData.find('/');
                        int idx = std::stoi(vertexData.substr(0, slashPos)) - 1;
                        faceIndices.push_back(idx);
                    }
                    catch (...) { continue; }
                }
                for (size_t i = 0; i < faceIndices.size(); i++) {
                    modelEdges.push_back({ faceIndices[i], faceIndices[(i + 1) % faceIndices.size()] });
                }
                fCount++;
            }
        }

        std::cout << "[OBJ] Finished parsing. Vertices: " << vCount << ", Faces: " << fCount << std::endl;

        if (modelVertices.empty()) {
            std::cerr << "[OBJ] Warning: No vertices were loaded!" << std::endl;
        }

        Vector3 minV = modelVertices[0], maxV = modelVertices[0];
        for (const auto& v : modelVertices) {
            minV.x = std::min(minV.x, v.x); maxV.x = std::max(maxV.x, v.x);
            minV.y = std::min(minV.y, v.y); maxV.y = std::max(maxV.y, v.y);
            minV.z = std::min(minV.z, v.z); maxV.z = std::max(maxV.z, v.z);
        }

        Vector3 center = { (minV.x + maxV.x) / 2, (minV.y + maxV.y) / 2, (minV.z + maxV.z) / 2 };
        float configScale = LoadConfig("/opt/volumetrix/rpi-fb-matrix/rpi-rgb-led-matrix/examples-api-use/config.txt", 12.0f);

        for (auto& v : modelVertices) {
            v.x = (v.x - center.x) * configScale;
            v.y = (v.y - center.y) * configScale;
            v.z = (v.z - center.z) * configScale;
        }

        minP = modelVertices[0], maxP = modelVertices[0];
        for (const auto& v : modelVertices) {
            minP.x = std::min(minP.x, v.x); maxP.x = std::max(maxP.x, v.x);
            minP.y = std::min(minP.y, v.y); maxP.y = std::max(maxP.y, v.y);
            minP.z = std::min(minP.z, v.z); maxP.z = std::max(maxP.z, v.z);
        }

        std::cout << "[OBJ] Scaling complete. Scale Factor: " << configScale << std::endl;
    }

    float LoadConfig(const std::string& filename, float defaultData) {
        std::ifstream file(filename);
        float data = defaultData;
        if (file.is_open()) {
            if (file >> data) {
                // std::cout << "[CONFIG] Loaded scale: " << scale << " from " << filename << std::endl;
            }
            else {
                std::cerr << "[CONFIG] Failed to parse float, using default." << std::endl;
            }
        }
        else {
            std::cerr << "[CONFIG] Could not open config file, using default: " << defaultData << std::endl;
        }
        return data;
    }
};

static int usage(const char* progname) {
    fprintf(stderr, "usage: %s <options> -D <demo-nr> [optional parameter]\n",
        progname);
    fprintf(stderr, "Options:\n");
    fprintf(stderr,
        "\t-D <demo-nr>              : Always needs to be set\n"
        "\t-L                        : Large display, in which each chain is 'folded down'\n"
        "\t                            in the middle in an U-arrangement to get more vertical space.\n"
        "\t-R <rotation>             : Sets the rotation of matrix. "
        "Allowed: 0, 90, 180, 270. Default: 0.\n"
        "\t-t <seconds>              : Run for these number of seconds, then exit.\n");


    rgb_matrix::PrintMatrixFlags(stderr);
    return 1;
}

int main(int argc, char* argv[]) {

    file_descriptor = i2c_init(i2c_bus, device_address);

    if (file_descriptor < 0) {
        return 1;
    }

    unsigned char pwr_mgmt_data[2] = { MPU9265_PWR_MGMT_1, 0x00 };
    writeBytes(pwr_mgmt_data, 2);
    usleep(100000);

    int runtime_seconds = -1;
    int demo = -1;
    int rotation = 0;
    bool large_display = false;

    RGBMatrix::Options matrix_options;
    rgb_matrix::RuntimeOptions runtime_opt;

    runtime_opt.drop_privileges = 0;
    matrix_options.rows = 32;
    matrix_options.chain_length = 1;
    matrix_options.parallel = 1;

    if (!ParseOptionsFromFlags(&argc, &argv, &matrix_options, &runtime_opt)) {
        return usage(argv[0]);
    }

    int opt;
    while ((opt = getopt(argc, argv, "dD:t:r:P:c:p:b:m:LR:")) != -1) {
        switch (opt) {
        case 'D':
            demo = atoi(optarg);
            break;

        case 't':
            runtime_seconds = atoi(optarg);
            break;

        case 'R':
            rotation = atoi(optarg);
            break;

        case 'L':
            if (matrix_options.chain_length == 1) {
                matrix_options.chain_length = 4;
            }
            large_display = true;
            break;

        case 'd':
            runtime_opt.daemon = 1;
            break;

        case 'r':
            fprintf(stderr, "Instead of deprecated -r, use --led-rows=%s instead.\n",
                optarg);
            matrix_options.rows = atoi(optarg);
            break;

        case 'P':
            matrix_options.parallel = atoi(optarg);
            break;

        case 'c':
            fprintf(stderr, "Instead of deprecated -c, use --led-chain=%s instead.\n",
                optarg);
            matrix_options.chain_length = atoi(optarg);
            break;

        case 'p':
            matrix_options.pwm_bits = atoi(optarg);
            break;

        case 'b':
            matrix_options.brightness = atoi(optarg);
            break;

        default:
            return usage(argv[0]);
        }
    }

    if (demo < 0) {
        fprintf(stderr, TERM_ERR "Expected required option -D <demo>\n" TERM_NORM);
        return usage(argv[0]);
    }

    if (rotation % 90 != 0) {
        fprintf(stderr, TERM_ERR "Rotation %d not allowed! "
            "Only 0, 90, 180 and 270 are possible.\n" TERM_NORM, rotation);
        return 1;
    }

    RGBMatrix* matrix = CreateMatrixFromOptions(matrix_options, runtime_opt);
    if (matrix == NULL)
        return 1;

    if (large_display) {
        matrix->ApplyStaticTransformer(UArrangementTransformer(
            matrix_options.parallel));
    }

    if (rotation > 0) {
        matrix->ApplyStaticTransformer(RotateTransformer(rotation));
    }

    printf("Size: %dx%d. Hardware gpio mapping: %s\n",
        matrix->width(), matrix->height(), matrix_options.hardware_mapping);

    Canvas* canvas = matrix;

    ThreadedCanvasManipulator* image_gen = NULL;
    image_gen = new ObjModelRenderer(canvas);

    signal(SIGTERM, InterruptHandler);
    signal(SIGINT, InterruptHandler);

    image_gen->Start();

    if (runtime_seconds > 0) {
        sleep(runtime_seconds);
    }
    else {
        printf("Press <CTRL-C> to exit and reset LEDs\n");
        while (!interrupt_received) {
            sleep(1);
        }
    }

    delete image_gen;
    delete canvas;

    printf("\%s. Exiting.\n",
        interrupt_received ? "Received CTRL-C" : "Timeout reached");
    return 0;
}
