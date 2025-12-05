#include "air.h"

#include "Navio2/C++/LSM9DS1.h"
#include "Navio2/C++/MPU9250.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <json/json.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#define GRAVITY 9.80665

static void usage() {
    printf("Usage: sudo ./calibrate_imu <LSM9DS1|MPU9250>\n");
}

// Ensure directory exists, creating parent components as needed (POSIX-style paths)
static bool ensure_dir_recursive(const std::string &path) {
    if (path.empty()) return false;
    std::string current;
    size_t i = 0;
    if (path[0] == '/') {
        current = "/";
        i = 1;
    }
    while (i <= path.size()) {
        size_t slash = path.find('/', i);
        std::string part = path.substr(i, (slash == std::string::npos ? path.size() : slash) - i);
        if (!part.empty()) {
            if (!current.empty() && current.back() != '/') current += "/";
            current += part;
            struct stat st{};
            if (stat(current.c_str(), &st) != 0) {
                if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                    perror(("mkdir failed for " + current).c_str());
                    return false;
                }
            } else if (!S_ISDIR(st.st_mode)) {
                fprintf(stderr, "%s exists and is not a directory\n", current.c_str());
                return false;
            }
        }
        if (slash == std::string::npos) break;
        i = slash + 1;
    }
    return true;
}

// Jacobi eigen decomposition for symmetric 3x3 matrix
static void jacobi_eigen_sym_3x3(const double A[3][3], double V[3][3], double d[3]) {
    // Initialize V to identity and d to diagonal of A
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            V[i][j] = (i == j) ? 1.0 : 0.0;
        }
        d[i] = A[i][i];
    }
    double B[3] = {d[0], d[1], d[2]};
    double Z[3] = {0.0, 0.0, 0.0};
    double off = std::abs(A[0][1]) + std::abs(A[0][2]) + std::abs(A[1][2]);
    if (off == 0.0) return;
    // Copy A since it's const
    double a[3][3];
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) a[i][j] = A[i][j];
    for (int iter = 0; iter < 50; ++iter) {
        double sm = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
        if (sm < 1e-12) break;
        double tresh = (iter < 4) ? (0.2 * sm / 9.0) : 0.0;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                double g = 100.0 * std::abs(a[p][q]);
                if (iter > 4 && (std::abs(d[p]) + g) == std::abs(d[p]) && (std::abs(d[q]) + g) == std::abs(d[q])) {
                    a[p][q] = 0.0;
                } else if (std::abs(a[p][q]) > tresh) {
                    double h = d[q] - d[p];
                    double t;
                    if ((std::abs(h) + g) == std::abs(h)) {
                        t = a[p][q] / h;
                    } else {
                        double theta = 0.5 * h / a[p][q];
                        t = 1.0 / (std::abs(theta) + std::sqrt(1.0 + theta * theta));
                        if (theta < 0.0) t = -t;
                    }
                    double c = 1.0 / std::sqrt(1 + t * t);
                    double s = t * c;
                    double tau = s / (1.0 + c);
                    double h2 = t * a[p][q];
                    Z[p] -= h2;
                    Z[q] += h2;
                    d[p] -= h2;
                    d[q] += h2;
                    a[p][q] = 0.0;
                    for (int j = 0; j < p; ++j) {
                        double g2 = a[j][p];
                        double h3 = a[j][q];
                        a[j][p] = g2 - s * (h3 + g2 * tau);
                        a[j][q] = h3 + s * (g2 - h3 * tau);
                    }
                    for (int j = p + 1; j < q; ++j) {
                        double g2 = a[p][j];
                        double h3 = a[j][q];
                        a[p][j] = g2 - s * (h3 + g2 * tau);
                        a[j][q] = h3 + s * (g2 - h3 * tau);
                    }
                    for (int j = q + 1; j < 3; ++j) {
                        double g2 = a[p][j];
                        double h3 = a[q][j];
                        a[p][j] = g2 - s * (h3 + g2 * tau);
                        a[q][j] = h3 + s * (g2 - h3 * tau);
                    }
                    for (int j = 0; j < 3; ++j) {
                        double g2 = V[j][p];
                        double h3 = V[j][q];
                        V[j][p] = g2 - s * (h3 + g2 * tau);
                        V[j][q] = h3 + s * (g2 - h3 * tau);
                    }
                }
            }
        }
        for (int i = 0; i < 3; ++i) {
            B[i] += Z[i];
            d[i] = B[i];
            Z[i] = 0.0;
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }
    std::string id(argv[1]);
    for (auto &c : id) c = (char)toupper(c);

    InertialSensor* imu = nullptr;
    if (id == "LSM9DS1") {
        imu = new LSM9DS1();
    } else if (id == "MPU9250") {
        imu = new MPU9250();
    } else {
        printf("Unknown IMU id '%s'. Expected LSM9DS1 or MPU9250.\n", id.c_str());
        return 1;
    }

    if (!imu->probe()) {
        printf("IMU probe failed for %s.\n", id.c_str());
        return 1;
    }
    imu->initialize();
    printf("IMU initialized: %s\n", id.c_str());

    // Sampling params
    const int accel_gyro_samples = 1000; // ~10s at 100Hz
    const int mag_warmup_samples = 50;
    const int mag_samples_target = 3000; // user rotates IMU; ~30s at 100Hz
    const int sample_delay_us = 10000; // 100 Hz

    // Accumulation for biases
    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    double gx_sum = 0, gy_sum = 0, gz_sum = 0;

    printf("\nStep 1/2: Place the vehicle flat and perfectly still.\n");
    printf("Collecting accelerometer and gyroscope bias (about 10 seconds)...\n");
    usleep(2000000);

    for (int i = 0; i < accel_gyro_samples; ++i) {
        imu->update();
        double ax, ay, az, gx, gy, gz, mx, my, mz;
        imu->read_accelerometer(&ax, &ay, &az);
        imu->read_gyroscope(&gx, &gy, &gz);
        // do not adjust; raw values
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        usleep(sample_delay_us);
    }

    double ax_bias = ax_sum / accel_gyro_samples;
    double ay_bias = ay_sum / accel_gyro_samples;
    double az_bias = (az_sum / accel_gyro_samples) - GRAVITY; // align +Z with gravity
    double gx_bias = gx_sum / accel_gyro_samples;
    double gy_bias = gy_sum / accel_gyro_samples;
    double gz_bias = gz_sum / accel_gyro_samples;

    printf("Computed accel biases: ax=%.4f ay=%.4f az=%.4f (g-comp)\n", ax_bias, ay_bias, az_bias);
    printf("Computed gyro  biases: gx=%.4f gy=%.4f gz=%.4f\n", gx_bias, gy_bias, gz_bias);

    // Magnetometer hard-iron calibration via min/max
    printf("\nStep 2/2: Magnetometer hard-iron calibration.\n");
    printf("Rotate the vehicle slowly through all orientations for ~30-60 seconds.\n");
    printf("Collecting magnetometer min/max...\n");

    // Warm up a little
    for (int i = 0; i < mag_warmup_samples; ++i) {
        imu->update();
        usleep(sample_delay_us);
    }

    double mx_min = 1e9, my_min = 1e9, mz_min = 1e9;
    double mx_max = -1e9, my_max = -1e9, mz_max = -1e9;
    int mag_count = 0;
    auto start = std::chrono::steady_clock::now();
    // Also accumulate for covariance-based soft-iron matrix
    double mean[3] = {0,0,0};
    std::vector<std::array<double,3>> mag_samples;
    mag_samples.reserve(mag_samples_target + 100);

    while (mag_count < mag_samples_target) {
        imu->update();
        double mx, my, mz;
        double ax, ay, az, gx, gy, gz;
        imu->read_magnetometer(&mx, &my, &mz);
        // Track min/max
        if (mx < mx_min) mx_min = mx; if (mx > mx_max) mx_max = mx;
        if (my < my_min) my_min = my; if (my > my_max) my_max = my;
        if (mz < mz_min) mz_min = mz; if (mz > mz_max) mz_max = mz;
        mag_samples.push_back({mx, my, mz});
        mean[0] += mx; mean[1] += my; mean[2] += mz;
        mag_count++;
        usleep(sample_delay_us);
        if ((mag_count % 200) == 0) {
            printf("."); fflush(stdout);
        }
    }
    printf("\nMag ranges: mx[%.3f, %.3f] my[%.3f, %.3f] mz[%.3f, %.3f]\n",
           mx_min, mx_max, my_min, my_max, mz_min, mz_max);

    double mx_bias = 0.5 * (mx_min + mx_max);
    double my_bias = 0.5 * (my_min + my_max);
    double mz_bias = 0.5 * (mz_min + mz_max);

    // Compute covariance of hard-iron corrected samples
    double N = (double)mag_samples.size();
    mean[0] /= N; mean[1] /= N; mean[2] /= N;
    double C[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (auto &s : mag_samples) {
        double vx = s[0] - mx_bias;
        double vy = s[1] - my_bias;
        double vz = s[2] - mz_bias;
        C[0][0] += vx*vx; C[0][1] += vx*vy; C[0][2] += vx*vz;
        C[1][0] += vy*vx; C[1][1] += vy*vy; C[1][2] += vy*vz;
        C[2][0] += vz*vx; C[2][1] += vz*vy; C[2][2] += vz*vz;
    }
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) C[i][j] /= (N > 1 ? (N - 1.0) : 1.0);

    // Eigen decomposition of symmetric covariance
    double V[3][3];
    double dlam[3];
    jacobi_eigen_sym_3x3(C, V, dlam);
    // Construct inverse sqrt(C): M = V * diag(1/sqrt(lam)) * V^T
    double Dinv2[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (int i = 0; i < 3; ++i) {
        double lam = dlam[i];
        if (lam < 1e-9) lam = 1e-9; // avoid div by zero
        Dinv2[i][i] = 1.0 / std::sqrt(lam);
    }
    double temp[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double Mmat[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    // temp = V * Dinv2
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                temp[i][j] += V[i][k] * Dinv2[k][j];
    // M = temp * V^T
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                Mmat[i][j] += temp[i][k] * V[j][k];

    double M[9] = { Mmat[0][0], Mmat[0][1], Mmat[0][2],
                    Mmat[1][0], Mmat[1][1], Mmat[1][2],
                    Mmat[2][0], Mmat[2][1], Mmat[2][2] };

    // Offsets arranged as expected by InertialSensor::adjust()
    double offsets[9] = {ax_bias, ay_bias, az_bias, gx_bias, gy_bias, gz_bias, mx_bias, my_bias, mz_bias};

    // Ensure output directory exists
    const std::string out_dir = "data/calibration";
    if (!ensure_dir_recursive(out_dir)) {
        printf("ERROR: Could not create output directory '%s'\n", out_dir.c_str());
        return 1;
    }

    // Write JSON calibration (readable)
    std::string json_path = out_dir + "/" + id + std::string("_calibration.json");
    Json::Value root;
    root["imu_id"] = id;
    root["method"] = "mag covariance whitening (ellipsoid-to-sphere)";
    root["reference"] = "Q. Li and J. G. Griffiths, Least squares ellipsoid specific fitting, 2004; also see J. M. Vasconcelos et al., 2011.";
    root["accel_bias"] = Json::arrayValue;
    root["gyro_bias"] = Json::arrayValue;
    root["mag_bias"] = Json::arrayValue;
    for (int i = 0; i < 3; ++i) root["accel_bias"].append(offsets[i]);
    for (int i = 3; i < 6; ++i) root["gyro_bias"].append(offsets[i]);
    for (int i = 6; i < 9; ++i) root["mag_bias"].append(offsets[i]);
    Json::Value mat(Json::arrayValue);
    for (int r = 0; r < 3; ++r) {
        Json::Value row(Json::arrayValue);
        for (int c = 0; c < 3; ++c) row.append(Mmat[r][c]);
        mat.append(row);
    }
    root["softiron_matrix"] = mat;

    std::ofstream jf(json_path);
    if (!jf.is_open()) {
        printf("ERROR: Could not open %s for writing. Ensure directory exists and permissions are sufficient.\n", json_path.c_str());
        return 1;
    }
    jf << root;
    jf.close();
    printf("\nCalibration saved (JSON): %s\n", json_path.c_str());

    // Also write binary for backward compatibility (optional)
    std::string bin_path = out_dir + "/" + id + std::string("_calibration.bin");
    std::ofstream bf(bin_path, std::ios::binary | std::ios::out);
    if (bf.is_open()) {
        bf.write(reinterpret_cast<char*>(offsets), sizeof(double) * 9);
        bf.write(reinterpret_cast<char*>(M), sizeof(double) * 9);
        bf.close();
        printf("Calibration saved (BIN, legacy): %s\n", bin_path.c_str());
    }

    printf("Next runs will load the JSON calibration automatically.\n");
    return 0;
}
