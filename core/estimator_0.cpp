#include "air.h"
#include <cmath>
#include <cstring>
#include <algorithm>

// Constants
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)
#define GRAVITY 9.81
#define EARTH_RADIUS 6378137.0

// EKF State dimension - Added gyro bias
#define STATE_DIM 18  // [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r, bax, bay, baz, bp, bq, br]

// State indices
#define IDX_X    0
#define IDX_Y    1
#define IDX_Z    2
#define IDX_VX   3
#define IDX_VY   4
#define IDX_VZ   5
#define IDX_PHI  6
#define IDX_THETA 7
#define IDX_PSI  8
#define IDX_P    9
#define IDX_Q    10
#define IDX_R    11
#define IDX_BAX  12
#define IDX_BAY  13
#define IDX_BAZ  14
#define IDX_BP   15
#define IDX_BQ   16
#define IDX_BR   17

// Initial reference position (set on first GPS fix)
struct gps_reference {
    double lat0;
    double lon0;
    double alt0;
    bool initialized;
};

// EKF State structure
struct ekf_state {
    double x[STATE_DIM];     // State vector
    double P[STATE_DIM][STATE_DIM];  // Covariance matrix
    bool initialized;
};

// Convert GPS lat/lon to local NED coordinates
void gps_to_ned(double lat, double lon, double alt, 
                const gps_reference& ref, 
                double* north, double* east, double* down) {
    double dlat = (lat - ref.lat0) * DEG_TO_RAD;
    double dlon = (lon - ref.lon0) * DEG_TO_RAD;
    double avg_lat = (lat + ref.lat0) / 2.0 * DEG_TO_RAD;
    
    *north = dlat * EARTH_RADIUS;
    *east = dlon * EARTH_RADIUS * cos(avg_lat);
    *down = -(alt - ref.alt0);
}

// Normalize angle to [-PI, PI]
static inline double normalize_angle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

// Clamp value to range
static inline double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

// Matrix operations
void matrix_multiply(double A[][STATE_DIM], double B[][STATE_DIM], double C[][STATE_DIM], int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < n; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void matrix_multiply_rect(double A[][STATE_DIM], double B[][STATE_DIM], double C[][STATE_DIM], 
                         int rows_A, int cols_A, int cols_B) {
    for (int i = 0; i < rows_A; i++) {
        for (int j = 0; j < cols_B; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < cols_A; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void matrix_add(double A[][STATE_DIM], double B[][STATE_DIM], double C[][STATE_DIM], int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void matrix_transpose(double A[][STATE_DIM], double AT[][STATE_DIM], int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            AT[i][j] = A[j][i];
        }
    }
}

// Matrix inversion for small matrices (using Gauss-Jordan with pivoting)
template<int N>
bool matrix_inverse_small(double A[][N], double Ainv[][N]) {
    double temp[N][N * 2];
    int pivot_row[N];
    
    // Create augmented matrix [A|I]
    for (int i = 0; i < N; i++) {
        pivot_row[i] = i;
        for (int j = 0; j < N; j++) {
            temp[i][j] = A[i][j];
            temp[i][j + N] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // Gauss-Jordan elimination with partial pivoting
    for (int i = 0; i < N; i++) {
        // Find pivot
        double max_val = fabs(temp[i][i]);
        int max_row = i;
        for (int k = i + 1; k < N; k++) {
            if (fabs(temp[k][i]) > max_val) {
                max_val = fabs(temp[k][i]);
                max_row = k;
            }
        }
        
        // Swap rows
        if (max_row != i) {
            for (int j = 0; j < 2 * N; j++) {
                double tmp = temp[i][j];
                temp[i][j] = temp[max_row][j];
                temp[max_row][j] = tmp;
            }
            int tmp_idx = pivot_row[i];
            pivot_row[i] = pivot_row[max_row];
            pivot_row[max_row] = tmp_idx;
        }
        
        double pivot = temp[i][i];
        if (fabs(pivot) < 1e-12) return false;
        
        // Normalize pivot row
        for (int j = 0; j < 2 * N; j++) {
            temp[i][j] /= pivot;
        }
        
        // Eliminate column
        for (int k = 0; k < N; k++) {
            if (k != i) {
                double factor = temp[k][i];
                for (int j = 0; j < 2 * N; j++) {
                    temp[k][j] -= factor * temp[i][j];
                }
            }
        }
    }
    
    // Extract inverse matrix
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            Ainv[pivot_row[i]][j] = temp[i][j + N];
        }
    }
    
    return true;
}

// Initialize EKF
void ekf_init(ekf_state* ekf) {
    // Initialize state to zero
    memset(ekf->x, 0, sizeof(ekf->x));
    
    // Initialize covariance matrix with appropriate uncertainty
    memset(ekf->P, 0, sizeof(ekf->P));
    
    // Position uncertainty (m^2) - large initial uncertainty
    for (int i = 0; i < 3; i++) ekf->P[i][i] = 100.0;
    
    // Velocity uncertainty (m^2/s^2)
    for (int i = 3; i < 6; i++) ekf->P[i][i] = 10.0;
    
    // Attitude uncertainty (rad^2) - moderate for attitude
    for (int i = 6; i < 9; i++) ekf->P[i][i] = 0.1;
    
    // Angular rate uncertainty (rad^2/s^2)
    for (int i = 9; i < 12; i++) ekf->P[i][i] = 0.01;
    
    // Accelerometer bias uncertainty (m^2/s^4)
    for (int i = 12; i < 15; i++) ekf->P[i][i] = 0.01;
    
    // Gyroscope bias uncertainty (rad^2/s^2)
    for (int i = 15; i < 18; i++) ekf->P[i][i] = 0.001;
    
    ekf->initialized = true;
}

// Compute rotation matrix from body to NED frame (Euler angles)
void body_to_ned_matrix(double phi, double theta, double psi, 
                       double R[3][3]) {
    double cp = cos(phi), sp = sin(phi);
    double ct = cos(theta), st = sin(theta);
    double cs = cos(psi), ss = sin(psi);
    
    // Body to NED rotation matrix
    R[0][0] = ct * cs;
    R[0][1] = sp * st * cs - cp * ss;
    R[0][2] = cp * st * cs + sp * ss;
    
    R[1][0] = ct * ss;
    R[1][1] = sp * st * ss + cp * cs;
    R[1][2] = cp * st * ss - sp * cs;
    
    R[2][0] = -st;
    R[2][1] = sp * ct;
    R[2][2] = cp * ct;
}

// EKF Prediction step with proper Jacobian
void ekf_predict(ekf_state* ekf, double ax, double ay, double az, 
                 double p, double q, double r, double dt) {
    // Extract state variables
    double x = ekf->x[IDX_X], y = ekf->x[IDX_Y], z = ekf->x[IDX_Z];
    double vx = ekf->x[IDX_VX], vy = ekf->x[IDX_VY], vz = ekf->x[IDX_VZ];
    double phi = ekf->x[IDX_PHI], theta = ekf->x[IDX_THETA], psi = ekf->x[IDX_PSI];
    double bax = ekf->x[IDX_BAX], bay = ekf->x[IDX_BAY], baz = ekf->x[IDX_BAZ];
    double bp = ekf->x[IDX_BP], bq = ekf->x[IDX_BQ], br = ekf->x[IDX_BR];
    
    // Remove biases
    double ax_c = ax - bax;
    double ay_c = ay - bay;
    double az_c = az - baz;
    double p_c = p - bp;
    double q_c = q - bq;
    double r_c = r - br;
    
    // Precompute trig functions
    double cp = cos(phi), sp = sin(phi);
    double ct = cos(theta), st = sin(theta);
    double cs = cos(psi), ss = sin(psi);
    double tt = tan(theta);
    double sec_theta = 1.0 / ct;
    
    // Protect against gimbal lock (near ±90 degrees pitch)
    double theta_clamped = clamp(theta, -PI/2 + 0.01, PI/2 - 0.01);
    if (fabs(theta) > PI/2 - 0.01) {
        tt = tan(theta_clamped);
        sec_theta = 1.0 / cos(theta_clamped);
    }
    
    // Rotate acceleration from body to NED frame
    double R[3][3];
    body_to_ned_matrix(phi, theta, psi, R);
    
    double ax_ned = R[0][0] * ax_c + R[0][1] * ay_c + R[0][2] * az_c;
    double ay_ned = R[1][0] * ax_c + R[1][1] * ay_c + R[1][2] * az_c;
    double az_ned = R[2][0] * ax_c + R[2][1] * ay_c + R[2][2] * az_c;
    
    // State prediction
    ekf->x[IDX_X] = x + vx * dt;
    ekf->x[IDX_Y] = y + vy * dt;
    ekf->x[IDX_Z] = z + vz * dt;
    ekf->x[IDX_VX] = vx + ax_ned * dt;
    ekf->x[IDX_VY] = vy + ay_ned * dt;
    ekf->x[IDX_VZ] = vz + (az_ned + GRAVITY) * dt;  // NED: down is positive
    
    // Euler angle derivatives (corrected)
    double phi_dot = p_c + q_c * sp * tt + r_c * cp * tt;
    double theta_dot = q_c * cp - r_c * sp;
    double psi_dot = (q_c * sp + r_c * cp) * sec_theta;
    
    ekf->x[IDX_PHI] = normalize_angle(phi + phi_dot * dt);
    ekf->x[IDX_THETA] = normalize_angle(theta + theta_dot * dt);
    ekf->x[IDX_PSI] = normalize_angle(psi + psi_dot * dt);
    
    ekf->x[IDX_P] = p_c;  // Store corrected rates in state
    ekf->x[IDX_Q] = q_c;
    ekf->x[IDX_R] = r_c;
    
    // Bias states remain constant (random walk model)
    
    // Compute Jacobian matrix F = df/dx
    double F[STATE_DIM][STATE_DIM] = {0};
    
    // Identity matrix
    for (int i = 0; i < STATE_DIM; i++) {
        F[i][i] = 1.0;
    }
    
    // Position derivatives: d/dv
    F[IDX_X][IDX_VX] = dt;
    F[IDX_Y][IDX_VY] = dt;
    F[IDX_Z][IDX_VZ] = dt;
    
    // Velocity derivatives: d/d(phi, theta, psi) and d/d(bax, bay, baz)
    // d(ax_ned)/d(phi)
    F[IDX_VX][IDX_PHI] = dt * (R[0][2] * ay_c - R[0][1] * az_c);
    F[IDX_VY][IDX_PHI] = dt * (R[1][2] * ay_c - R[1][1] * az_c);
    F[IDX_VZ][IDX_PHI] = dt * (R[2][2] * ay_c - R[2][1] * az_c);
    
    // d(ax_ned)/d(theta)
    F[IDX_VX][IDX_THETA] = dt * ((-st * cs) * ax_c + 
                                  (sp * ct * cs) * ay_c + 
                                  (cp * ct * cs) * az_c);
    F[IDX_VY][IDX_THETA] = dt * ((-st * ss) * ax_c + 
                                  (sp * ct * ss) * ay_c + 
                                  (cp * ct * ss) * az_c);
    F[IDX_VZ][IDX_THETA] = dt * ((-ct) * ax_c + (-sp * st) * ay_c + (-cp * st) * az_c);
    
    // d(ax_ned)/d(psi)
    F[IDX_VX][IDX_PSI] = dt * ((-ct * ss) * ax_c + 
                                (-sp * st * ss - cp * cs) * ay_c + 
                                (-cp * st * ss + sp * cs) * az_c);
    F[IDX_VY][IDX_PSI] = dt * ((ct * cs) * ax_c + 
                                (sp * st * cs - cp * ss) * ay_c + 
                                (cp * st * cs + sp * ss) * az_c);
    
    // d(ax_ned)/d(bias)
    F[IDX_VX][IDX_BAX] = -dt * R[0][0];
    F[IDX_VX][IDX_BAY] = -dt * R[0][1];
    F[IDX_VX][IDX_BAZ] = -dt * R[0][2];
    F[IDX_VY][IDX_BAX] = -dt * R[1][0];
    F[IDX_VY][IDX_BAY] = -dt * R[1][1];
    F[IDX_VY][IDX_BAZ] = -dt * R[1][2];
    F[IDX_VZ][IDX_BAX] = -dt * R[2][0];
    F[IDX_VZ][IDX_BAY] = -dt * R[2][1];
    F[IDX_VZ][IDX_BAZ] = -dt * R[2][2];
    
    // Attitude derivatives: d/d(p, q, r) and d/d(bp, bq, br)
    double tt2 = tt * tt;
    double sec2_theta = sec_theta * sec_theta;
    
    // d(phi_dot)/d(phi, theta)
    F[IDX_PHI][IDX_PHI] = 1.0 + dt * (q_c * cp * tt + r_c * (-sp) * tt);
    F[IDX_PHI][IDX_THETA] = dt * ((q_c * sp + r_c * cp) * sec2_theta);
    F[IDX_PHI][IDX_P] = dt;
    F[IDX_PHI][IDX_Q] = dt * sp * tt;
    F[IDX_PHI][IDX_R] = dt * cp * tt;
    F[IDX_PHI][IDX_BP] = -dt;
    F[IDX_PHI][IDX_BQ] = -dt * sp * tt;
    F[IDX_PHI][IDX_BR] = -dt * cp * tt;
    
    // d(theta_dot)/d(phi)
    F[IDX_THETA][IDX_PHI] = dt * (-q_c * sp - r_c * cp);
    F[IDX_THETA][IDX_Q] = dt * cp;
    F[IDX_THETA][IDX_R] = -dt * sp;
    F[IDX_THETA][IDX_BQ] = -dt * cp;
    F[IDX_THETA][IDX_BR] = dt * sp;
    
    // d(psi_dot)/d(phi, theta)
    F[IDX_PSI][IDX_PHI] = dt * ((q_c * cp - r_c * sp) * sec_theta);
    F[IDX_PSI][IDX_THETA] = dt * ((q_c * sp + r_c * cp) * tt * sec_theta);
    F[IDX_PSI][IDX_Q] = dt * sp * sec_theta;
    F[IDX_PSI][IDX_R] = dt * cp * sec_theta;
    F[IDX_PSI][IDX_BQ] = -dt * sp * sec_theta;
    F[IDX_PSI][IDX_BR] = -dt * cp * sec_theta;
    
    // Process noise covariance Q (tuned based on ArduPilot values)
    double Q[STATE_DIM][STATE_DIM] = {0};
    
    // Position process noise (m^2)
    double pos_noise = 0.05 * dt * dt;
    Q[IDX_X][IDX_X] = Q[IDX_Y][IDX_Y] = Q[IDX_Z][IDX_Z] = pos_noise;
    
    // Velocity process noise (m^2/s^2) - higher due to accel integration
    double vel_noise = 0.25 * dt * dt;
    Q[IDX_VX][IDX_VX] = Q[IDX_VY][IDX_VY] = Q[IDX_VZ][IDX_VZ] = vel_noise;
    
    // Attitude process noise (rad^2)
    double att_noise = 0.0005 * dt * dt;
    Q[IDX_PHI][IDX_PHI] = Q[IDX_THETA][IDX_THETA] = Q[IDX_PSI][IDX_PSI] = att_noise;
    
    // Angular rate process noise (rad^2/s^2)
    double rate_noise = 0.005 * dt;
    Q[IDX_P][IDX_P] = Q[IDX_Q][IDX_Q] = Q[IDX_R][IDX_R] = rate_noise;
    
    // Accelerometer bias drift (random walk)
    double acc_bias_noise = 0.00001 * dt;
    Q[IDX_BAX][IDX_BAX] = Q[IDX_BAY][IDX_BAY] = Q[IDX_BAZ][IDX_BAZ] = acc_bias_noise;
    
    // Gyroscope bias drift (random walk)
    double gyro_bias_noise = 0.000001 * dt;
    Q[IDX_BP][IDX_BP] = Q[IDX_BQ][IDX_BQ] = Q[IDX_BR][IDX_BR] = gyro_bias_noise;
    
    // Covariance prediction: P = F*P*F' + Q
    double FP[STATE_DIM][STATE_DIM], FPFt[STATE_DIM][STATE_DIM], Ft[STATE_DIM][STATE_DIM];
    matrix_multiply(F, ekf->P, FP, STATE_DIM);
    matrix_transpose(F, Ft, STATE_DIM);
    matrix_multiply(FP, Ft, FPFt, STATE_DIM);
    matrix_add(FPFt, Q, ekf->P, STATE_DIM);
}

// EKF Update step with GPS measurements
void ekf_update_gps(ekf_state* ekf, double gps_x, double gps_y, double gps_z,
                    double gps_vx, double gps_vy, double gps_vz) {
    const int meas_dim = 6;
    
    // Measurement matrix H (measuring position and velocity)
    double H[meas_dim][STATE_DIM] = {0};
    H[0][IDX_X] = 1.0;
    H[1][IDX_Y] = 1.0;
    H[2][IDX_Z] = 1.0;
    H[3][IDX_VX] = 1.0;
    H[4][IDX_VY] = 1.0;
    H[5][IDX_VZ] = 1.0;
    
    // Measurement noise covariance R (GPS uncertainties)
    double R[meas_dim][meas_dim] = {0};
    R[0][0] = R[1][1] = R[2][2] = 25.0;  // GPS position noise (5m std)
    R[3][3] = R[4][4] = R[5][5] = 1.0;   // GPS velocity noise (1 m/s std)
    
    // Innovation: z = measurement - predicted measurement
    double z[meas_dim];
    z[0] = gps_x - ekf->x[IDX_X];
    z[1] = gps_y - ekf->x[IDX_Y];
    z[2] = gps_z - ekf->x[IDX_Z];
    z[3] = gps_vx - ekf->x[IDX_VX];
    z[4] = gps_vy - ekf->x[IDX_VY];
    z[5] = gps_vz - ekf->x[IDX_VZ];
    
    // Innovation covariance: S = H*P*H' + R
    double HP[meas_dim][STATE_DIM] = {0};
    for (int i = 0; i < meas_dim; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                HP[i][j] += H[i][k] * ekf->P[k][j];
            }
        }
    }
    
    double HPHt[meas_dim][meas_dim] = {0};
    for (int i = 0; i < meas_dim; i++) {
        for (int j = 0; j < meas_dim; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                HPHt[i][j] += HP[i][k] * H[j][k];
            }
        }
    }
    
    double S[meas_dim][meas_dim];
    for (int i = 0; i < meas_dim; i++) {
        for (int j = 0; j < meas_dim; j++) {
            S[i][j] = HPHt[i][j] + R[i][j];
        }
    }
    
    // Kalman gain: K = P*H'*inv(S)
    double Sinv[meas_dim][meas_dim];
    bool inv_success = matrix_inverse_small<meas_dim>(S, Sinv);
    if (!inv_success) {
        // Skip update if matrix is singular
        return;
    }
    
    double PHt[STATE_DIM][meas_dim] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < meas_dim; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                PHt[i][j] += ekf->P[i][k] * H[j][k];
            }
        }
    }
    
    double K[STATE_DIM][meas_dim] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < meas_dim; j++) {
            for (int k = 0; k < meas_dim; k++) {
                K[i][j] += PHt[i][k] * Sinv[k][j];
            }
        }
    }
    
    // State update: x = x + K*z
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < meas_dim; j++) {
            ekf->x[i] += K[i][j] * z[j];
        }
    }
    
    // Normalize angles
    ekf->x[IDX_PHI] = normalize_angle(ekf->x[IDX_PHI]);
    ekf->x[IDX_THETA] = normalize_angle(ekf->x[IDX_THETA]);
    ekf->x[IDX_PSI] = normalize_angle(ekf->x[IDX_PSI]);
    
    // Covariance update using Joseph form for numerical stability
    // P = (I - K*H)*P*(I - K*H)' + K*R*K'
    double I_KH[STATE_DIM][STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            double sum = 0.0;
            for (int k = 0; k < meas_dim; k++) {
                sum += K[i][k] * H[k][j];
            }
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - sum;
        }
    }
    
    double P_temp1[STATE_DIM][STATE_DIM], P_temp2[STATE_DIM][STATE_DIM];
    double I_KHt[STATE_DIM][STATE_DIM];
    matrix_transpose(I_KH, I_KHt, STATE_DIM);
    matrix_multiply(I_KH, ekf->P, P_temp1, STATE_DIM);
    matrix_multiply(P_temp1, I_KHt, P_temp2, STATE_DIM);
    
    // K*R*K'
    double KR[STATE_DIM][meas_dim] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < meas_dim; j++) {
            for (int k = 0; k < meas_dim; k++) {
                KR[i][j] += K[i][k] * R[k][j];
            }
        }
    }
    
    double KRKt[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < meas_dim; k++) {
                KRKt[i][j] += KR[i][k] * K[j][k];
            }
        }
    }
    
    matrix_add(P_temp2, KRKt, ekf->P, STATE_DIM);
}

// EKF Update step with accelerometer/magnetometer for attitude correction
void ekf_update_attitude(ekf_state* ekf, double ax, double ay, double az,
                         double mx, double my, double mz) {
    const int meas_dim = 3;
    
    // Compute attitude from accelerometer (only if not in significant motion)
    double accel_mag = sqrt(ax*ax + ay*ay + az*az);
    
    // Skip if acceleration magnitude is too different from gravity (motion detected)
    if (fabs(accel_mag - GRAVITY) > 2.0) {
        return;  // Significant acceleration, don't trust accel for attitude
    }
    
    if (accel_mag < 0.1) {
        return;  // No significant acceleration
    }
    
    // Compute roll and pitch from accelerometer
    double phi_meas = atan2(ay, -az);
    double theta_meas = atan2(-ax, sqrt(ay*ay + az*az));
    
    // Compute tilt-compensated heading from magnetometer
    double phi = ekf->x[IDX_PHI];
    double theta = ekf->x[IDX_THETA];
    
    // Rotate magnetometer reading to horizontal plane
    double cp = cos(phi), sp = sin(phi);
    double ct = cos(theta), st = sin(theta);
    
    double mag_x = mx * ct - my * sp * st - mz * cp * st;
    double mag_y = my * cp - mz * sp;
    double mag_mag = sqrt(mag_x*mag_x + mag_y*mag_y);
    
    if (mag_mag < 0.1) {
        return;  // Invalid magnetometer reading
    }
    
    double psi_meas = atan2(-mag_y, mag_x);
    
    // Measurement matrix H (measuring attitude)
    double H[meas_dim][STATE_DIM] = {0};
    H[0][IDX_PHI] = 1.0;
    H[1][IDX_THETA] = 1.0;
    H[2][IDX_PSI] = 1.0;
    
    // Measurement noise covariance R
    double R[meas_dim][meas_dim] = {0};
    R[0][0] = 0.01;  // Roll noise (rad^2)
    R[1][1] = 0.01;  // Pitch noise (rad^2)
    R[2][2] = 0.05;  // Yaw noise (rad^2) - magnetometer less accurate
    
    // Innovation (normalized angle differences)
    double z[meas_dim];
    z[0] = normalize_angle(phi_meas - ekf->x[IDX_PHI]);
    z[1] = normalize_angle(theta_meas - ekf->x[IDX_THETA]);
    z[2] = normalize_angle(psi_meas - ekf->x[IDX_PSI]);
    
    // Innovation covariance: S = H*P*H' + R
    double HP[meas_dim][STATE_DIM] = {0};
    for (int i = 0; i < meas_dim; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            HP[i][j] = ekf->P[IDX_PHI + i][j];  // H is sparse
        }
    }
    
    double S[meas_dim][meas_dim] = {0};
    for (int i = 0; i < meas_dim; i++) {
        for (int j = 0; j < meas_dim; j++) {
            S[i][j] = ekf->P[IDX_PHI + i][IDX_PHI + j] + R[i][j];
        }
    }
    
    // Kalman gain: K = P*H'*inv(S)
    double Sinv[meas_dim][meas_dim];
    bool inv_success = matrix_inverse_small<meas_dim>(S, Sinv);
    if (!inv_success) {
        return;
    }
    
    // Kalman gain: K = P*H'*inv(S)
    double K[STATE_DIM][meas_dim] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < meas_dim; j++) {
            for (int k = 0; k < meas_dim; k++) {
                K[i][j] += ekf->P[i][IDX_PHI + k] * Sinv[k][j];
            }
        }
    }
    
    // State update: x = x + K*z
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < meas_dim; j++) {
            ekf->x[i] += K[i][j] * z[j];
        }
    }
    
    // Normalize angles
    ekf->x[IDX_PHI] = normalize_angle(ekf->x[IDX_PHI]);
    ekf->x[IDX_THETA] = normalize_angle(ekf->x[IDX_THETA]);
    ekf->x[IDX_PSI] = normalize_angle(ekf->x[IDX_PSI]);
    
    // Covariance update using Joseph form
    double I_KH[STATE_DIM][STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            double sum = 0.0;
            for (int k = 0; k < meas_dim; k++) {
                sum += K[i][k] * H[k][j];
            }
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - sum;
        }
    }
    
    double P_temp1[STATE_DIM][STATE_DIM], P_temp2[STATE_DIM][STATE_DIM];
    double I_KHt[STATE_DIM][STATE_DIM];
    matrix_transpose(I_KH, I_KHt, STATE_DIM);
    matrix_multiply(I_KH, ekf->P, P_temp1, STATE_DIM);
    matrix_multiply(P_temp1, I_KHt, P_temp2, STATE_DIM);
    
    // K*R*K'
    double KR[STATE_DIM][meas_dim] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < meas_dim; j++) {
            KR[i][j] = K[i][j] * R[j][j];  // R is diagonal
        }
    }
    
    double KRKt[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < meas_dim; k++) {
                KRKt[i][j] += KR[i][k] * K[j][k];
            }
        }
    }
    
    matrix_add(P_temp2, KRKt, ekf->P, STATE_DIM);
}

void* estimation_loop(void* arguments) {
    thread_struct* args = (thread_struct*)arguments;
    double* array = args->array;
    const air_config* cfg = args->cfg;
    std::map<std::string, int> keys = cfg->keys;
    int imu = 0, gps = 0;
    int max_sleep = hertz_to_microseconds(cfg->ESTIMATION_LOOP_RATE);
    uint64_t start_time, now, last_time;
    
    // GPS reference initialization
    gps_reference gps_ref;
    gps_ref.initialized = false;
    
    // Initialize Extended Kalman Filter
    ekf_state ekf;
    ekf_init(&ekf);
    
    // Previous timestamp
    last_time = current_time_microseconds();
    
    printf("Estimator_0: Extended Kalman Filter initialized (18 states)\n");
    printf("Estimator_0: Waiting for initial sensor data...\n");

    while (true) {
        start_time = current_time_microseconds();
        
        // Wait for new IMU data
        if (imu == array[keys["y_IMU_UPDATES"]]) {
            usleep(100);
            continue;
        }
        imu = array[keys["y_IMU_UPDATES"]];
        
        // Calculate time step
        double dt = (start_time - last_time) / 1000000.0;
        if (dt > 0.5 || dt <= 0.0) {
            dt = 1.0 / cfg->ESTIMATION_LOOP_RATE;
        }
        if (dt > 0.1) {
            dt = 0.1;  // Cap maximum dt for stability
        }
        last_time = start_time;

        ////////////////////////////////////////////////////////////////
        // READ SENSOR DATA (calibrated)
        ////////////////////////////////////////////////////////////////
        
        // Read calibrated IMU data (primary IMU)
        double ax = array[keys["y_IMU_1_AX_CALIB"]];
        double ay = array[keys["y_IMU_1_AY_CALIB"]];
        double az = array[keys["y_IMU_1_AZ_CALIB"]];
        double gyro_p = array[keys["y_IMU_1_GYRO_P_CALIB"]];
        double gyro_q = array[keys["y_IMU_1_GYRO_Q_CALIB"]];
        double gyro_r = array[keys["y_IMU_1_GYRO_R_CALIB"]];
        double mx = array[keys["y_IMU_1_MAG_X_CALIB"]];
        double my = array[keys["y_IMU_1_MAG_Y_CALIB"]];
        double mz = array[keys["y_IMU_1_MAG_Z_CALIB"]];

        // Stationary detection for drift prevention
        static const double ACC_NORM_TOL = 0.20;   // m/s^2 tolerance around 1g
        static const double GYRO_NORM_TOL = 0.02;  // rad/s
        double acc_norm = sqrt(ax*ax + ay*ay + az*az);
        double gyro_norm = sqrt(gyro_p*gyro_p + gyro_q*gyro_q + gyro_r*gyro_r);
        bool is_stationary = (fabs(acc_norm - GRAVITY) < ACC_NORM_TOL) && (gyro_norm < GYRO_NORM_TOL);

        ////////////////////////////////////////////////////////////////
        // EKF PREDICTION STEP
        ////////////////////////////////////////////////////////////////
        // Preserve previous state to prevent drift before GPS initialization
        double prev_x = ekf.x[IDX_X];
        double prev_y = ekf.x[IDX_Y];
        double prev_z = ekf.x[IDX_Z];
        double prev_vx = ekf.x[IDX_VX];
        double prev_vy = ekf.x[IDX_VY];
        double prev_vz = ekf.x[IDX_VZ];
        
        ekf_predict(&ekf, ax, ay, az, gyro_p, gyro_q, gyro_r, dt);

        // Before GPS initialization, only allow attitude estimation
        if (!gps_ref.initialized) {
            ekf.x[IDX_X] = prev_x;
            ekf.x[IDX_Y] = prev_y;
            ekf.x[IDX_Z] = prev_z;
            ekf.x[IDX_VX] = prev_vx;
            ekf.x[IDX_VY] = prev_vy;
            ekf.x[IDX_VZ] = prev_vz;
        }
        // If stationary, zero vertical velocity and freeze z to prevent drift
        else if (is_stationary) {
            ekf.x[IDX_VZ] = 0.0;
            ekf.x[IDX_Z] = prev_z;
        }

        ////////////////////////////////////////////////////////////////
        // EKF UPDATE STEP - ATTITUDE (every cycle)
        ////////////////////////////////////////////////////////////////
        
        ekf_update_attitude(&ekf, ax, ay, az, mx, my, mz);

        ////////////////////////////////////////////////////////////////
        // EKF UPDATE STEP - GPS (when available)
        ////////////////////////////////////////////////////////////////
        
        if (gps != array[keys["y_GPS_UPDATES"]]) {
            gps = array[keys["y_GPS_UPDATES"]];
            
            double gps_lat = array[keys["y_GPS_POSN_LAT"]];
            double gps_lon = array[keys["y_GPS_POSN_LON"]];
            double gps_alt = array[keys["y_GPS_POSN_ALT"]];
            double gps_vn = array[keys["y_GPS_VEL_N"]];
            double gps_ve = array[keys["y_GPS_VEL_E"]];
            double gps_vd = array[keys["y_GPS_VEL_D"]];
            int gps_status = (int)array[keys["y_GPS_STATUS"]];
            
            // Check for valid GPS fix (3D fix)
            if (gps_status >= 3 && fabs(gps_lat) > 0.001 && fabs(gps_lon) > 0.001) {
                
                // Initialize GPS reference on first valid fix
                if (!gps_ref.initialized) {
                    gps_ref.lat0 = gps_lat;
                    gps_ref.lon0 = gps_lon;
                    gps_ref.alt0 = gps_alt;
                    gps_ref.initialized = true;
                    
                    // Initialize EKF position with first GPS fix
                    ekf.x[IDX_X] = 0.0;
                    ekf.x[IDX_Y] = 0.0;
                    ekf.x[IDX_Z] = 0.0;
                    
                    printf("Estimator_0: GPS reference initialized at (%.6f, %.6f, %.1f)\n", 
                           gps_lat, gps_lon, gps_alt);
                }
                
                // Convert GPS position to NED coordinates
                double gps_x, gps_y, gps_z;
                gps_to_ned(gps_lat, gps_lon, gps_alt, gps_ref, &gps_x, &gps_y, &gps_z);
                
                // Update EKF with GPS measurements
                ekf_update_gps(&ekf, gps_x, gps_y, gps_z, gps_vn, gps_ve, gps_vd);
            }
        }

        ////////////////////////////////////////////////////////////////
        // EXTRACT STATE ESTIMATES FROM EKF
        ////////////////////////////////////////////////////////////////
        
        double x = ekf.x[IDX_X];
        double y = ekf.x[IDX_Y];
        double z = ekf.x[IDX_Z];
        double vx = ekf.x[IDX_VX];
        double vy = ekf.x[IDX_VY];
        double vz = ekf.x[IDX_VZ];
        double phi = ekf.x[IDX_PHI];
        double theta = ekf.x[IDX_THETA];
        double psi = ekf.x[IDX_PSI];
        // Use corrected rates (with bias removed) from state
        double p = ekf.x[IDX_P];
        double q = ekf.x[IDX_Q];
        double r = ekf.x[IDX_R];

        ////////////////////////////////////////////////////////////////
        // COMPUTE AIRSPEED PARAMETERS
        ////////////////////////////////////////////////////////////////
        
        // Total velocity magnitude
        double vt = sqrt(vx*vx + vy*vy + vz*vz);
        
        // Rotation matrix elements
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        double cos_psi = cos(psi);
        double sin_psi = sin(psi);
        
        // Transform velocity from NED to body frame
        double vx_body = cos_theta * cos_psi * vx + cos_theta * sin_psi * vy - sin_theta * vz;
        double vy_body = (sin_phi * sin_theta * cos_psi - cos_phi * sin_psi) * vx +
                         (sin_phi * sin_theta * sin_psi + cos_phi * cos_psi) * vy +
                         sin_phi * cos_theta * vz;
        double vz_body = (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) * vx +
                         (cos_phi * sin_theta * sin_psi - sin_phi * cos_psi) * vy +
                         cos_phi * cos_theta * vz;
        
        // Angle of attack and sideslip
        double alpha = 0.0, beta = 0.0;
        if (vt > 0.1) {
            alpha = atan2(vz_body, vx_body);
            beta = asin(clamp(vy_body / vt, -1.0, 1.0));
        }

        ////////////////////////////////////////////////////////////////
        // WRITE ESTIMATED STATE TO SHARED MEMORY
        ////////////////////////////////////////////////////////////////
        
        array[keys["xh_0_UPDATES"]]++;
        array[keys["xh_0_X"]] = x;
        array[keys["xh_0_Y"]] = y;
        array[keys["xh_0_Z"]] = z;
        array[keys["xh_0_VT"]] = vt;
        array[keys["xh_0_ALPHA"]] = alpha;
        array[keys["xh_0_BETA"]] = beta;
        array[keys["xh_0_PHI"]] = phi;
        array[keys["xh_0_THETA"]] = theta;
        array[keys["xh_0_PSI"]] = psi;
        array[keys["xh_0_P"]] = p;
        array[keys["xh_0_Q"]] = q;
        array[keys["xh_0_R"]] = r;

        ////////////////////////////////////////////////////////////////

        now = current_time_microseconds();
        int sleep_time = (int)(max_sleep - (now - start_time));
        usleep(std::max(sleep_time, 0));
    }
}
