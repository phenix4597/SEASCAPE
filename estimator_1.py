import time
import numpy as np
import struct
from core import shared_mem_helper_estimator_1 as helper

def estimator_loop(mem: helper.helper, max_sleep: float):
    # mem object is defined in 'core/shared_mem_helper_estimator_1.py'
    # mem.read_y() will return a list of all values defined in the y vector in config.json
    # mem.read_xh() will return a list of all values defined in the xh_1 vector in config.json
    # mem.write_xh() will require a list of all values defined in the xh_1 vector in config.json

    home_lat = None
    home_lon = None
    home_alt = None

    while True:
        t0 = time.time()

        # read latest sensor data
        # if you want to use IMU_2, be sure you have both IMUs enabled in config.json
        [
            NEW_GPS,
            IMU_1_AX,
            IMU_1_AY,
            IMU_1_AZ,
            IMU_1_GYRO_P,
            IMU_1_GYRO_Q,
            IMU_1_GYRO_R,
            IMU_1_MAG_X,
            IMU_1_MAG_Y,
            IMU_1_MAG_Z,
            IMU_2_AX,
            IMU_2_AY,
            IMU_2_AZ,
            IMU_2_GYRO_P,
            IMU_2_GYRO_Q,
            IMU_2_GYRO_R,
            IMU_2_MAG_X,
            IMU_2_MAG_Y,
            IMU_2_MAG_Z,
            BARO_PRES,
            GPS_POSN_LAT,
            GPS_POSN_LON,
            GPS_POSN_ALT,
            GPS_VEL_N,
            GPS_VEL_E,
            GPS_VEL_D,
            GPS_STATUS,
        ] = mem.read_y(use_calibrated_imu_data=True)


        # read previous xh data
        [X, Y, Z, VT, ALPHA, BETA, PHI, THETA, PSI, P, Q, R] = mem.read_xh()

        # read estimator_0 data
        [X_0, Y_0, Z_0, VT_0, ALPHA_0, BETA_0, PHI_0, THETA_0, PSI_0, P_0, Q_0, R_0] = mem.read_xh(xh_index=0)

        
        # output raw y values
        # print(f"y: {[NEW_GPS, IMU_1_AX, IMU_1_AY, IMU_1_AZ, IMU_1_GYRO_P, IMU_1_GYRO_Q, IMU_1_GYRO_R, IMU_1_MAG_X, IMU_1_MAG_Y, IMU_1_MAG_Z, IMU_2_AX, IMU_2_AY, IMU_2_AZ, IMU_2_GYRO_P, IMU_2_GYRO_Q, IMU_2_GYRO_R, IMU_2_MAG_X, IMU_2_MAG_Y, IMU_2_MAG_Z, BARO_PRES, GPS_POSN_LAT, GPS_POSN_LON, GPS_POSN_ALT, GPS_VEL_N, GPS_VEL_E, GPS_VEL_D, GPS_STATUS]}")

        # \/ \/ \/ \/ \/ \/ \/ \/ \/  ESTIMATOR CODE STARTS HERE  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ #

        # ## ==================== Complementary filter parameters
        dt = max_sleep  # time step
        alpha = 0.98  # complementary filter coefficient (higher = trust gyro more)
        
        # Integrate gyro rates to get attitude change
        PHI += IMU_1_GYRO_P * dt
        THETA += IMU_1_GYRO_Q * dt
        PSI += IMU_1_GYRO_R * dt
        
        # Calculate attitude from accelerometer
        roll_acc = np.arctan2(IMU_1_AY, np.sqrt(IMU_1_AX**2 + IMU_1_AZ**2))
        pitch_acc = np.arctan2(-IMU_1_AX, np.sqrt(IMU_1_AY**2 + IMU_1_AZ**2))
        
        # Apply complementary filter
        PHI = alpha * PHI + (1 - alpha) * roll_acc
        THETA = alpha * THETA + (1 - alpha) * pitch_acc
        
        # Calculate yaw from magnetometer (simplified, assumes level flight)
        mag_x_corrected = IMU_1_MAG_X * np.cos(THETA) + IMU_1_MAG_Z * np.sin(THETA)
        mag_y_corrected = (IMU_1_MAG_X * np.sin(PHI) * np.sin(THETA) + 
                          IMU_1_MAG_Y * np.cos(PHI) - 
                          IMU_1_MAG_Z * np.sin(PHI) * np.cos(THETA))
        yaw_mag = np.arctan2(-mag_y_corrected, mag_x_corrected)
        
        # Blend gyro yaw with magnetometer yaw
        PSI = alpha * PSI + (1 - alpha) * yaw_mag
        
        # Update body rates
        P = IMU_1_GYRO_P
        Q = IMU_1_GYRO_Q
        R = IMU_1_GYRO_R
        
        # Print attitude in degrees
        # print(f"Pitch: {np.degrees(PHI):.2f}°, Roll: {np.degrees(THETA):.2f}°, Yaw: {np.degrees(PSI):.2f}°")

        if NEW_GPS:
            # Check for valid 3D fix (status >= 3) and valid coordinates
            pass
        else:
            pass  # estimation without gps

        # /\ /\ /\ /\ /\ /\ /\ /\ /\  ESTIMATOR CODE STOPS HERE  /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ #

        
        
        # publish new xh data
        mem.write_xh([X, Y, Z, VT, ALPHA, BETA, PHI, THETA, PSI, P, Q, R])

        # sleep to maintain frequency
        sleep_time = max_sleep - (time.time() - t0)
        time.sleep(max(sleep_time, 0))
